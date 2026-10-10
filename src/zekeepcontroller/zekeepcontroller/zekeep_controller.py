from __future__ import annotations

import signal
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from moveit_msgs.srv import GetStateValidity
from rcl_interfaces.msg import ParameterDescriptor
from zekeep_msgs.srv import WebTaskLease
from .web_task_gate import WebTaskGate

from .hardware_manager import HardwareManager
from .motor_passthrough import MotorPassthrough
from .ros_actions import ArmActions
from .ros_publishers import JointStatePublisher
from .ros_services import ArmServices
from .servo_stream import ServoStream


class ZekeepController(Node):
    """ROS 2 node that wires hardware, publishers, services, and actions."""
    def __init__(self) -> None:
        super().__init__("ZekeepController")

        self.reentrant_group = ReentrantCallbackGroup()
        self.slow_group = MutuallyExclusiveCallbackGroup()
        self.sensor_qos = qos_profile_sensor_data

        self.declare_parameter("hardware_config", "")
        self.declare_parameter("model", "")
        self.declare_parameter("channel", "")
        self.declare_parameter("auto_enable", False)
        self.declare_parameter("joint_state_rate", 100.0)
        self.declare_parameter("arm_namespace", "zekeep")
        self.declare_parameter("cmd_arbitration", "reject")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ee_frame_id", "link6")
        self.declare_parameter("disable_after_safe_home", True)

        hardware_config = self.get_parameter("hardware_config").value or None
        model = str(self.get_parameter("model").value or "")
        channel = str(self.get_parameter("channel").value or "")
        self.arm_namespace = str(self.get_parameter("arm_namespace").value or "zekeep").strip("/")
        joint_state_rate = float(self.get_parameter("joint_state_rate").value)
        cmd_arbitration = str(self.get_parameter("cmd_arbitration").value or "reject")
        self.disable_after_safe_home = bool(
            self.get_parameter("disable_after_safe_home").value
        )
        if cmd_arbitration not in ("reject", "preempt"):
            self.get_logger().warn(
                f"unsupported cmd_arbitration={cmd_arbitration!r}; using 'reject'"
            )
            cmd_arbitration = "reject"

        self.hardware = HardwareManager(
            hardware_config=hardware_config,
            model=model,
            channel=channel,
            auto_enable=bool(self.get_parameter("auto_enable").value),
        )
        # Report the resolved calibration; launch overrides must not replace it.
        for name, value in (
            ("web_gripper_open_rad", self.hardware.gripper_open_position),
            ("web_gripper_close_rad", self.hardware.gripper_close_position),
            ("web_gripper_max_width_m", self.hardware.gripper_max_width),
        ):
            self.declare_parameter(
                name, float(value), ParameterDescriptor(read_only=True),
                ignore_override=True,
            )
        self._validity_client = self.create_client(
            GetStateValidity, "/check_state_validity", callback_group=self.reentrant_group
        )
        self.hardware.validate_home_path = self.validate_home_path
        self.hardware.connect()

        self.joint_state_publisher = JointStatePublisher(
            self,
            self.hardware,
            self.arm_namespace,
            joint_state_rate,
        )
        self.web_task_gate = WebTaskGate(self.hardware)
        self.create_service(WebTaskLease, f"/{self.arm_namespace}/web_task/lease",
                            self.web_task_gate.lease, callback_group=self.reentrant_group)
        self.create_timer(0.1, self.check_web_task_lease, callback_group=self.reentrant_group)
        self.task_services = ArmServices(self, self.hardware, self.arm_namespace + "/web_task", internal=True)
        self.task_actions = ArmActions(self, self.hardware, self.arm_namespace + "/web_task", internal=True)
        self.arm_services = ArmServices(self, self.hardware, self.arm_namespace)
        self.arm_actions = ArmActions(self, self.hardware, self.arm_namespace)
        self.motor_passthrough = MotorPassthrough(
            self,
            self.hardware,
            self.arm_namespace,
            cmd_arbitration,
        )
        self.servo_stream = ServoStream(self, self.hardware, self.arm_namespace)

        self.get_logger().info(
            f"ZekeepController started: namespace=/{self.arm_namespace}, "
            f"joints={self.hardware.joint_names}"
        )

    def publish_arm_status(self, *, read_hardware: bool = True) -> None:
        """Publish current controller status, optionally reading motor feedback."""
        self.joint_state_publisher.publish_status(read_hardware=read_hardware)

    def check_web_task_lease(self) -> None:
        try:
            self.web_task_gate.watchdog()
        except Exception as exc:
            self.get_logger().error(f"web task hold unconfirmed; ownership retained: {exc}")

    def validate_home_path(self, samples) -> None:
        """Fail closed when the planning scene cannot validate the sampled path."""
        if not self._validity_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("MoveIt /check_state_validity unavailable; keep controller running")
        deadline = time.monotonic() + 30.0
        for index, positions in enumerate(samples):
            if self.hardware._home_cancel.is_set():
                raise RuntimeError("safe_home interrupted during path validation")
            request = GetStateValidity.Request()
            request.robot_state.is_diff = True
            request.robot_state.joint_state.name = list(self.hardware.joint_names)
            request.robot_state.joint_state.position = [float(value) for value in positions]
            future = self._validity_client.call_async(request)
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            while not done.wait(0.02):
                if time.monotonic() >= deadline or self.hardware._home_cancel.is_set():
                    future.cancel()
                    raise RuntimeError("safe_home collision validation timed out or interrupted")
            response = future.result()
            if response is None or not response.valid:
                contact = response.contacts[0] if response and response.contacts else None
                detail = (
                    f": {contact.contact_body_1} / {contact.contact_body_2} "
                    f"(depth {contact.depth:.6f} m)"
                    if contact else ""
                )
                raise RuntimeError(
                    f"safe_home path rejected by MoveIt at sample "
                    f"{index + 1}/{len(samples)}{detail}"
                )

    def shutdown(self) -> None:
        """Stop active control and release hardware resources."""
        self.hardware.shutdown(
            disable_after_safe_home=self.disable_after_safe_home,
        )


def main(args=None) -> None:
    # Keep ROS and motor holding control alive after a failed exit attempt.
    # launch must not escalate a failed graceful exit to SIGKILL.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    requested = threading.Event()
    previous = {sig: signal.signal(sig, lambda *_: requested.set())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    node = ZekeepController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    while True:
        requested.wait()
        requested.clear()
        try:
            node.shutdown()
        except Exception as exc:
            node.get_logger().error(
                f"{exc}. Controller retained; correct fault then request exit again."
            )
            continue
        break
    executor.shutdown()
    worker.join()
    node.destroy_node()
    rclpy.shutdown()
    for sig, handler in previous.items():
        signal.signal(sig, handler)


if __name__ == "__main__":
    main()
