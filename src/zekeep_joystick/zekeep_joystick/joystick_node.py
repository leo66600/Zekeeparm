from __future__ import annotations

import math
import time

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from zekeep_msgs.srv import GripperCommand
from zekeep_msgs.action import GripperGrasp

from .mapping import integrate_positions, joint_velocities


class JoystickController(Node):
    """Map an Xbox-style gamepad to the guarded Zekeep servo stream."""

    def __init__(self) -> None:
        super().__init__("zekeep_joystick")
        defaults = {
            "arm_namespace": "zekeep",
            "joy_topic": "/joy",
            "joint_names": [f"joint{i}" for i in range(1, 7)],
            "axis_indices": [0, 1, 4, 3, 6, 7],
            "axis_signs": [1.0] * 6,
            "max_speeds": [0.25] * 6,
            "lower_limits": [-2.58, 0.0, 0.0, -1.57, -1.57, -1.57],
            "upper_limits": [2.58, 3.7, 3.7, 1.57, 1.57, 1.57],
            "deadman_button": 4,
            "close_button": 0,
            "open_button": 1,
            "deadzone": 0.12,
            "publish_rate": 50.0,
            "joy_timeout": 0.3,
            "grasp_closing_torque": 0.0,
            "grasp_hold_torque": 0.0,
            "grasp_timeout": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.axis_indices = list(self.get_parameter("axis_indices").value)
        self.axis_signs = list(self.get_parameter("axis_signs").value)
        self.max_speeds = list(self.get_parameter("max_speeds").value)
        self.lower_limits = list(self.get_parameter("lower_limits").value)
        self.upper_limits = list(self.get_parameter("upper_limits").value)
        self.deadman_button = int(self.get_parameter("deadman_button").value)
        self.close_button = int(self.get_parameter("close_button").value)
        self.open_button = int(self.get_parameter("open_button").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.joy_timeout = float(self.get_parameter("joy_timeout").value)
        self.grasp_options = [float(self.get_parameter(name).value) for name in (
            "grasp_closing_torque", "grasp_hold_torque", "grasp_timeout"
        )]
        if not all(math.isfinite(value) and value >= 0.0 for value in self.grasp_options):
            raise ValueError("grasp torque and timeout parameters must be finite and nonnegative")
        self._validate_parameters()

        root = f"/{self.namespace}"
        self.publisher = self.create_publisher(
            JointTrajectory, root + "/servo_joint_trajectory", 10
        )
        self.start_client = self.create_client(Trigger, root + "/servo/start")
        self.stop_client = self.create_client(Trigger, root + "/servo/stop")
        self.open_client = self.create_client(GripperCommand, root + "/gripper/open")
        self.close_client = ActionClient(self, GripperGrasp, root + "/gripper/grasp")
        self.create_subscription(
            Joy,
            str(self.get_parameter("joy_topic").value),
            self._on_joy,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            root + "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self.publish_rate, self._tick)

        self.axes: list[float] | None = None
        self.buttons: list[int] = []
        self.last_joy_time = 0.0
        self.feedback: list[float] | None = None
        self.last_feedback_time = 0.0
        self.targets: list[float] | None = None
        self.state = "idle"
        self.pending_gripper = None
        self.require_deadman_release = False
        self.last_tick = time.monotonic()
        self.get_logger().info(
            "ready; manually enable the arm, then hold LB to move"
        )

    def _validate_parameters(self) -> None:
        vectors = (
            self.joint_names,
            self.axis_indices,
            self.axis_signs,
            self.max_speeds,
            self.lower_limits,
            self.upper_limits,
        )
        if any(len(values) != 6 for values in vectors):
            raise ValueError("joint mapping vectors must each contain six values")
        if len(set(self.joint_names)) != 6:
            raise ValueError("joint_names must contain six unique names")
        if not self.namespace:
            raise ValueError("arm_namespace must not be empty")
        if any(index < 0 for index in self.axis_indices):
            raise ValueError("axis_indices must be nonnegative")
        if min(self.deadman_button, self.close_button, self.open_button) < 0:
            raise ValueError("button indices must be nonnegative")
        if any(speed <= 0.0 for speed in self.max_speeds):
            raise ValueError("max_speeds must be positive")
        numeric_values = (
            self.axis_signs
            + self.max_speeds
            + self.lower_limits
            + self.upper_limits
            + [self.deadzone, self.publish_rate, self.joy_timeout]
        )
        if not all(math.isfinite(value) for value in numeric_values):
            raise ValueError("numeric parameters must be finite")
        if any(
            lower >= upper
            for lower, upper in zip(self.lower_limits, self.upper_limits)
        ):
            raise ValueError("each lower limit must be below its upper limit")
        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError("deadzone must be in [0, 1)")
        if self.publish_rate <= 0.0 or self.joy_timeout <= 0.0:
            raise ValueError("publish_rate and joy_timeout must be positive")

    def _on_joint_state(self, message: JointState) -> None:
        by_name = dict(zip(message.name, message.position))
        positions = [float(by_name[name]) for name in self.joint_names] if all(
            name in by_name for name in self.joint_names
        ) else []
        if positions and all(math.isfinite(position) for position in positions):
            self.feedback = positions
            self.last_feedback_time = time.monotonic()

    def _on_joy(self, message: Joy) -> None:
        previous = self.buttons
        self.axes = list(message.axes)
        self.buttons = list(message.buttons)
        self.last_joy_time = time.monotonic()
        deadman = self._button(self.deadman_button)
        if not deadman:
            self.require_deadman_release = False
        if deadman and not (self._button(self.close_button) and self._button(self.open_button)):
            if self._rising(previous, self.close_button):
                self._request_gripper(self.close_client, "close")
            elif self._rising(previous, self.open_button):
                self._request_gripper(self.open_client, "open")

    def _button(self, index: int) -> bool:
        return index < len(self.buttons) and bool(self.buttons[index])

    def _rising(self, previous: list[int], index: int) -> bool:
        return self._button(index) and (index >= len(previous) or not previous[index])

    def _tick(self) -> None:
        now = time.monotonic()
        fresh = now - self.last_joy_time <= self.joy_timeout
        deadman = fresh and self._button(self.deadman_button)

        if self.state == "active" and not deadman:
            self._stop_stream("joystick timed out" if not fresh else "deadman released")
            return
        if (
            self.state == "idle"
            and deadman
            and not self.require_deadman_release
            and self.feedback is not None
            and now - self.last_feedback_time <= self.joy_timeout
        ):
            self._start_stream()
            return
        if self.state != "active" or self.axes is None or self.targets is None:
            self.last_tick = now
            return
        if len(self.axes) <= max(self.axis_indices):
            self.get_logger().error("Joy message has too few axes; stopping")
            self._stop_stream("invalid joystick mapping")
            return

        dt = min(now - self.last_tick, 2.0 / self.publish_rate)
        self.last_tick = now
        velocities = joint_velocities(
            self.axes,
            self.axis_indices,
            self.axis_signs,
            self.max_speeds,
            self.deadzone,
        )
        self.targets, velocities = integrate_positions(
            self.targets,
            velocities,
            self.lower_limits,
            self.upper_limits,
            dt,
        )
        message = JointTrajectory()
        message.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.targets
        point.velocities = velocities
        message.points = [point]
        self.publisher.publish(message)

    def _start_stream(self) -> None:
        if not self.start_client.service_is_ready():
            self.get_logger().warn("servo/start unavailable")
            self.require_deadman_release = True
            return
        self.state = "starting"
        future = self.start_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_started)

    def _on_started(self, future) -> None:
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            self.targets = list(self.feedback)
            self.last_tick = time.monotonic()
            self.state = "active"
            self.get_logger().info("joystick control active")
        except Exception as exc:
            self.state = "idle"
            self.require_deadman_release = True
            self.get_logger().error(f"cannot start joystick control: {exc}")

    def _stop_stream(self, reason: str) -> None:
        self.state = "stopping"
        self.targets = None
        if not self.stop_client.service_is_ready():
            self.pending_gripper = None
            self.state = "idle"
            self.require_deadman_release = True
            self.get_logger().error(
                f"servo/stop unavailable after {reason}; driver watchdog will hold"
            )
            return
        future = self.stop_client.call_async(Trigger.Request())
        future.add_done_callback(lambda result: self._on_stopped(result, reason))

    def _on_stopped(self, future, reason: str) -> None:
        pending = self.pending_gripper
        self.pending_gripper = None
        self.state = "idle"
        self.require_deadman_release = True
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            self.get_logger().info(f"joystick control stopped: {reason}")
        except Exception as exc:
            self.get_logger().error(f"servo stop failed; driver watchdog will hold: {exc}")
            return
        if pending is not None:
            self._command_gripper(*pending)

    def _request_gripper(self, client, label: str) -> None:
        if self.state == "active":
            self.pending_gripper = (client, label)
            self._stop_stream(f"before gripper {label}")
        elif self.state == "idle":
            self._command_gripper(client, label)
        else:
            self.get_logger().warn("control transition or gripper busy; press A/B again when idle")

    def _command_gripper(self, client, label: str) -> None:
        if label == "close":
            self._start_grasp(client)
            return
        if not client.service_is_ready():
            self.get_logger().warn(f"gripper/{label} unavailable")
            return
        request = GripperCommand.Request()
        request.position = 0.0
        request.timeout = 3.0
        self.state = "gripper"
        self.require_deadman_release = True
        try:
            client.call_async(request).add_done_callback(
                lambda future: self._on_gripper_done(future, label)
            )
        except Exception as exc:
            self.state = "idle"
            self.get_logger().error(f"gripper/{label} request failed: {exc}")

    def _on_gripper_done(self, future, label: str) -> None:
        self.state = "idle"
        self.require_deadman_release = True
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            self.get_logger().info(f"gripper/{label} complete; release LB before joint control")
        except Exception as exc:
            self.get_logger().error(f"gripper/{label} failed: {exc}")

    def _start_grasp(self, client) -> None:
        if not client.server_is_ready():
            self.get_logger().warn("gripper/grasp action unavailable")
            return
        goal = GripperGrasp.Goal()
        goal.closing_torque, goal.hold_torque, goal.timeout = self.grasp_options
        self.state = "gripper"
        self.require_deadman_release = True
        try:
            client.send_goal_async(goal).add_done_callback(self._on_grasp_accepted)
        except Exception as exc:
            self._grasp_failed(exc)

    def _on_grasp_accepted(self, future) -> None:
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError("grasp rejected by driver")
            handle.get_result_async().add_done_callback(self._on_grasp_result)
            self.get_logger().info("gripper closing with torque; waiting for contact")
        except Exception as exc:
            self._grasp_failed(exc)

    def _on_grasp_result(self, future) -> None:
        self.state = "idle"
        self.require_deadman_release = True
        try:
            wrapped = future.result()
            result = wrapped.result
            if (wrapped.status != GoalStatus.STATUS_SUCCEEDED
                    or not result.success or not result.object_detected):
                raise RuntimeError(result.message)
            self.get_logger().info(
                "object detected; holding torque active. Release LB before joint control; LB+B opens"
            )
        except Exception as exc:
            self._grasp_failed(exc)

    def _grasp_failed(self, exc) -> None:
        self.state = "idle"
        self.require_deadman_release = True
        self.get_logger().error(f"gripper/grasp failed: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoystickController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
