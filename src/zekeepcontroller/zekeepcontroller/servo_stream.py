from __future__ import annotations

import time
import threading

from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory


class ServoStream:
    """Guarded latest-command adapter to the SDK loop."""

    def __init__(self, node: Node, hardware, namespace: str):
        self.node = node
        self.hardware = hardware
        self.names = tuple(hardware.joint_names)
        node.declare_parameter("servo_command_timeout_s", 0.2)
        self.timeout = float(node.get_parameter("servo_command_timeout_s").value)
        if not 0.05 <= self.timeout <= 1.0:
            raise ValueError("servo_command_timeout_s must be in [0.05, 1.0]")
        self.last_command = None
        self._lock = threading.Lock()
        root = f"/{namespace}/servo"
        self.start_service = node.create_service(Trigger, root + "/start", node.web_task_gate.service(self.start))
        self.stop_service = node.create_service(Trigger, root + "/stop", self.stop)
        self.require_mit_service = node.create_service(
            Trigger, root + "/require_mit", node.web_task_gate.service(self.require_mit)
        )
        self.require_posvel_service = node.create_service(
            Trigger, root + "/require_posvel", node.web_task_gate.service(self.require_posvel)
        )
        self.subscription = node.create_subscription(
            JointTrajectory,
            f"/{namespace}/servo_joint_trajectory",
            self.command,
            10,
            callback_group=node.reentrant_group,
        )
        self.timer = node.create_timer(0.02, self.watchdog, callback_group=node.reentrant_group)

    def start(self, _request, response):
        with self._lock:
            try:
                self.hardware.begin_servo_stream()
                self.last_command = time.monotonic()
                response.success, response.message = True, "servo stream started"
            except Exception as exc:
                response.success, response.message = False, str(exc)
        return response

    def require_mit(self, _request, response):
        response.success = self.hardware._arm_control_mode == "mit"
        response.message = (
            "MIT mode configured" if response.success
            else "follow requires MIT; restart bringup with zekeep_hardware_withnogravity.yaml"
        )
        return response

    def require_posvel(self, _request, response):
        response.success = self.hardware._arm_control_mode == "posvel"
        response.message = (
            "POS_VEL mode configured" if response.success
            else "follow requires POS_VEL; restart bringup with zekeep_hardware.yaml"
        )
        return response

    def stop(self, _request, response):
        with self._lock:
            self.hardware.stop_servo_stream()
            self.last_command = None
        response.success, response.message = True, "servo stream stopped; holding position"
        return response

    def command(self, msg: JointTrajectory):
        with self._lock:
            if self.hardware.state_machine != "SERVO_RUNNING":
                return
            try:
                # A single point replaces the prior servo target immediately.
                if len(msg.points) != 1 or len(msg.joint_names) != len(self.names):
                    raise ValueError("servo command must contain one point for six joints")
                if len(set(msg.joint_names)) != len(self.names) or set(msg.joint_names) != set(self.names):
                    raise ValueError("servo command joint names do not match arm")
                point = msg.points[0]
                if len(point.positions) != len(self.names) or len(point.velocities) != len(self.names):
                    raise ValueError("servo point must contain six positions and velocities")
                by_name = {name: i for i, name in enumerate(msg.joint_names)}
                positions = [point.positions[by_name[name]] for name in self.names]
                velocities = [point.velocities[by_name[name]] for name in self.names]
                self.hardware.set_servo_target(positions, velocities)
                self.last_command = time.monotonic()
            except RuntimeError as exc:
                if (str(exc) == "servo command received while servo stream is inactive"
                        and self.hardware.state_machine != "SERVO_RUNNING"):
                    self.last_command = None
                    return
                self._reject(exc)
            except Exception as exc:
                self._reject(exc)

    def _reject(self, exc):
        self.node.get_logger().error(f"servo command rejected; holding: {exc}")
        self.hardware.stop_servo_stream()
        self.last_command = None

    def watchdog(self):
        with self._lock:
            if (self.hardware.state_machine == "SERVO_RUNNING" and self.last_command is not None
                    and time.monotonic() - self.last_command > self.timeout):
                self.node.get_logger().error("servo command timeout; holding arm")
                self.hardware.stop_servo_stream()
                self.last_command = None
