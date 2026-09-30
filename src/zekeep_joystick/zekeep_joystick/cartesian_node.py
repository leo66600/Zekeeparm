from __future__ import annotations

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from zekeep_msgs.action import GripperGrasp
from zekeep_msgs.srv import GripperCommand, MoveToPoseIK

from .mapping import integrate_cartesian_pose, integrate_positions, joint_velocities


class CartesianJoystickController(Node):
    """Jog link6 in the base frame through IK and the guarded servo stream."""

    def __init__(self) -> None:
        super().__init__("zekeep_cartesian_joystick")
        defaults = {
            "arm_namespace": "zekeep",
            "joy_topic": "/joy",
            "base_frame": "base_link",
            "end_frame": "link6",
            "joint_names": [f"joint{i}" for i in range(1, 7)],
            # X, Y, Z, roll, pitch, yaw: LY, LX, RY, D-pad Y, D-pad X, RX.
            "axis_indices": [1, 0, 3, 7, 6, 2],
            "axis_signs": [1.0] * 6,
            "max_speeds": [0.04, 0.04, 0.04, 0.25, 0.25, 0.25],
            "joint_axis_indices": [0, 1, 3, 2, 6, 7],
            "joint_axis_signs": [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            "joint_max_speeds": [0.25] * 6,
            "joint_lower_limits": [-2.58, 0.0, 0.0, -1.57, -1.57, -1.57],
            "joint_upper_limits": [2.58, 3.7, 3.7, 1.57, 1.57, 1.57],
            "deadman_button": 6,
            "close_button": 0,
            "open_button": 1,
            "safe_home_button": 3,
            "joint_mode_button": 2,
            "deadzone": 0.12,
            "publish_rate": 15.0,
            "joy_timeout": 0.3,
            "max_joint_step": 0.15,
            "max_joint_speed": 0.5,
            "grasp_closing_torque": 0.0,
            "grasp_hold_torque": 0.0,
            "grasp_timeout": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        value = lambda name: self.get_parameter(name).value
        self.namespace = str(value("arm_namespace")).strip("/")
        self.base_frame = str(value("base_frame"))
        self.end_frame = str(value("end_frame"))
        self.joint_names = list(value("joint_names"))
        self.axis_indices = list(value("axis_indices"))
        self.axis_signs = list(value("axis_signs"))
        self.max_speeds = list(value("max_speeds"))
        self.joint_axis_indices = list(value("joint_axis_indices"))
        self.joint_axis_signs = list(value("joint_axis_signs"))
        self.joint_max_speeds = list(value("joint_max_speeds"))
        self.joint_lower_limits = list(value("joint_lower_limits"))
        self.joint_upper_limits = list(value("joint_upper_limits"))
        self.deadman_button = int(value("deadman_button"))
        self.close_button = int(value("close_button"))
        self.open_button = int(value("open_button"))
        self.safe_home_button = int(value("safe_home_button"))
        self.joint_mode_button = int(value("joint_mode_button"))
        self.deadzone = float(value("deadzone"))
        self.publish_rate = float(value("publish_rate"))
        self.joy_timeout = float(value("joy_timeout"))
        self.max_joint_step = float(value("max_joint_step"))
        self.max_joint_speed = float(value("max_joint_speed"))
        self.grasp_options = [float(value(name)) for name in (
            "grasp_closing_torque", "grasp_hold_torque", "grasp_timeout"
        )]
        self._validate_parameters()

        root = f"/{self.namespace}"
        self.publisher = self.create_publisher(
            JointTrajectory, root + "/servo_joint_trajectory", 10
        )
        self.start_client = self.create_client(Trigger, root + "/servo/start")
        self.stop_client = self.create_client(Trigger, root + "/servo/stop")
        self.safe_home_client = self.create_client(Trigger, root + "/safe_home")
        self.ik_client = self.create_client(MoveToPoseIK, root + "/move_to_pose_ik")
        self.open_client = self.create_client(GripperCommand, root + "/gripper/open")
        self.close_client = ActionClient(self, GripperGrasp, root + "/gripper/grasp")
        self.create_subscription(
            Joy, str(value("joy_topic")), self._on_joy, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState, root + "/joint_states", self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / self.publish_rate, self._tick)

        self.axes: list[float] | None = None
        self.buttons: list[int] = []
        self.feedback: list[float] | None = None
        self.target_joints: list[float] | None = None
        self.target_pose: list[float] | None = None
        self.pending_pose: list[float] | None = None
        self.last_joy_time = 0.0
        self.last_feedback_time = 0.0
        self.last_tick = time.monotonic()
        self.ik_pending = False
        self.control_mode = "cartesian"
        self.state = "idle"
        self.pending_after_stop = None
        self.require_deadman_release = False
        self.last_warning = 0.0
        self.get_logger().info(
            "ready; manually enable arm, then hold LB for Cartesian control"
        )

    def _validate_parameters(self) -> None:
        vectors = (
            self.joint_names, self.axis_indices, self.axis_signs, self.max_speeds,
            self.joint_axis_indices, self.joint_axis_signs, self.joint_max_speeds,
            self.joint_lower_limits, self.joint_upper_limits,
        )
        if any(len(values) != 6 for values in vectors):
            raise ValueError("control mapping vectors must each contain six values")
        numbers = (self.axis_signs + self.max_speeds + self.joint_axis_signs
                   + self.joint_max_speeds + self.joint_lower_limits
                   + self.joint_upper_limits + [
            self.deadzone, self.publish_rate, self.joy_timeout,
            self.max_joint_step, self.max_joint_speed,
        ] + self.grasp_options)
        if (not self.namespace or len(set(self.joint_names)) != 6
                or any(index < 0 for index in self.axis_indices + self.joint_axis_indices)
                or min(self.deadman_button, self.close_button, self.open_button,
                       self.safe_home_button, self.joint_mode_button) < 0
                or not all(math.isfinite(number) for number in numbers)
                or any(value < 0.0 for value in self.grasp_options)
                or any(speed <= 0.0 for speed in self.max_speeds + self.joint_max_speeds)
                or any(lower >= upper for lower, upper in zip(
                    self.joint_lower_limits, self.joint_upper_limits
                ))
                or not 0.0 <= self.deadzone < 1.0
                or min(self.publish_rate, self.joy_timeout,
                       self.max_joint_step, self.max_joint_speed) <= 0.0):
            raise ValueError("invalid Cartesian joystick parameters")

    def _on_joy(self, message: Joy) -> None:
        previous = self.buttons
        self.axes = list(message.axes)
        self.buttons = list(message.buttons)
        self.last_joy_time = time.monotonic()
        deadman = self._deadman()
        if not deadman:
            self.require_deadman_release = False
        if self._rising(previous, self.safe_home_button):
            self._request_operation(self._safe_home, "before safe_home")
        elif deadman:
            if self._rising(previous, self.joint_mode_button):
                self._request_operation(self._switch_to_joint_mode, "before joint mode")
            elif (not (self._button(self.close_button) and self._button(self.open_button))
                    and self._rising(previous, self.close_button)):
                self._request_gripper(self.close_client, "close")
            elif (not (self._button(self.close_button) and self._button(self.open_button))
                  and self._rising(previous, self.open_button)):
                self._request_gripper(self.open_client, "open")

    def _on_joint_state(self, message: JointState) -> None:
        by_name = dict(zip(message.name, message.position))
        positions = [float(by_name[name]) for name in self.joint_names] if all(
            name in by_name for name in self.joint_names
        ) else []
        if positions and all(math.isfinite(position) for position in positions):
            self.feedback = positions
            self.last_feedback_time = time.monotonic()

    def _deadman(self) -> bool:
        return self._button(self.deadman_button)

    def _button(self, index: int) -> bool:
        return index < len(self.buttons) and bool(self.buttons[index])

    def _rising(self, previous: list[int], index: int) -> bool:
        return self._button(index) and (index >= len(previous) or not previous[index])

    def _current_pose(self) -> list[float]:
        transform = self.tf_buffer.lookup_transform(
            self.base_frame, self.end_frame, rclpy.time.Time()
        ).transform
        rotation = transform.rotation
        roll, pitch, yaw = euler_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )
        return [
            transform.translation.x, transform.translation.y,
            transform.translation.z, roll, pitch, yaw,
        ]

    def _tick(self) -> None:
        now = time.monotonic()
        fresh = now - self.last_joy_time <= self.joy_timeout
        deadman = fresh and self._deadman()
        if self.state == "active" and not deadman:
            self._stop("joystick timed out" if not fresh else "deadman released")
            return
        if (self.state == "idle" and deadman and not self.require_deadman_release
                and self.feedback is not None
                and now - self.last_feedback_time <= self.joy_timeout):
            self._start()
            return
        if self.state != "active" or self.axes is None or self.target_joints is None:
            self.last_tick = now
            return
        if self.control_mode == "joint":
            self._tick_joint(now)
            return
        if self.target_pose is None:
            self.last_tick = now
            return
        self._publish_joint_target(self.target_joints, [0.0] * 6)
        if self.ik_pending:
            return
        if len(self.axes) <= max(self.axis_indices):
            self._stop("Joy message has too few axes")
            return

        dt = min(now - self.last_tick, 2.0 / self.publish_rate)
        self.last_tick = now
        velocities = joint_velocities(
            self.axes, self.axis_indices, self.axis_signs,
            self.max_speeds, self.deadzone,
        )
        if not any(velocities):
            return
        candidate = integrate_cartesian_pose(self.target_pose, velocities, dt)
        request = MoveToPoseIK.Request()
        request.target_pose = self._pose_message(candidate)
        if not self.ik_client.service_is_ready():
            self._warn("move_to_pose_ik unavailable")
            return
        self.pending_pose = candidate
        self.ik_pending = True
        self.ik_client.call_async(request).add_done_callback(self._on_ik)

    def _tick_joint(self, now: float) -> None:
        if len(self.axes) <= max(self.joint_axis_indices):
            self._stop("Joy message has too few axes")
            return
        dt = min(now - self.last_tick, 2.0 / self.publish_rate)
        self.last_tick = now
        velocities = joint_velocities(
            self.axes, self.joint_axis_indices, self.joint_axis_signs,
            self.joint_max_speeds, self.deadzone,
        )
        self.target_joints, velocities = integrate_positions(
            self.target_joints, velocities, self.joint_lower_limits,
            self.joint_upper_limits, dt,
        )
        self._publish_joint_target(self.target_joints, velocities)

    @staticmethod
    def _pose_message(values: list[float]) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = values[:3]
        quaternion = quaternion_from_euler(*values[3:])
        pose.orientation.x, pose.orientation.y = quaternion[:2]
        pose.orientation.z, pose.orientation.w = quaternion[2:]
        return pose

    def _start(self) -> None:
        if not self.start_client.service_is_ready():
            self._warn("servo/start unavailable")
            self.require_deadman_release = True
            return
        if self.control_mode == "cartesian":
            try:
                self.target_pose = self._current_pose()
            except TransformException as exc:
                self._warn(f"TF unavailable: {exc}")
                self.require_deadman_release = True
                return
        self.state = "starting"
        self.start_client.call_async(Trigger.Request()).add_done_callback(self._on_started)

    def _on_started(self, future) -> None:
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            self.target_joints = list(self.feedback)
            self.last_tick = time.monotonic()
            self.state = "active"
            self.get_logger().info(f"{self.control_mode} joystick control active")
        except Exception as exc:
            self.state = "idle"
            self.require_deadman_release = True
            self.get_logger().error(f"cannot start {self.control_mode} control: {exc}")

    def _on_ik(self, future) -> None:
        self.ik_pending = False
        if self.state != "active":
            return
        try:
            response = future.result()
            solution = [float(value) for value in response.q_solution]
            if not response.success:
                raise RuntimeError(response.message)
            if len(solution) != 6 or not all(math.isfinite(value) for value in solution):
                raise RuntimeError("IK returned an invalid joint vector")
            delta = [value - old for value, old in zip(solution, self.target_joints)]
            if max(abs(value) for value in delta) > self.max_joint_step:
                raise RuntimeError("IK solution exceeds max_joint_step")
            period = 1.0 / self.publish_rate
            velocities = [
                max(-self.max_joint_speed, min(self.max_joint_speed, value / period))
                for value in delta
            ]
            self._publish_joint_target(solution, velocities)
            self.target_joints = solution
            self.target_pose = self.pending_pose
        except Exception as exc:
            self._warn(f"Cartesian target rejected: {exc}")
        finally:
            self.pending_pose = None

    def _publish_joint_target(
        self, positions: list[float], velocities: list[float]
    ) -> None:
        message = JointTrajectory()
        message.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        message.points = [point]
        self.publisher.publish(message)

    def _stop(self, reason: str) -> None:
        self.state = "stopping"
        self.ik_pending = False
        self.pending_pose = None
        if not self.stop_client.service_is_ready():
            self.pending_after_stop = None
            self.state = "idle"
            self.require_deadman_release = True
            self.get_logger().error("servo/stop unavailable; driver watchdog will hold")
            return
        self.stop_client.call_async(Trigger.Request()).add_done_callback(
            lambda future: self._on_stopped(future, reason)
        )

    def _on_stopped(self, future, reason: str) -> None:
        pending = self.pending_after_stop
        self.pending_after_stop = None
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            self.get_logger().info(f"{self.control_mode} control stopped: {reason}")
        except Exception as exc:
            self.get_logger().error(f"servo stop failed; watchdog will hold: {exc}")
            pending = None
        self.state = "idle"
        self.require_deadman_release = True
        self.target_pose = self.target_joints = None
        if pending is not None:
            callback, args = pending
            callback(*args)

    def _request_operation(self, callback, reason: str, *args) -> None:
        if self.state == "active":
            self.pending_after_stop = (callback, args)
            self._stop(reason)
        elif self.state == "idle":
            callback(*args)
        else:
            self.get_logger().warn("control transition or operation busy; release LB and retry")

    def _request_gripper(self, client, label: str) -> None:
        self._request_operation(
            self._command_gripper, f"before gripper {label}", client, label
        )

    def _switch_to_joint_mode(self) -> None:
        self.control_mode = "joint"
        self.target_pose = self.pending_pose = None
        self.ik_pending = False
        self.require_deadman_release = True
        self.get_logger().info("joint position control selected; release LB, then hold LB to move")

    def _safe_home(self) -> None:
        if not self.safe_home_client.service_is_ready():
            self.require_deadman_release = True
            self.get_logger().warn("safe_home unavailable")
            return
        self.state = "safe_home"
        self.require_deadman_release = True
        try:
            self.safe_home_client.call_async(Trigger.Request()).add_done_callback(
                self._on_safe_home
            )
            self.get_logger().info("safe_home requested; waiting for collision-checked motion")
        except Exception as exc:
            self.state = "idle"
            self.get_logger().error(f"safe_home request failed: {exc}")

    def _on_safe_home(self, future) -> None:
        self.state = "idle"
        self.require_deadman_release = True
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            self.get_logger().info("safe_home complete; release LB before control")
        except Exception as exc:
            self.get_logger().error(f"safe_home failed: {exc}")

    def _command_gripper(self, client, label: str) -> None:
        if label == "close":
            self._start_grasp(client)
            return
        if not client.service_is_ready():
            self.get_logger().warn("gripper/open unavailable")
            return
        request = GripperCommand.Request()
        request.position = 0.0
        request.timeout = 3.0
        self.state = "gripper"
        self.require_deadman_release = True
        try:
            client.call_async(request).add_done_callback(self._on_gripper_opened)
        except Exception as exc:
            self.state = "idle"
            self.get_logger().error(f"gripper/open request failed: {exc}")

    def _on_gripper_opened(self, future) -> None:
        self.state = "idle"
        self.require_deadman_release = True
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            self.get_logger().info("gripper/open complete; release LB before Cartesian control")
        except Exception as exc:
            self.get_logger().error(f"gripper/open failed: {exc}")

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
                "object detected; holding torque active. Release LB before Cartesian control"
            )
        except Exception as exc:
            self._grasp_failed(exc)

    def _grasp_failed(self, exc) -> None:
        self.state = "idle"
        self.require_deadman_release = True
        self.get_logger().error(f"gripper/grasp failed: {exc}")

    def _warn(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warning >= 1.0:
            self.get_logger().warn(message)
            self.last_warning = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartesianJoystickController()
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
