from __future__ import annotations

import math
import threading
import time
from typing import Any

from control_msgs.action import FollowJointTrajectory, GripperCommand
import numpy as np
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from zekeep_msgs.action import GripperGrasp, MoveToPose

from .conversions import pose_to_xyz_rpy


_TRAJECTORY_GOAL_POSITION_TOLERANCE_RAD = 0.005
_TRAJECTORY_GOAL_VELOCITY_TOLERANCE_RAD_S = 0.03
_TRAJECTORY_GOAL_SETTLE_SAMPLES = 3
_TRAJECTORY_GOAL_SAMPLE_INTERVAL_S = 0.01
_TRAJECTORY_GOAL_DEFAULT_TIMEOUT_S = 3.0
_TRAJECTORY_COMMAND_PERIOD_S = 0.005


class ArmActions:
    """Register ROS actions for arm trajectories and gripper operations."""

    def __init__(self, node: Node, hardware: Any, namespace: str, *, internal=False) -> None:
        self._node = node
        self._hardware = hardware
        self._namespace = namespace
        self._internal = internal
        self._gripper_execution_lock = threading.Lock()
        self._move_to_pose_server = ActionServer(
            node,
            MoveToPose,
            f"/{namespace}/move_to_pose",
            execute_callback=node.web_task_gate.action(self.execute_move_to_pose, MoveToPose.Result, internal=internal),
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )
        self._follow_joint_trajectory_server = ActionServer(
            node,
            FollowJointTrajectory,
            f"/{namespace}/follow_joint_trajectory",
            execute_callback=node.web_task_gate.action(self.execute_follow_joint_trajectory, FollowJointTrajectory.Result, internal=internal),
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )
        self._gripper_command_server = ActionServer(
            node,
            GripperCommand,
            f"/{namespace}/gripper/command",
            execute_callback=node.web_task_gate.action(self.execute_gripper_command, GripperCommand.Result, internal=internal),
            goal_callback=self.gripper_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )
        self._gripper_grasp_server = ActionServer(
            node,
            GripperGrasp,
            f"/{namespace}/gripper/grasp",
            execute_callback=node.web_task_gate.action(self.execute_gripper_grasp, GripperGrasp.Result, internal=internal),
            goal_callback=self.gripper_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )

    def arm_goal_callback(self, _goal_request: Any) -> GoalResponse:
        return self._gate_goal(
            ("TRAJ_RUNNING", "SERVO_RUNNING", "GRAVITY_COMP", "SAFE_HOMING"), "arm motion"
        )

    def gripper_goal_callback(self, _goal_request: Any) -> GoalResponse:
        return self._gate_goal(("SERVO_RUNNING", "GRAVITY_COMP", "SAFE_HOMING"), "gripper")

    def _gate_goal(
        self, blocked: tuple[str, ...], label: str
    ) -> GoalResponse:
        if not self._node.web_task_gate.allowed(self._internal):
            return GoalResponse.REJECT
        state = self._hardware.state_machine
        if getattr(self._hardware, "_shutdown_pending", False) or state == "SHUTDOWN_FAILED" or state in blocked:
            self._node.get_logger().warn(f"rejecting {label} goal in state {state}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle: ServerGoalHandle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _fail_move_to_pose(
        self,
        goal_handle: ServerGoalHandle,
        result: MoveToPose.Result,
        message: str,
        *,
        canceled: bool = False,
        generation=None,
    ) -> MoveToPose.Result:
        if generation is not None and self._hardware.trajectory_generation == generation and self._hardware.state_machine == "TRAJ_RUNNING":
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()
        if canceled:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        result.success = False
        result.message = message
        result.final_pose = self._hardware.current_pose()
        return result

    def execute_move_to_pose(
        self, goal_handle: ServerGoalHandle
    ) -> MoveToPose.Result:
        goal = goal_handle.request
        result = MoveToPose.Result()
        generation = self._hardware.trajectory_generation

        try:
            x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(goal.target_pose)
            ok = self._hardware.move_to_pose_traj(
                x, y, z, roll, pitch, yaw, float(goal.duration)
            )
        except Exception as exc:
            if self._hardware.trajectory_generation == generation:
                self._hardware.hold_current_position()
            return self._fail_move_to_pose(goal_handle, result, str(exc), generation=generation)

        if not ok:
            return self._fail_move_to_pose(
                goal_handle, result, "trajectory planning failed", generation=generation
            )
        self._node.publish_arm_status()

        duration = self._hardware.planned_motion_duration
        deadline = time.monotonic() + duration + 2.0 + _TRAJECTORY_GOAL_DEFAULT_TIMEOUT_S
        settled_samples = 0
        settle_deadline = None
        while True:
            if self._hardware.state_machine == "SAFE_HOMING" or self._hardware.trajectory_generation != generation:
                break
            if goal_handle.is_cancel_requested:
                self._hardware.stop_motion()
                self._hardware.hold_current_position()
                return self._fail_move_to_pose(
                    goal_handle, result, "move_to_pose canceled", canceled=True, generation=generation
                )
            if time.monotonic() > deadline:
                self._hardware.stop_motion()
                self._hardware.hold_current_position()
                return self._fail_move_to_pose(
                    goal_handle, result, "move_to_pose timeout", generation=generation
                )
            if not self._hardware.motion_active():
                if settle_deadline is None:
                    settle_deadline = time.monotonic() + _TRAJECTORY_GOAL_DEFAULT_TIMEOUT_S
                positions = self._hardware.get_joint_positions()
                velocities = self._hardware.get_joint_velocities()
                error = np.max(np.abs(positions - self._hardware.planned_joint_target))
                if (np.all(np.isfinite(positions)) and np.all(np.isfinite(velocities))
                        and error <= _TRAJECTORY_GOAL_POSITION_TOLERANCE_RAD
                        and np.max(np.abs(velocities)) <= _TRAJECTORY_GOAL_VELOCITY_TOLERANCE_RAD_S):
                    settled_samples += 1
                    if settled_samples >= _TRAJECTORY_GOAL_SETTLE_SAMPLES:
                        break
                else:
                    settled_samples = 0
                if time.monotonic() >= settle_deadline:
                    self._hardware.stop_motion()
                    self._hardware.hold_current_position()
                    return self._fail_move_to_pose(
                        goal_handle, result, f"move_to_pose did not settle: error={error:.6f}rad",
                        generation=generation,
                    )
            time.sleep(0.02)

        if self._hardware.state_machine == "SAFE_HOMING" or self._hardware.trajectory_generation != generation:
            return self._fail_move_to_pose(
                goal_handle, result, "move_to_pose interrupted", generation=generation
            )

        positions = self._hardware.get_joint_positions()
        velocities = self._hardware.get_joint_velocities()
        result.success = True
        result.message = (
            "move_to_traj accepted "
            f"positions={[float(v) for v in positions]} "
            f"velocities={[float(v) for v in velocities]}"
        )
        result.final_pose = self._hardware.current_pose()
        self._hardware.set_state_machine("IDLE")
        self._node.publish_arm_status()
        goal_handle.succeed()
        return result

    def execute_follow_joint_trajectory(
        self, goal_handle: ServerGoalHandle
    ) -> FollowJointTrajectory.Result:
        goal = goal_handle.request
        result = FollowJointTrajectory.Result()
        trajectory = goal.trajectory
        joint_names = list(trajectory.joint_names)

        if not joint_names or not trajectory.points:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory must include joint_names and points"
            return result

        if joint_names != self._hardware.joint_names:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = (
                f"trajectory joint_names must be {self._hardware.joint_names}"
            )
            return result

        targets_pos, targets_vel, point_times = [], [], []
        try:
            for point in trajectory.points:
                stamp = point.time_from_start
                if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
                    raise ValueError("time_from_start must be nonnegative and normalized")
                timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
                if point_times and timestamp <= point_times[-1]:
                    raise ValueError("time_from_start must be strictly increasing")
                point_times.append(timestamp)
                for field in ("positions", "velocities", "accelerations", "effort"):
                    values = np.asarray(getattr(point, field, []), dtype=np.float64)
                    if (field == "positions" or values.size) and (
                        values.shape != (len(joint_names),) or not np.all(np.isfinite(values))
                    ):
                        raise ValueError(f"invalid point {field}: expected finite joint vector")
                pos = np.asarray(point.positions, dtype=np.float64)
                vel = np.asarray(point.velocities, dtype=np.float64) if point.velocities else np.zeros_like(pos)
                self._hardware.validate_joint_positions(pos)
                targets_pos.append(pos)
                targets_vel.append(vel)
            current = self._hardware.get_joint_positions().copy()
            if not np.all(np.isfinite(current)):
                raise ValueError("non-finite starting joint feedback")
            if point_times[0] == 0.0:
                if np.max(np.abs(targets_pos[0] - current)) > _TRAJECTORY_GOAL_POSITION_TOLERANCE_RAD:
                    raise ValueError("zero-time first point is discontinuous from feedback")
            else:
                targets_pos.insert(0, current)
                targets_vel.insert(0, np.zeros_like(current))
                point_times.insert(0, 0.0)
            limits = self._hardware.joint_velocity_limits
            for index in range(1, len(targets_pos)):
                duration = point_times[index] - point_times[index - 1]
                p0, p1 = targets_pos[index - 1:index + 1]
                v0, v1 = targets_vel[index - 1:index + 1]
                # Cubic Hermite velocity is quadratic. Check endpoints and its
                # analytic extremum for every joint before any motor output.
                a = 6 * (p0 - p1) / duration + 3 * (v0 + v1)
                b = 6 * (p1 - p0) / duration - 4 * v0 - 2 * v1
                with np.errstate(divide="ignore", invalid="ignore"):
                    extremum = np.clip(np.divide(-b, 2 * a, out=np.zeros_like(a), where=a != 0), 0, 1)
                peak = np.maximum(np.maximum(np.abs(v0), np.abs(v1)), np.abs(a * extremum**2 + b * extremum + v0))
                if not np.all(np.isfinite(peak)) or np.any(peak > limits + 1e-9):
                    raise ValueError("trajectory exceeds configured joint velocity limits")
        except (ValueError, TypeError, OverflowError) as exc:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result

        generation = None
        try:
            generation = self._hardware.begin_trajectory_stream()
            self._node.publish_arm_status()
            start = time.monotonic()
            for index in range(1, len(targets_pos)):
                p0 = targets_pos[index - 1]
                p1 = targets_pos[index]
                v0 = targets_vel[index - 1]
                v1 = targets_vel[index]

                t0 = point_times[index - 1]
                t1 = point_times[index]
                T = t1 - t0

                while True:
                    if self._hardware.state_machine == "SAFE_HOMING" or self._hardware.trajectory_generation != generation:
                        goal_handle.abort()
                        result.error_code = (
                            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        )
                        result.error_string = (
                            "follow_joint_trajectory preempted by safe_home"
                        )
                        return result

                    now = time.monotonic() - start

                    # Preserve both position and velocity continuity between points.
                    t_seg = max(0.0, min(T, now - t0))
                    s = t_seg / T
                    s2 = s * s
                    s3 = s2 * s

                    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
                    h10 = s3 - 2.0 * s2 + s
                    h01 = -2.0 * s3 + 3.0 * s2
                    h11 = s3 - s2

                    target_pos = h00 * p0 + h10 * T * v0 + h01 * p1 + h11 * T * v1

                    dh00 = 6.0 * s2 - 6.0 * s
                    dh10 = 3.0 * s2 - 4.0 * s + 1.0
                    dh01 = -6.0 * s2 + 6.0 * s
                    dh11 = 3.0 * s2 - 2.0 * s

                    target_vel = (dh00 * p0 + dh10 * T * v0 + dh01 * p1 + dh11 * T * v1) / T

                    self._hardware.set_joint_position_target(
                        target_pos,
                        velocities=target_vel,
                        generation=generation,
                    )

                    positions = self._hardware.get_joint_positions()
                    velocities = self._hardware.get_joint_velocities()

                    feedback = FollowJointTrajectory.Feedback()
                    feedback.header.stamp = self._node.get_clock().now().to_msg()
                    feedback.joint_names = self._hardware.joint_names
                    feedback.desired.positions = [float(v) for v in target_pos]
                    feedback.desired.velocities = [float(v) for v in target_vel]
                    feedback.actual.positions = [float(v) for v in positions]
                    feedback.actual.velocities = [float(v) for v in velocities]
                    feedback.error.positions = [float(v) for v in target_pos - positions]
                    feedback.error.velocities = [float(v) for v in target_vel - velocities]
                    goal_handle.publish_feedback(feedback)

                    if goal_handle.is_cancel_requested:
                        self._hardware.hold_current_position()
                        goal_handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "follow_joint_trajectory canceled"
                        return result

                    if now >= t1:
                        break

                    time.sleep(_TRAJECTORY_COMMAND_PERIOD_S)

            goal_time_tolerance = getattr(goal, "goal_time_tolerance", None)
            settle_timeout_s = (
                float(getattr(goal_time_tolerance, "sec", 0))
                + float(getattr(goal_time_tolerance, "nanosec", 0)) * 1e-9
            )
            if settle_timeout_s <= 0.0:
                settle_timeout_s = _TRAJECTORY_GOAL_DEFAULT_TIMEOUT_S
            settle_deadline = time.monotonic() + settle_timeout_s
            settled_samples = 0
            final_target = np.asarray(targets_pos[-1], dtype=np.float64)
            positions = self._hardware.get_joint_positions()
            velocities = self._hardware.get_joint_velocities()
            while time.monotonic() < settle_deadline:
                if self._hardware.state_machine == "SAFE_HOMING" or self._hardware.trajectory_generation != generation:
                    goal_handle.abort()
                    result.error_code = (
                        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    )
                    result.error_string = (
                        "final joint settle preempted by safe_home"
                    )
                    return result
                if goal_handle.is_cancel_requested:
                    self._hardware.hold_current_position()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "follow_joint_trajectory canceled"
                    return result

                positions = self._hardware.get_joint_positions()
                velocities = self._hardware.get_joint_velocities()
                position_error = float(
                    np.max(np.abs(final_target - positions))
                )
                peak_velocity = float(np.max(np.abs(velocities)))
                if (
                    position_error <= _TRAJECTORY_GOAL_POSITION_TOLERANCE_RAD
                    and peak_velocity
                    <= _TRAJECTORY_GOAL_VELOCITY_TOLERANCE_RAD_S
                ):
                    settled_samples += 1
                    if settled_samples >= _TRAJECTORY_GOAL_SETTLE_SAMPLES:
                        break
                else:
                    settled_samples = 0
                time.sleep(_TRAJECTORY_GOAL_SAMPLE_INTERVAL_S)
            else:
                position_error = float(
                    np.max(np.abs(final_target - positions))
                )
                peak_velocity = float(np.max(np.abs(velocities)))
                self._hardware.hold_current_position()
                goal_handle.abort()
                result.error_code = (
                    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                )
                result.error_string = (
                    "final joint target did not settle: "
                    f"max_position_error={position_error:.6f}rad "
                    f"peak_velocity={peak_velocity:.6f}rad/s"
                )
                return result

        except Exception as exc:
            if generation is not None and self._hardware.trajectory_generation == generation:
                self._hardware.hold_current_position()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = f"execution failed: {exc}"
            return result
        finally:
            if generation is not None and self._hardware.trajectory_generation == generation and self._hardware.state_machine == "TRAJ_RUNNING":
                self._hardware.set_state_machine("IDLE")
                self._node.publish_arm_status()

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        positions = self._hardware.get_joint_positions()
        velocities = self._hardware.get_joint_velocities()
        result.error_string = (
            "joint target accepted "
            f"positions={[float(v) for v in positions]} "
            f"velocities={[float(v) for v in velocities]}"
        )
        return result

    def _begin_gripper_execution(self) -> bool:
        lock = getattr(self, "_gripper_execution_lock", None)
        if lock is None:
            lock = threading.Lock()
            self._gripper_execution_lock = lock
        return bool(lock.acquire(blocking=False))

    def _end_gripper_execution(self) -> None:
        self._gripper_execution_lock.release()

    def execute_gripper_command(
        self, goal_handle: ServerGoalHandle
    ) -> GripperCommand.Result:
        if not self._begin_gripper_execution():
            result = GripperCommand.Result()
            goal_handle.abort()
            result.reached_goal = False
            return result
        try:
            return self._execute_gripper_command(goal_handle)
        finally:
            self._end_gripper_execution()

    def _execute_gripper_command(
        self, goal_handle: ServerGoalHandle
    ) -> GripperCommand.Result:
        goal = goal_handle.request.command
        result = GripperCommand.Result()
        feedback = GripperCommand.Feedback()

        try:
            self._hardware.set_gripper_target(goal.position)
        except Exception:
            goal_handle.abort()
            result.position = 0.0
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
            return result

        start = time.monotonic()
        last_pos = self._hardware.get_gripper_state()[0]
        stalled = False
        while time.monotonic() - start < 5.0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                pos, _, effort, _ = self._hardware.get_gripper_state()
                result.position = pos
                result.effort = effort
                result.stalled = stalled
                result.reached_goal = False
                return result

            pos, _, effort, _ = self._hardware.get_gripper_state()
            reached = self._hardware.gripper_reached_target()
            stalled = abs(pos - last_pos) < 1e-4 and abs(effort) >= float(goal.max_effort)
            feedback.position = pos
            feedback.effort = effort
            feedback.stalled = stalled
            feedback.reached_goal = reached
            goal_handle.publish_feedback(feedback)
            if reached:
                break
            last_pos = pos
            time.sleep(0.05)

        pos, _, effort, _ = self._hardware.get_gripper_state()
        result.position = pos
        result.effort = effort
        result.stalled = stalled
        result.reached_goal = self._hardware.gripper_reached_target()
        goal_handle.succeed()
        return result

    def execute_gripper_grasp(
        self, goal_handle: ServerGoalHandle
    ) -> GripperGrasp.Result:
        if not self._begin_gripper_execution():
            result = GripperGrasp.Result()
            goal_handle.abort()
            result.success = False
            result.object_detected = False
            result.message = "another gripper action is active"
            return result
        try:
            return self._execute_gripper_grasp(goal_handle)
        finally:
            self._end_gripper_execution()

    def _execute_gripper_grasp(
        self, goal_handle: ServerGoalHandle
    ) -> GripperGrasp.Result:
        goal = goal_handle.request
        result = GripperGrasp.Result()
        feedback = GripperGrasp.Feedback()
        requested_timeout = float(goal.timeout)
        if not math.isfinite(requested_timeout) or requested_timeout < 0.0:
            goal_handle.abort()
            result.success = False
            result.object_detected = False
            result.message = "gripper grasp timeout must be finite and nonnegative"
            return result
        timeout = (
            requested_timeout
            if requested_timeout > 0.0
            else self._hardware.gripper_grasp_timeout
        )
        started = False
        try:
            self._hardware.start_gripper_grasp(
                closing_torque=float(goal.closing_torque),
                hold_torque=float(goal.hold_torque),
            )
            started = True
        except Exception as exc:
            goal_handle.abort()
            result.success = False
            result.object_detected = False
            result.message = str(exc)
            return result

        try:
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                done, detected, state, position, velocity, torque = (
                    self._hardware.get_gripper_grasp_status()
                )
                feedback.position = position
                feedback.velocity = velocity
                feedback.torque = torque
                feedback.state = state
                goal_handle.publish_feedback(feedback)
                if done:
                    result.success = bool(detected)
                    result.object_detected = bool(detected)
                    result.final_position = position
                    result.final_torque = torque
                    result.message = (
                        "object detected and held" if detected else "empty grasp"
                    )
                    goal_handle.succeed()
                    return result
                if goal_handle.is_cancel_requested:
                    self._hardware.stop_gripper_grasp()
                    started = False
                    goal_handle.canceled()
                    result.success = False
                    result.object_detected = False
                    result.final_position = position
                    result.final_torque = torque
                    result.message = (
                        "gripper grasp canceled; holding current position"
                    )
                    return result
                time.sleep(0.02)

            # Sample once more at the deadline before clearing the grasp state.
            # The control loop may have classified contact or the hard stop
            # between the last polling iteration and the deadline.
            done, detected, state, position, velocity, torque = (
                self._hardware.get_gripper_grasp_status()
            )
            feedback.position = position
            feedback.velocity = velocity
            feedback.torque = torque
            feedback.state = state
            goal_handle.publish_feedback(feedback)
            if done:
                result.success = bool(detected)
                result.object_detected = bool(detected)
                result.final_position = position
                result.final_torque = torque
                result.message = (
                    "object detected and held" if detected else "empty grasp"
                )
                goal_handle.succeed()
                return result

            self._hardware.stop_gripper_grasp()
            started = False
            goal_handle.abort()
            result.success = False
            result.object_detected = False
            result.final_position = position
            result.final_torque = torque
            result.message = (
                "gripper grasp timeout; holding current position "
                f"state={state} position={position:.4f}rad "
                f"velocity={velocity:.4f}rad/s torque={torque:.4f}"
            )
            return result
        except Exception as exc:
            cleanup_error = None
            if started:
                try:
                    self._hardware.stop_gripper_grasp()
                except Exception as stop_exc:
                    cleanup_error = stop_exc
            goal_handle.abort()
            result.success = False
            result.object_detected = False
            if cleanup_error is None:
                result.message = f"gripper grasp failed safely: {exc}"
            else:
                result.message = (
                    f"gripper grasp failed: {exc}; "
                    f"cleanup also failed: {cleanup_error}"
                )
            return result
