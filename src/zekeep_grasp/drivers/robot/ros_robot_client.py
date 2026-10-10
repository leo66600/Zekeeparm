"""ROS-only robot client used by visual grasping.

This client never opens the motor serial port.  The zekeepcontroller process
remains the sole hardware owner.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
import math
import threading
import time
from typing import Any

import numpy as np


_ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))


def _joint_state_qos():
    from rclpy.qos import qos_profile_sensor_data

    return qos_profile_sensor_data


@dataclass(frozen=True)
class JointSample:
    positions: np.ndarray
    velocities: np.ndarray
    received_monotonic: float
    sequence: int


@dataclass(frozen=True)
class MoveItGraspPlan:
    outward_trajectory: Any
    return_trajectory: Any
    outward_final_positions: np.ndarray
    return_final_positions: np.ndarray


class JointStateCache:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._sample: JointSample | None = None
        self._sequence = 0

    def update(
        self,
        names,
        positions,
        velocities,
        *,
        received_monotonic: float,
    ) -> None:
        position_by_name = dict(zip(names, positions))
        velocity_by_name = dict(zip(names, velocities))
        missing = [name for name in _ARM_JOINT_NAMES if name not in position_by_name]
        missing_velocities = [
            name for name in _ARM_JOINT_NAMES if name not in velocity_by_name
        ]
        if missing or missing_velocities:
            return
        q = np.array([position_by_name[name] for name in _ARM_JOINT_NAMES], dtype=np.float64)
        qd = np.array(
            [velocity_by_name.get(name, 0.0) for name in _ARM_JOINT_NAMES],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(qd)):
            return
        with self._condition:
            self._sequence += 1
            self._sample = JointSample(
                q,
                qd,
                float(received_monotonic),
                self._sequence,
            )
            self._condition.notify_all()

    def wait(self, timeout_s: float) -> JointSample:
        deadline = time.monotonic() + float(timeout_s)
        with self._condition:
            while self._sample is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("joint_states not received")
                self._condition.wait(timeout=remaining)
            return self._sample

    def latest(self, *, max_age_s: float, now: float | None = None) -> JointSample:
        now = time.monotonic() if now is None else float(now)
        with self._condition:
            sample = self._sample
        if sample is None:
            raise RuntimeError("joint_states not received")
        age = now - sample.received_monotonic
        if age < 0.0 or age > float(max_age_s):
            raise RuntimeError(
                f"joint_states stale: age={age:.3f}s limit={float(max_age_s):.3f}s"
            )
        return sample


def _rotation_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError("rotation must have shape (3, 3)")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            qw = (matrix[2, 1] - matrix[1, 2]) / scale
            qx = 0.25 * scale
            qy = (matrix[0, 1] + matrix[1, 0]) / scale
            qz = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            qw = (matrix[0, 2] - matrix[2, 0]) / scale
            qx = (matrix[0, 1] + matrix[1, 0]) / scale
            qy = 0.25 * scale
            qz = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            qw = (matrix[1, 0] - matrix[0, 1]) / scale
            qx = (matrix[0, 2] + matrix[2, 0]) / scale
            qy = (matrix[1, 2] + matrix[2, 1]) / scale
            qz = 0.25 * scale
    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


class RosRobotClient:
    def __init__(
        self,
        *,
        namespace: str,
        joint_state_topic: str,
        timeout_s: float,
        joint_state_max_age_s: float,
        T_link6_to_end: np.ndarray,
        gripper_open_width_m: float = 0.06517241379310346,
        safe_home_timeout_s: float = 35.0,
    ) -> None:
        import rclpy
        from action_msgs.msg import GoalStatus
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from geometry_msgs.msg import Pose
        from moveit_msgs.action import ExecuteTrajectory
        from moveit_msgs.msg import (
            AttachedCollisionObject,
            CollisionObject,
            Constraints,
            JointConstraint,
            MoveItErrorCodes,
            PlanningScene,
            RobotState,
        )
        from moveit_msgs.srv import (
            ApplyPlanningScene,
            GetCartesianPath,
            GetPlanningScene,
            GetMotionPlan,
            GetPositionIK,
        )
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from zekeep_msgs.action import GripperGrasp, MoveToPose
        from zekeep_msgs.srv import GripperCommand
        from sensor_msgs.msg import JointState
        from shape_msgs.msg import SolidPrimitive
        from std_srvs.srv import Trigger
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        gripper_open_width = float(gripper_open_width_m)
        if (
            not math.isfinite(gripper_open_width)
            or gripper_open_width < 0.0
            or gripper_open_width > 0.06517241379310346
        ):
            raise ValueError("gripper open width must be within [0, 0.06517241379310346] m")
        self._gripper_open_joint_position_m = gripper_open_width / 2.0

        self._rclpy = rclpy
        self._GoalStatus = GoalStatus
        self._Duration = Duration
        self._Pose = Pose
        self._ExecuteTrajectory = ExecuteTrajectory
        self._FollowJointTrajectory = FollowJointTrajectory
        self._JointTrajectory = JointTrajectory
        self._JointTrajectoryPoint = JointTrajectoryPoint
        self._ApplyPlanningScene = ApplyPlanningScene
        self._CollisionObject = CollisionObject
        self._AttachedCollisionObject = AttachedCollisionObject
        self._Constraints = Constraints
        self._GetCartesianPath = GetCartesianPath
        self._GetPlanningScene = GetPlanningScene
        self._GetMotionPlan = GetMotionPlan
        self._GetPositionIK = GetPositionIK
        self._JointConstraint = JointConstraint
        self._MoveItErrorCodes = MoveItErrorCodes
        self._PlanningScene = PlanningScene
        self._RobotState = RobotState
        self._SolidPrimitive = SolidPrimitive
        self._JointState = JointState
        self._MoveToPose = MoveToPose
        self._GripperGrasp = GripperGrasp
        self._GripperCommand = GripperCommand
        self._Trigger = Trigger
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self._node = Node(f"zekeep_grasp_client_{int(time.time() * 1000) % 100000}")
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._cache = JointStateCache()
        self._timeout_s = float(timeout_s)
        self._safe_home_timeout_s = float(safe_home_timeout_s)
        if not math.isfinite(self._safe_home_timeout_s) or self._safe_home_timeout_s <= 0.0:
            raise ValueError("safe-home timeout must be finite and positive")
        self._joint_state_max_age_s = float(joint_state_max_age_s)
        self._T_end_to_link6 = np.linalg.inv(np.asarray(T_link6_to_end, dtype=np.float64))
        self._active_goal = None
        self._cancel_requested_goal = None
        self._action_starting = False
        self._active_goal_lock = threading.Lock()
        ns = str(namespace).strip("/")
        self._node.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_cb,
            _joint_state_qos(),
        )
        self._move_client = ActionClient(self._node, MoveToPose, f"/{ns}/move_to_pose")
        self._execute_trajectory_client = ActionClient(
            self._node,
            ExecuteTrajectory,
            "/execute_trajectory",
        )
        self._follow_joint_trajectory_client = ActionClient(
            self._node,
            FollowJointTrajectory,
            f"/{ns}/follow_joint_trajectory",
        )
        self._cartesian_client = self._node.create_client(
            GetCartesianPath,
            "/compute_cartesian_path",
        )
        self._motion_plan_client = self._node.create_client(
            GetMotionPlan,
            "/plan_kinematic_path",
        )
        self._compute_ik_client = self._node.create_client(
            GetPositionIK,
            "/compute_ik",
        )
        self._planning_scene_client = self._node.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )
        self._get_planning_scene_client = self._node.create_client(GetPlanningScene, '/get_planning_scene')
        self._grasp_client = ActionClient(
            self._node,
            GripperGrasp,
            f"/{ns}/gripper/grasp",
        )
        self._open_client = self._node.create_client(
            GripperCommand,
            f"/{ns}/gripper/open",
        )
        self._gc_start_client = self._node.create_client(
            Trigger,
            f"/{ns}/gravity_compensation/start",
        )
        self._gc_stop_client = self._node.create_client(
            Trigger,
            f"/{ns}/gravity_compensation/stop",
        )
        self._safe_home_client = self._node.create_client(
            Trigger,
            f"/{ns}/safe_home",
        )
        self._disable_client = self._node.create_client(
            Trigger,
            f"/{ns}/disable",
        )
        self._stop_client = self._node.create_client(Trigger, f"/{ns}/stop")

    @property
    def joint_count(self) -> int:
        return len(_ARM_JOINT_NAMES)

    def start(
        self,
        *,
        require_gripper: bool = True,
        require_moveit: bool = False,
        require_direct_joint: bool = False,
    ) -> None:
        self._spin_thread.start()
        self._cache.wait(self._timeout_s)
        if not self._move_client.wait_for_server(timeout_sec=self._timeout_s):
            raise TimeoutError("move_to_pose action not available")
        if require_gripper:
            if not self._grasp_client.wait_for_server(timeout_sec=self._timeout_s):
                raise TimeoutError("gripper/grasp action not available")
            if not self._open_client.wait_for_service(timeout_sec=self._timeout_s):
                raise TimeoutError("gripper/open service not available")
        if require_moveit:
            if not self._cartesian_client.wait_for_service(
                timeout_sec=self._timeout_s
            ):
                raise TimeoutError("MoveIt compute_cartesian_path unavailable")
            if not self._motion_plan_client.wait_for_service(
                timeout_sec=self._timeout_s
            ):
                raise TimeoutError("MoveIt plan_kinematic_path unavailable")
            if not self._compute_ik_client.wait_for_service(
                timeout_sec=self._timeout_s
            ):
                raise TimeoutError("MoveIt compute_ik unavailable")
            if not self._execute_trajectory_client.wait_for_server(
                timeout_sec=self._timeout_s
            ):
                raise TimeoutError("MoveIt execute_trajectory unavailable")
            if not self._planning_scene_client.wait_for_service(
                timeout_sec=self._timeout_s
            ):
                raise TimeoutError("MoveIt apply_planning_scene unavailable")
        if require_direct_joint and not self._follow_joint_trajectory_client.wait_for_server(
            timeout_sec=self._timeout_s
        ):
            raise TimeoutError("follow_joint_trajectory action unavailable")

    def _joint_state_cb(self, message) -> None:
        self._cache.update(
            message.name,
            message.position,
            message.velocity,
            received_monotonic=time.monotonic(),
        )

    def get_state(self, request_feedback: bool = False):
        del request_feedback
        sample = self._cache.latest(max_age_s=self._joint_state_max_age_s)
        return sample.positions.copy(), sample.velocities.copy(), np.zeros(6)

    def latest_sample(self) -> JointSample:
        return self._cache.latest(max_age_s=self._joint_state_max_age_s)

    def is_stationary(self, max_velocity_rad_s: float) -> bool:
        sample = self.latest_sample()
        return bool(
            np.max(np.abs(sample.velocities)) <= float(max_velocity_rad_s)
        )

    def wait_until_stationary(
        self,
        *,
        velocity_tolerance_rad_s: float,
        required_samples: int,
        sample_interval_s: float,
        timeout_s: float,
        cancel_event: threading.Event | None = None,
    ) -> None:
        if required_samples <= 0:
            raise ValueError("required stationary samples must be positive")
        deadline = time.monotonic() + float(timeout_s)
        consecutive = 0
        last_sequence = -1
        peak_velocity = float("inf")
        peak_joint_index = 0
        while time.monotonic() < deadline:
            if cancel_event is not None and cancel_event.is_set():
                raise RuntimeError("stationary wait cancelled")
            sample = self.latest_sample()
            if sample.sequence == last_sequence:
                time.sleep(max(float(sample_interval_s), 0.001))
                continue
            last_sequence = sample.sequence
            absolute_velocities = np.abs(sample.velocities)
            peak_joint_index = int(np.argmax(absolute_velocities))
            peak_velocity = float(absolute_velocities[peak_joint_index])
            if peak_velocity <= float(velocity_tolerance_rad_s):
                consecutive += 1
                if consecutive >= int(required_samples):
                    return
            else:
                consecutive = 0
            time.sleep(max(float(sample_interval_s), 0.0))
        raise TimeoutError(
            "robot joint feedback did not become stationary: "
            f"joint={_ARM_JOINT_NAMES[peak_joint_index]} "
            f"peak_velocity={peak_velocity:.6f}rad/s "
            f"limit={float(velocity_tolerance_rad_s):.6f}rad/s"
        )

    def wait_for_settle(
        self,
        target_positions: np.ndarray,
        *,
        position_tolerance_rad: float,
        velocity_tolerance_rad_s: float,
        required_samples: int,
        sample_interval_s: float,
        timeout_s: float,
        cancel_event: threading.Event | None = None,
    ) -> None:
        target = np.asarray(target_positions, dtype=np.float64).reshape(-1)[:6]
        if target.size != 6 or not np.all(np.isfinite(target)):
            raise ValueError("settle target must contain six finite joint positions")
        if required_samples <= 0:
            raise ValueError("required settle samples must be positive")
        deadline = time.monotonic() + float(timeout_s)
        consecutive = 0
        last_sequence = -1
        while time.monotonic() < deadline:
            if cancel_event is not None and cancel_event.is_set():
                raise RuntimeError("settle wait cancelled")
            sample = self.latest_sample()
            if sample.sequence == last_sequence:
                time.sleep(max(float(sample_interval_s), 0.001))
                continue
            last_sequence = sample.sequence
            position_error = float(np.max(np.abs(sample.positions - target)))
            peak_velocity = float(np.max(np.abs(sample.velocities)))
            if (
                position_error <= float(position_tolerance_rad)
                and peak_velocity <= float(velocity_tolerance_rad_s)
            ):
                consecutive += 1
                if consecutive >= int(required_samples):
                    return
            else:
                consecutive = 0
            time.sleep(max(float(sample_interval_s), 0.0))
        raise TimeoutError("robot joint feedback did not settle at motion target")

    def assert_capture_sync(self, capture_monotonic: float, tolerance_s: float) -> None:
        sample = self.latest_sample()
        delta = abs(float(capture_monotonic) - sample.received_monotonic)
        if delta > float(tolerance_s):
            raise RuntimeError(
                f"RGB-D/joint-state sync error {delta:.3f}s exceeds "
                f"{float(tolerance_s):.3f}s"
            )

    def move_end_pose(self, T_base_to_end: np.ndarray, duration: float) -> bool:
        T_base_to_link6 = (
            np.asarray(T_base_to_end, dtype=np.float64) @ self._T_end_to_link6
        )
        goal = self._MoveToPose.Goal()
        goal.target_pose = self._pose_message(T_base_to_link6)
        goal.duration = float(duration)
        # duration is requested timing; POS_VEL may stretch it. Keep a bounded
        # transport wait while the server enforces planned timing and settling.
        result = self._run_action(self._move_client, goal, max(float(duration) + 10.0, 120.0))
        return bool(result.success)

    def solve_end_pose_ik(
        self,
        T_base_to_end: np.ndarray,
        seed_positions: np.ndarray,
        *,
        timeout_s: float,
        attempts: int,
    ) -> tuple[np.ndarray | None, int]:
        transform = np.asarray(T_base_to_end, dtype=np.float64)
        seed = np.asarray(seed_positions, dtype=np.float64).reshape(-1)
        timeout = float(timeout_s)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError("MoveIt IK target must be a finite 4x4 transform")
        if seed.size != len(_ARM_JOINT_NAMES) or not np.all(np.isfinite(seed)):
            raise ValueError("MoveIt IK seed must contain six finite joints")
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("MoveIt IK timeout must be positive")
        if int(attempts) <= 0:
            raise ValueError("MoveIt IK attempts must be positive")

        request = self._GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = "arm"
        ik_request.robot_state = self._robot_state(seed)
        ik_request.avoid_collisions = True
        ik_request.ik_link_name = "link6"
        ik_request.pose_stamped.header.frame_id = "base_link"
        ik_request.pose_stamped.pose = self._pose_message(
            transform @ self._T_end_to_link6
        )
        duration = self._Duration()
        duration.sec = int(timeout)
        duration.nanosec = int(round((timeout - duration.sec) * 1e9))
        if duration.nanosec >= 1_000_000_000:
            duration.sec += 1
            duration.nanosec -= 1_000_000_000
        ik_request.timeout = duration
        if hasattr(ik_request, "attempts"):
            ik_request.attempts = int(attempts)

        response = self._wait_future(
            self._compute_ik_client.call_async(request),
            max(self._timeout_s, timeout + 1.0),
        )
        if response is None:
            raise RuntimeError("MoveIt compute_ik returned no response")
        error_code = int(response.error_code.val)
        if error_code != int(self._MoveItErrorCodes.SUCCESS):
            return None, error_code

        joint_state = response.solution.joint_state
        position_by_name = dict(zip(joint_state.name, joint_state.position))
        missing = [
            name for name in _ARM_JOINT_NAMES if name not in position_by_name
        ]
        if missing:
            raise RuntimeError(
                f"MoveIt IK solution missing joints: {','.join(missing)}"
            )
        solution = np.array(
            [position_by_name[name] for name in _ARM_JOINT_NAMES],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(solution)):
            raise RuntimeError("MoveIt IK returned non-finite joints")
        return solution, error_code

    def plan_joint_goal(
        self,
        start_positions: np.ndarray,
        target_positions: np.ndarray,
        *,
        pipeline_id: str,
        planner_id: str,
        num_planning_attempts: int,
        allowed_planning_time_s: float,
        max_velocity_scaling_factor: float,
        max_acceleration_scaling_factor: float,
        joint_tolerance_rad: float,
        label: str,
        gripper_open: bool = False,
    ):
        start = np.asarray(start_positions, dtype=np.float64).reshape(-1)
        target = np.asarray(target_positions, dtype=np.float64).reshape(-1)
        if (
            start.size != len(_ARM_JOINT_NAMES)
            or target.size != len(_ARM_JOINT_NAMES)
            or not np.all(np.isfinite(start))
            or not np.all(np.isfinite(target))
        ):
            raise ValueError(
                "MoveIt joint plan requires six finite start and target joints"
            )
        attempts = int(num_planning_attempts)
        planning_time = float(allowed_planning_time_s)
        velocity_scale = float(max_velocity_scaling_factor)
        acceleration_scale = float(max_acceleration_scaling_factor)
        tolerance = float(joint_tolerance_rad)
        if attempts <= 0:
            raise ValueError("MoveIt planning attempts must be positive")
        if not math.isfinite(planning_time) or planning_time <= 0.0:
            raise ValueError("MoveIt planning time must be positive")
        if (
            not math.isfinite(velocity_scale)
            or not 0.0 < velocity_scale <= 1.0
            or not math.isfinite(acceleration_scale)
            or not 0.0 < acceleration_scale <= 1.0
        ):
            raise ValueError("MoveIt scaling factors must be in (0, 1]")
        if not math.isfinite(tolerance) or tolerance <= 0.0:
            raise ValueError("MoveIt joint goal tolerance must be positive")

        constraints = self._Constraints()
        constraints.name = f"{label}_joint_goal"
        constraints.joint_constraints = []
        for joint_name, position in zip(_ARM_JOINT_NAMES, target):
            constraint = self._JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = float(position)
            constraint.tolerance_above = tolerance
            constraint.tolerance_below = tolerance
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)

        request = self._GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.start_state = self._robot_state(
            start, gripper_joint_position_m=self._gripper_open_joint_position_m if gripper_open else None
        )
        motion_request.goal_constraints = [constraints]
        motion_request.pipeline_id = str(pipeline_id)
        motion_request.planner_id = str(planner_id)
        motion_request.group_name = "arm"
        motion_request.num_planning_attempts = attempts
        motion_request.allowed_planning_time = planning_time
        motion_request.max_velocity_scaling_factor = velocity_scale
        motion_request.max_acceleration_scaling_factor = acceleration_scale

        response = self._wait_future(
            self._motion_plan_client.call_async(request),
            max(self._timeout_s, planning_time + 1.0),
        )
        if response is None:
            raise RuntimeError(f"MoveIt {label} OMPL plan returned no response")
        plan_response = response.motion_plan_response
        error_code = int(plan_response.error_code.val)
        if error_code != int(self._MoveItErrorCodes.SUCCESS):
            raise RuntimeError(
                f"MoveIt {label} OMPL plan failed: "
                f"code={error_code} "
                f"{getattr(plan_response.error_code, 'message', '')}"
            )
        trajectory = plan_response.trajectory
        final_positions = self.trajectory_final_positions(trajectory)
        if np.max(np.abs(final_positions - target)) > tolerance + 1e-6:
            raise RuntimeError(
                f"MoveIt {label} OMPL trajectory missed joint goal"
            )
        return trajectory

    def plan_cartesian_grasp_path(
        self,
        waypoint_transforms_base_to_end,
        *,
        grasp_index: int,
        start_positions: np.ndarray,
        max_step_m: float,
        revolute_jump_threshold_rad: float,
        max_joint_velocity_rad_s: float = 0.25,
    ) -> MoveItGraspPlan:
        transforms = tuple(
            np.asarray(transform, dtype=np.float64)
            for transform in waypoint_transforms_base_to_end
        )
        if len(transforms) < 3:
            raise ValueError("grasp path requires at least three waypoints")
        if not 0 < int(grasp_index) < len(transforms) - 1:
            raise ValueError("grasp_index must split outward and return paths")
        if any(transform.shape != (4, 4) for transform in transforms):
            raise ValueError("waypoint transforms must have shape (4, 4)")

        outward_trajectory = self.plan_cartesian_end_path(
            np.asarray(start_positions, dtype=np.float64),
            transforms[1 : int(grasp_index) + 1],
            max_step_m=max_step_m,
            revolute_jump_threshold_rad=revolute_jump_threshold_rad,
            max_joint_velocity_rad_s=max_joint_velocity_rad_s,
            label="outward",
        )
        outward_final = self.trajectory_final_positions(outward_trajectory)
        return_trajectory = self.plan_cartesian_end_path(
            outward_final,
            transforms[int(grasp_index) + 1 :],
            max_step_m=max_step_m,
            revolute_jump_threshold_rad=revolute_jump_threshold_rad,
            max_joint_velocity_rad_s=max_joint_velocity_rad_s,
            label="return",
        )
        return MoveItGraspPlan(
            outward_trajectory=outward_trajectory,
            return_trajectory=return_trajectory,
            outward_final_positions=outward_final,
            return_final_positions=self.trajectory_final_positions(
                return_trajectory
            ),
        )

    def plan_cartesian_end_path(
        self,
        start_positions: np.ndarray,
        waypoint_transforms_base_to_end,
        *,
        max_step_m: float,
        revolute_jump_threshold_rad: float,
        max_joint_velocity_rad_s: float = 0.25,
        gripper_open: bool = False,
        label: str,
    ):
        link6_transforms = tuple(
            np.asarray(transform, dtype=np.float64) @ self._T_end_to_link6
            for transform in waypoint_transforms_base_to_end
        )
        if not link6_transforms:
            raise ValueError("MoveIt Cartesian path requires a waypoint")
        return self._compute_cartesian_path(
            start_positions,
            link6_transforms,
            max_step_m=max_step_m,
            revolute_jump_threshold_rad=revolute_jump_threshold_rad,
            max_joint_velocity_rad_s=max_joint_velocity_rad_s,
            gripper_open=bool(gripper_open),
            label=label,
        )

    def diagnose_cartesian_end_path(
        self,
        start_positions: np.ndarray,
        waypoint_transforms_base_to_end,
        *,
        max_step_m: float,
        revolute_jump_threshold_rad: float,
    ) -> dict[str, tuple[int, float]]:
        """Probe partial Cartesian failures without returning executable paths."""
        link6_transforms = tuple(
            np.asarray(transform, dtype=np.float64) @ self._T_end_to_link6
            for transform in waypoint_transforms_base_to_end
        )
        probes = (
            ("no_jump", 0.0, True),
            ("no_collision", float(revolute_jump_threshold_rad), False),
            ("no_jump_no_collision", 0.0, False),
        )
        diagnostics: dict[str, tuple[int, float]] = {}
        for name, jump_threshold, avoid_collisions in probes:
            response = self._request_cartesian_path(
                start_positions,
                link6_transforms,
                max_step_m=max_step_m,
                revolute_jump_threshold_rad=jump_threshold,
                avoid_collisions=avoid_collisions,
                gripper_open=True,
                label=f"diagnostic_{name}",
            )
            diagnostics[name] = (
                int(response.error_code.val),
                float(response.fraction),
            )
        return diagnostics

    def _request_cartesian_path(
        self,
        start_positions: np.ndarray,
        waypoint_transforms_base_to_link6,
        *,
        max_step_m: float,
        revolute_jump_threshold_rad: float,
        avoid_collisions: bool,
        gripper_open: bool,
        label: str,
    ):
        request = self._GetCartesianPath.Request()
        request.header.frame_id = "base_link"
        request.start_state = self._robot_state(
            start_positions,
            gripper_joint_position_m=(
                self._gripper_open_joint_position_m
                if gripper_open
                else None
            ),
        )
        request.group_name = "arm"
        request.link_name = "link6"
        request.waypoints = [
            self._pose_message(transform)
            for transform in waypoint_transforms_base_to_link6
        ]
        request.max_step = float(max_step_m)
        request.jump_threshold = 0.0
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = float(
            revolute_jump_threshold_rad
        )
        request.avoid_collisions = bool(avoid_collisions)
        response = self._wait_future(
            self._cartesian_client.call_async(request),
            self._timeout_s,
        )
        if response is None:
            raise RuntimeError(f"MoveIt {label} Cartesian plan returned no response")
        return response

    def _compute_cartesian_path(
        self,
        start_positions: np.ndarray,
        waypoint_transforms_base_to_link6,
        *,
        max_step_m: float,
        revolute_jump_threshold_rad: float,
        max_joint_velocity_rad_s: float,
        gripper_open: bool,
        label: str,
    ):
        response = self._request_cartesian_path(
            start_positions,
            waypoint_transforms_base_to_link6,
            max_step_m=max_step_m,
            revolute_jump_threshold_rad=revolute_jump_threshold_rad,
            avoid_collisions=True,
            gripper_open=gripper_open,
            label=label,
        )
        error_code = int(response.error_code.val)
        fraction = float(response.fraction)
        if (
            error_code != int(self._MoveItErrorCodes.SUCCESS)
            or not math.isfinite(fraction)
            or fraction < 1.0 - 1e-6
        ):
            message = getattr(response.error_code, "message", "")
            raise RuntimeError(
                f"MoveIt {label} Cartesian plan failed: "
                f"code={error_code} fraction={fraction:.3f} {message}"
            )
        self.time_parameterize_trajectory(
            response.solution,
            start_positions=start_positions,
            max_joint_velocity_rad_s=max_joint_velocity_rad_s,
        )
        self.trajectory_final_positions(response.solution)
        return response.solution

    def _robot_state(
        self,
        positions: np.ndarray,
        *,
        gripper_joint_position_m: float | None = None,
    ):
        values = np.asarray(positions, dtype=np.float64).reshape(-1)
        if values.size != len(_ARM_JOINT_NAMES) or not np.all(
            np.isfinite(values)
        ):
            raise ValueError("MoveIt start state must contain six finite joints")
        names = list(_ARM_JOINT_NAMES)
        state_positions = [float(value) for value in values]
        if gripper_joint_position_m is not None:
            gripper_position = float(gripper_joint_position_m)
            if (
                not math.isfinite(gripper_position)
                or gripper_position < 0.0
                or gripper_position > 0.03258620689655173
            ):
                raise ValueError(
                    "MoveIt gripper joint position must be within [0, 0.03258620689655173] m"
                )
            names.extend(("gripper_joint", "right_joint"))
            state_positions.extend((gripper_position, gripper_position))
        joint_state = self._JointState()
        joint_state.name = names
        joint_state.position = state_positions
        state = self._RobotState()
        state.is_diff = True  # Preserve attached payloads and unspecified joints from the scene.
        state.joint_state = joint_state
        return state

    @staticmethod
    def trajectory_final_positions(trajectory) -> np.ndarray:
        joint_trajectory = trajectory.joint_trajectory
        if not joint_trajectory.points:
            raise RuntimeError("MoveIt returned an empty joint trajectory")
        final_by_name = dict(
            zip(
                joint_trajectory.joint_names,
                joint_trajectory.points[-1].positions,
            )
        )
        missing = [
            name for name in _ARM_JOINT_NAMES if name not in final_by_name
        ]
        if missing:
            raise RuntimeError(
                f"MoveIt trajectory missing joints: {','.join(missing)}"
            )
        return np.array(
            [final_by_name[name] for name in _ARM_JOINT_NAMES],
            dtype=np.float64,
        )

    def reverse_trajectory(
        self,
        trajectory,
        *,
        start_positions: np.ndarray,
        target_positions: np.ndarray,
        max_joint_velocity_rad_s: float,
    ):
        start = np.asarray(start_positions, dtype=np.float64).reshape(-1)
        target = np.asarray(target_positions, dtype=np.float64).reshape(-1)
        if (
            start.size != len(_ARM_JOINT_NAMES)
            or target.size != len(_ARM_JOINT_NAMES)
            or not np.all(np.isfinite(start))
            or not np.all(np.isfinite(target))
        ):
            raise ValueError(
                "reversed trajectory requires six finite start and target joints"
            )
        reversed_trajectory = copy.deepcopy(trajectory)
        joint_trajectory = reversed_trajectory.joint_trajectory
        if not joint_trajectory.points:
            raise RuntimeError("cannot reverse an empty trajectory")
        joint_names = tuple(joint_trajectory.joint_names)
        missing = [
            name for name in _ARM_JOINT_NAMES if name not in joint_names
        ]
        if missing:
            raise RuntimeError(
                f"MoveIt trajectory missing joints: {','.join(missing)}"
            )

        points = list(reversed(joint_trajectory.points))
        target_by_name = dict(zip(_ARM_JOINT_NAMES, target))
        target_in_trajectory_order = [
            float(target_by_name[name]) for name in joint_names
        ]
        final_positions = np.asarray(
            points[-1].positions,
            dtype=np.float64,
        )
        if (
            final_positions.shape != (len(joint_names),)
            or not np.all(np.isfinite(final_positions))
        ):
            raise RuntimeError(
                "MoveIt trajectory point has invalid joint positions"
            )
        if np.max(
            np.abs(final_positions - target_in_trajectory_order)
        ) > 1e-9:
            target_point = copy.deepcopy(points[-1])
            target_point.positions = target_in_trajectory_order
            points.append(target_point)
        else:
            points[-1].positions = target_in_trajectory_order

        for point in points:
            if hasattr(point, "velocities"):
                point.velocities = []
            if hasattr(point, "accelerations"):
                point.accelerations = []
            if hasattr(point, "effort"):
                point.effort = []
        joint_trajectory.points = points
        self.time_parameterize_trajectory(
            reversed_trajectory,
            start_positions=start,
            max_joint_velocity_rad_s=max_joint_velocity_rad_s,
        )
        reversed_final = self.trajectory_final_positions(
            reversed_trajectory
        )
        if np.max(np.abs(reversed_final - target)) > 1e-9:
            raise RuntimeError("reversed trajectory missed its target")
        return reversed_trajectory

    def time_parameterize_trajectory(
        self,
        trajectory,
        *,
        start_positions: np.ndarray,
        max_joint_velocity_rad_s: float,
    ) -> None:
        max_velocity = float(max_joint_velocity_rad_s)
        if not math.isfinite(max_velocity) or max_velocity <= 0.0:
            raise ValueError("MoveIt trajectory velocity must be positive")
        joint_trajectory = trajectory.joint_trajectory
        start = np.asarray(start_positions, dtype=np.float64).reshape(-1)
        if start.size != len(_ARM_JOINT_NAMES):
            raise ValueError("MoveIt start state must contain six joints")
        start_by_name = dict(zip(_ARM_JOINT_NAMES, start))
        previous = np.array(
            [start_by_name[name] for name in joint_trajectory.joint_names],
            dtype=np.float64,
        )
        positions_by_time = [previous.copy()]
        segment_durations = []
        elapsed = 0.0
        for point in joint_trajectory.points:
            positions = np.asarray(point.positions, dtype=np.float64)
            if positions.shape != previous.shape or not np.all(
                np.isfinite(positions)
            ):
                raise RuntimeError(
                    "MoveIt trajectory point has invalid joint positions"
                )
            step_duration = max(
                0.02,
                float(np.max(np.abs(positions - previous))) / max_velocity,
            )
            segment_durations.append(step_duration)
            elapsed += step_duration
            seconds = int(elapsed)
            nanoseconds = int(round((elapsed - seconds) * 1_000_000_000))
            if nanoseconds >= 1_000_000_000:
                seconds += 1
                nanoseconds -= 1_000_000_000
            duration = self._Duration()
            duration.sec = seconds
            duration.nanosec = nanoseconds
            point.time_from_start = duration
            positions_by_time.append(positions.copy())
            previous = positions

        if not joint_trajectory.points:
            return

        positions_array = np.stack(positions_by_time)
        durations = np.asarray(segment_durations, dtype=np.float64)
        secants = np.diff(positions_array, axis=0) / durations[:, None]
        waypoint_velocities = np.zeros_like(positions_array)

        # Monotone cubic slopes keep velocity continuous through waypoints while
        # avoiding overshoot. A direction change intentionally gets zero speed;
        # ordinary points on the same path no longer force stop-and-go motion.
        for index in range(1, len(positions_array) - 1):
            before = secants[index - 1]
            after = secants[index]
            same_direction = before * after > 0.0
            before_duration = durations[index - 1]
            after_duration = durations[index]
            weight_before = 2.0 * after_duration + before_duration
            weight_after = after_duration + 2.0 * before_duration
            denominator = np.ones_like(before)
            denominator[same_direction] = (
                weight_before / before[same_direction]
                + weight_after / after[same_direction]
            )
            waypoint_velocities[index, same_direction] = (
                weight_before + weight_after
            ) / denominator[same_direction]

        waypoint_velocities = np.clip(
            waypoint_velocities,
            -max_velocity,
            max_velocity,
        )
        for index, point in enumerate(joint_trajectory.points, start=1):
            point.velocities = [
                float(value) for value in waypoint_velocities[index]
            ]
            if hasattr(point, "accelerations"):
                point.accelerations = []

    @staticmethod
    def trajectory_duration_s(trajectory) -> float:
        points = trajectory.joint_trajectory.points
        if not points:
            return 0.0
        duration = points[-1].time_from_start
        return float(duration.sec) + float(duration.nanosec) * 1e-9

    def execute_moveit_trajectory(self, trajectory, timeout_s: float, *, cancel_event=None) -> None:
        goal = self._ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        result = self._run_action(
            self._execute_trajectory_client,
            goal,
            max(
                float(timeout_s),
                self.trajectory_duration_s(trajectory) + 5.0,
            ),
            cancel_event=cancel_event,
        )
        if int(result.error_code.val) != int(self._MoveItErrorCodes.SUCCESS):
            raise RuntimeError(
                "MoveIt trajectory execution failed: "
                f"code={int(result.error_code.val)} "
                f"{getattr(result.error_code, 'message', '')}"
            )

    def execute_planned_joint_trajectory(self, trajectory, *, cancel_event=None) -> None:
        """Send a collision-checked MoveIt plan through the controller task endpoint."""
        goal = self._FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        if tuple(goal.trajectory.joint_names) != _ARM_JOINT_NAMES:
            raise ValueError('planned trajectory must contain the six ordered arm joints')
        result = self._run_action(self._follow_joint_trajectory_client, goal,
            self.trajectory_duration_s(trajectory) + 8.0, cancel_event=cancel_event)
        if result.error_code != self._FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(result.error_string or 'planned controller trajectory failed')

    def execute_direct_joint_target(
        self,
        target_positions: np.ndarray,
        *,
        duration_s: float,
        cancel_event: threading.Event | None = None,
    ) -> None:
        target = np.asarray(target_positions, dtype=np.float64).reshape(-1)
        duration = float(duration_s)
        if target.size != len(_ARM_JOINT_NAMES) or not np.all(np.isfinite(target)):
            raise ValueError("direct joint target must contain six finite positions")
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("direct joint duration must be positive")

        trajectory = self._JointTrajectory()
        trajectory.joint_names = list(_ARM_JOINT_NAMES)
        # Explicitly provide the measured start state. A one-point goal makes
        # the controller infer the start asynchronously; after a grasp/lift
        # that can intermittently trigger a start-state/path rejection.
        start = np.asarray(self.latest_sample().positions, dtype=np.float64).reshape(-1)
        if start.size != len(_ARM_JOINT_NAMES) or not np.all(np.isfinite(start)):
            raise RuntimeError("cannot execute direct joint target without joint state")

        start_point = self._JointTrajectoryPoint()
        start_point.positions = [float(value) for value in start]
        start_point.time_from_start = self._Duration()
        start_point.time_from_start.sec = 0
        start_point.time_from_start.nanosec = 0

        target_point = self._JointTrajectoryPoint()
        target_point.positions = [float(value) for value in target]
        target_point.time_from_start = self._Duration()
        target_point.time_from_start.sec = int(duration)
        target_point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory.points = [start_point, target_point]

        goal = self._FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        if cancel_event is None:
            result = self._run_action(
                self._follow_joint_trajectory_client,
                goal,
                duration + 5.0,
            )
        else:
            result = self._run_action(
                self._follow_joint_trajectory_client,
                goal,
                duration + 5.0,
                cancel_event=cancel_event,
            )
        if int(result.error_code) != int(
            self._FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                "direct joint trajectory failed: "
                f"code={int(result.error_code)} "
                f"{getattr(result, 'error_string', '')}"
            )

    def execute_joint_trajectory(
        self,
        waypoints,
        *,
        cancel_event: threading.Event | None = None,
    ) -> None:
        if len(waypoints) < 2:
            raise ValueError("joint trajectory requires at least two waypoints")
        trajectory = self._JointTrajectory()
        trajectory.joint_names = list(_ARM_JOINT_NAMES)
        previous_time = -1.0
        for index, waypoint in enumerate(waypoints):
            positions = np.asarray(waypoint.positions, dtype=np.float64).reshape(-1)
            velocities = np.asarray(waypoint.velocities, dtype=np.float64).reshape(-1)
            timestamp = float(waypoint.time_from_start_s)
            if (
                positions.size != len(_ARM_JOINT_NAMES)
                or velocities.size != len(_ARM_JOINT_NAMES)
                or not np.all(np.isfinite(positions))
                or not np.all(np.isfinite(velocities))
                or not math.isfinite(timestamp)
                or timestamp < 0.0
                or (index > 0 and timestamp <= previous_time)
            ):
                raise ValueError("joint trajectory contains an invalid waypoint")
            point = self._JointTrajectoryPoint()
            point.positions = [float(value) for value in positions]
            point.velocities = [float(value) for value in velocities]
            seconds = int(timestamp)
            nanoseconds = int(round((timestamp - seconds) * 1_000_000_000))
            if nanoseconds >= 1_000_000_000:
                seconds += 1
                nanoseconds -= 1_000_000_000
            point.time_from_start = self._Duration()
            point.time_from_start.sec = seconds
            point.time_from_start.nanosec = nanoseconds
            trajectory.points.append(point)
            previous_time = timestamp

        goal = self._FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        if cancel_event is None:
            result = self._run_action(
                self._follow_joint_trajectory_client,
                goal,
                previous_time + 5.0,
            )
        else:
            result = self._run_action(
                self._follow_joint_trajectory_client,
                goal,
                previous_time + 5.0,
                cancel_event=cancel_event,
            )
        if int(result.error_code) != int(
            self._FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                "joint trajectory failed: "
                f"code={int(result.error_code)} "
                f"{getattr(result, 'error_string', '')}"
            )

    def apply_worktable_top(
        self,
        table_z_m: float,
        *,
        width_m: float,
        length_m: float,
        mount_inset_m: float,
        thickness_m: float,
    ) -> None:
        table_z = float(table_z_m)
        width = float(width_m)
        length = float(length_m)
        mount_inset = float(mount_inset_m)
        thickness = float(thickness_m)
        if not math.isfinite(table_z):
            raise ValueError("MoveIt worktable Z must be finite")
        if (
            not math.isfinite(width)
            or width <= 0.0
            or not math.isfinite(length)
            or length <= 0.0
            or not math.isfinite(mount_inset)
            or mount_inset < 0.0
            or mount_inset >= width / 2.0
            or not math.isfinite(thickness)
            or thickness <= 0.0
        ):
            raise ValueError("MoveIt worktable geometry is invalid")

        table = self._CollisionObject()
        table.header.frame_id = "base_link"
        table.id = "fixed_worktable"
        table.operation = self._CollisionObject.ADD

        box = self._SolidPrimitive()
        box.type = self._SolidPrimitive.BOX
        box.dimensions = [width, length, thickness]

        pose = self._Pose()
        pose.position.x = width / 2.0 - mount_inset
        pose.position.y = 0.0
        pose.position.z = table_z - thickness / 2.0
        pose.orientation.w = 1.0
        table.primitives = [box]
        table.primitive_poses = [pose]

        scene = self._PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [table]
        request = self._ApplyPlanningScene.Request()
        request.scene = scene
        response = self._wait_future(
            self._planning_scene_client.call_async(request),
            self._timeout_s,
        )
        if response is None or not bool(response.success):
            raise RuntimeError("MoveIt rejected measured worktable update")

    def _apply_planning_scene_diff(self, scene) -> None:
        scene.is_diff = True
        request = self._ApplyPlanningScene.Request()
        request.scene = scene
        response = self._wait_future(
            self._planning_scene_client.call_async(request),
            self._timeout_s,
        )
        if response is None or not bool(response.success):
            raise RuntimeError("MoveIt rejected planning-scene update")

    def apply_attached_collision_box(
        self,
        object_id: str,
        *,
        link_name: str,
        dimensions_m: tuple[float, float, float] | list[float],
        touch_links: tuple[str, ...] = (),
    ) -> None:
        """Attach a padded box to the TCP for subsequent MoveIt planning."""
        dimensions = [float(value) for value in dimensions_m]
        if len(dimensions) != 3 or not all(math.isfinite(value) and value > 0.0 for value in dimensions):
            raise ValueError("attached collision box dimensions are invalid")
        attached = self._AttachedCollisionObject()
        attached.link_name = str(link_name)
        attached.touch_links = list(touch_links)
        obj = self._CollisionObject()
        obj.id = str(object_id)
        obj.header.frame_id = str(link_name)
        obj.operation = self._CollisionObject.ADD
        primitive = self._SolidPrimitive()
        primitive.type = self._SolidPrimitive.BOX
        primitive.dimensions = dimensions
        pose = self._Pose()
        pose.orientation.w = 1.0
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        attached.object = obj
        scene = self._PlanningScene()
        scene.robot_state = self._RobotState()
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        self._apply_planning_scene_diff(scene)

    def set_payload_table_contact(self, object_id: str, allowed: bool) -> None:
        """Allow the picked object's initial table contact, preserving other pairs."""
        from moveit_msgs.msg import AllowedCollisionEntry, PlanningSceneComponents
        if not self._get_planning_scene_client.wait_for_service(timeout_sec=self._timeout_s):
            raise RuntimeError('MoveIt get_planning_scene unavailable')
        request = self._GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        response = self._wait_future(self._get_planning_scene_client.call_async(request), self._timeout_s)
        if response is None:
            raise RuntimeError('no collision matrix received')
        matrix = response.scene.allowed_collision_matrix
        for name in (str(object_id), 'fixed_worktable'):
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        object_index = matrix.entry_names.index(str(object_id))
        table_index = matrix.entry_names.index('fixed_worktable')
        matrix.entry_values[object_index].enabled[table_index] = bool(allowed)
        matrix.entry_values[table_index].enabled[object_index] = bool(allowed)
        scene = self._PlanningScene()
        scene.allowed_collision_matrix = matrix
        scene.robot_state.is_diff = True
        self._apply_planning_scene_diff(scene)

    def remove_attached_collision_box(self, object_id: str, *, link_name: str) -> None:
        attached = self._AttachedCollisionObject()
        attached.link_name = str(link_name)
        obj = self._CollisionObject()
        obj.id = str(object_id)
        obj.operation = getattr(self._CollisionObject, "REMOVE", 1)
        attached.object = obj
        scene = self._PlanningScene()
        scene.robot_state = self._RobotState()
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        self._apply_planning_scene_diff(scene)

    def apply_world_collision_box(
        self,
        object_id: str,
        *,
        center_xyz_m: tuple[float, float, float] | list[float],
        dimensions_m: tuple[float, float, float] | list[float],
    ) -> None:
        center = [float(value) for value in center_xyz_m]
        dimensions = [float(value) for value in dimensions_m]
        if len(center) != 3 or len(dimensions) != 3:
            raise ValueError("world collision box requires xyz and dimensions")
        if not all(math.isfinite(value) for value in center) or not all(
            math.isfinite(value) and value > 0.0 for value in dimensions
        ):
            raise ValueError("world collision box values are invalid")
        obj = self._CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = str(object_id)
        obj.operation = self._CollisionObject.ADD
        primitive = self._SolidPrimitive()
        primitive.type = self._SolidPrimitive.BOX
        primitive.dimensions = dimensions
        pose = self._Pose()
        pose.position.x, pose.position.y, pose.position.z = center
        pose.orientation.w = 1.0
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        scene = self._PlanningScene()
        scene.world.collision_objects = [obj]
        self._apply_planning_scene_diff(scene)

    def remove_world_collision_object(self, object_id: str) -> None:
        obj = self._CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = str(object_id)
        obj.operation = getattr(self._CollisionObject, "REMOVE", 1)
        scene = self._PlanningScene()
        scene.world.collision_objects = [obj]
        self._apply_planning_scene_diff(scene)

    def start_gravity_compensation(self) -> None:
        self._call_trigger_service(
            self._gc_start_client,
            "start gravity compensation",
        )

    def stop_gravity_compensation(self) -> None:
        self._call_trigger_service(
            self._gc_stop_client,
            "stop gravity compensation",
        )

    def safe_home(self) -> None:
        self._call_trigger_service(
            self._safe_home_client,
            "safe home",
            timeout_s=self._safe_home_timeout_s,
        )

    def disable(self) -> None:
        self._call_trigger_service(
            self._disable_client,
            "disable",
        )

    def stop_and_hold(self) -> None:
        self._call_trigger_service(self._stop_client, "stop and hold")

    def open_gripper(self) -> None:
        request = self._GripperCommand.Request()
        future = self._open_client.call_async(request)
        result = self._wait_future(future, self._timeout_s)
        if result is None or not result.success:
            message = result.message if result is not None else "no response"
            raise RuntimeError(f"gripper open failed: {message}")

    def grasp(self, *, closing_torque: float = 0.0, hold_torque: float = 0.0, cancel_event=None) -> bool:
        goal = self._GripperGrasp.Goal()
        goal.closing_torque = float(closing_torque)
        goal.hold_torque = float(hold_torque)
        goal.timeout = 0.0
        result = self._run_action(
            self._grasp_client,
            goal,
            self._timeout_s,
            cancel_event=cancel_event,
        )
        return bool(result.object_detected)

    def _run_action(
        self,
        client,
        goal,
        timeout_s: float,
        *,
        cancel_event: threading.Event | None = None,
    ):
        if cancel_event is not None and cancel_event.is_set():
            raise RuntimeError("ROS action cancelled before sending")
        with self._active_goal_lock:
            if self._active_goal is not None or self._action_starting:
                raise RuntimeError("another ROS action is already active")
            self._action_starting = True
        try:
            goal_handle = self._wait_future(
                client.send_goal_async(goal),
                self._timeout_s,
            )
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("ROS action goal rejected")
        except BaseException:
            with self._active_goal_lock:
                self._action_starting = False
            raise
        with self._active_goal_lock:
            self._action_starting = False
            self._active_goal = goal_handle
            self._cancel_requested_goal = None
        try:
            if cancel_event is not None and cancel_event.is_set():
                self.cancel_active(timeout_s=2.0)
            wrapped = self._wait_future(
                goal_handle.get_result_async(),
                max(float(timeout_s), self._timeout_s),
                cancel_event=cancel_event,
            )
            if wrapped is None:
                raise TimeoutError("ROS action returned no result")
            if wrapped.status != self._GoalStatus.STATUS_SUCCEEDED:
                message = getattr(wrapped.result, "message", "action failed")
                raise RuntimeError(message)
            return wrapped.result
        except BaseException:
            try:
                self.cancel_active(timeout_s=2.0)
            except Exception:
                pass
            raise
        finally:
            with self._active_goal_lock:
                if self._active_goal is goal_handle:
                    self._active_goal = None
                if self._cancel_requested_goal is goal_handle:
                    self._cancel_requested_goal = None

    def cancel_active(self, timeout_s: float = 2.0) -> None:
        with self._active_goal_lock:
            goal = self._active_goal
            if goal is None or self._cancel_requested_goal is goal:
                return
            self._cancel_requested_goal = goal
        future = goal.cancel_goal_async()
        self._wait_future(future, timeout_s)

    def _wait_future(self, future, timeout_s: float, *, cancel_event=None):
        deadline = time.monotonic() + float(timeout_s)
        while not future.done():
            if cancel_event is not None and cancel_event.is_set():
                raise RuntimeError('ROS request cancelled')
            if time.monotonic() >= deadline:
                raise TimeoutError("ROS request timed out")
            time.sleep(0.01)
        return future.result()

    def _call_trigger_service(
        self,
        client,
        label: str,
        *,
        timeout_s: float | None = None,
    ) -> None:
        timeout = self._timeout_s if timeout_s is None else float(timeout_s)
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f"{label} service not available")
        result = self._wait_future(
            client.call_async(self._Trigger.Request()),
            timeout,
        )
        if result is None or not result.success:
            message = result.message if result is not None else "no response"
            raise RuntimeError(f"{label} failed: {message}")

    def _pose_message(self, transform: np.ndarray):
        message = self._Pose()
        message.position.x, message.position.y, message.position.z = (
            float(value) for value in transform[:3, 3]
        )
        qx, qy, qz, qw = _rotation_to_quaternion(transform[:3, :3])
        message.orientation.x = qx
        message.orientation.y = qy
        message.orientation.z = qz
        message.orientation.w = qw
        return message

    def close(self) -> None:
        try:
            self.cancel_active()
        except Exception:
            pass
        self._executor.shutdown(timeout_sec=2.0)
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        self._node.destroy_node()
        if self._owns_rclpy and self._rclpy.ok():
            self._rclpy.shutdown()
