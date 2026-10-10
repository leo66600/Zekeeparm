#!/usr/bin/env python3
"""固定点物料搬运教学节点。

节点执行 9 步 Pick and Place 标准时序。所有教学点位、安全限速和夹爪参数
均从 YAML 参数读取，代码中不硬编码坐标或速度。
"""

from __future__ import annotations

import math
from copy import deepcopy
from typing import Iterable

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from zekeep_msgs.srv import GripperCommand


def _duration_from_seconds(seconds: float) -> Duration:
    """将浮点秒转换为 ROS Duration。"""
    duration = Duration()
    duration.sec = int(seconds)
    duration.nanosec = int((seconds - duration.sec) * 1_000_000_000)
    return duration


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """不依赖额外 Python 包的 RPY 到四元数转换。"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class PickAndPlaceNode(Node):
    """读取 YAML 参数并阻塞执行固定取放流程。"""

    def __init__(self) -> None:
        super().__init__("pick_and_place_node")

        self.declare_parameter("sim", True)
        self.declare_parameter("auto_start", True)
        self._declare_task_parameters()
        self._load_parameters()

        self._plan_client = self.create_client(GetMotionPlan, self.motion_plan_service)
        self._cartesian_client = self.create_client(
            GetCartesianPath, self.cartesian_path_service
        )
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, self.execute_trajectory_action
        )
        self._sim_gripper_client = ActionClient(
            self, FollowJointTrajectory, self.sim_gripper_action
        )
        self._hardware_open_client = self.create_client(
            GripperCommand, self.hardware_gripper_open_service
        )
        self._hardware_close_client = self.create_client(
            GripperCommand, self.hardware_gripper_close_service
        )

    def _declare_task_parameters(self) -> None:
        """声明 YAML 中使用的全部参数，便于 launch 私有命名空间加载。"""
        declarations = {
            "arm_group": "arm",
            "gripper_group": "gripper",
            "end_effector_link": "link6",
            "reference_frame": "base_link",
            "home_named_target": "ready_pose",
            "home_joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            "home_joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "velocity_scaling": 0.2,
            "acceleration_scaling": 0.15,
            "planning_time": 5.0,
            "planning_attempts": 5,
            "interface_wait_timeout": 60.0,
            "cartesian_max_step": 0.01,
            "cartesian_jump_threshold": 0.0,
            "cartesian_min_fraction": 0.95,
            "avoid_collisions": True,
            "hover_height": 0.10,
            "pick_pose.position.x": 0.0,
            "pick_pose.position.y": 0.0,
            "pick_pose.position.z": 0.0,
            "pick_pose.rpy.roll": 0.0,
            "pick_pose.rpy.pitch": 0.0,
            "pick_pose.rpy.yaw": 0.0,
            "place_pose.position.x": 0.0,
            "place_pose.position.y": 0.0,
            "place_pose.position.z": 0.0,
            "place_pose.rpy.roll": 0.0,
            "place_pose.rpy.pitch": 0.0,
            "place_pose.rpy.yaw": 0.0,
            "gripper.joint_name": "gripper_joint",
            "gripper.open_named_target": "open",
            "gripper.close_named_target": "closed",
            "gripper.open_position": 0.03258620689655173,
            "gripper.close_position": 0.0,
            "gripper.hardware_open_position": 1.35,
            "gripper.hardware_close_position": 0.0,
            "gripper.closing_torque": 1.0,
            "gripper.hold_torque": 0.30,
            "gripper.max_effort": 1.5,
            "gripper.command_duration": 2.0,
            "gripper.wait_timeout": 5.0,
            "arm_namespace": "zekeep",
            "sim_gripper_action": "/gripper_controller/follow_joint_trajectory",
            "hardware_gripper_open_service": "/zekeep/gripper/open",
            "hardware_gripper_close_service": "/zekeep/gripper/close",
            "motion_plan_service": "/plan_kinematic_path",
            "cartesian_path_service": "/compute_cartesian_path",
            "execute_trajectory_action": "/execute_trajectory",
        }
        for name, value in declarations.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        """缓存参数，并做安全范围检查。"""
        self.sim = bool(self.get_parameter("sim").value)
        self.arm_group = str(self.get_parameter("arm_group").value)
        self.end_effector_link = str(self.get_parameter("end_effector_link").value)
        self.reference_frame = str(self.get_parameter("reference_frame").value)
        self.home_named_target = str(self.get_parameter("home_named_target").value)
        self.home_joint_names = [
            str(value) for value in self.get_parameter("home_joint_names").value
        ]
        self.home_joint_positions = [
            float(value) for value in self.get_parameter("home_joint_positions").value
        ]
        self.velocity_scaling = float(self.get_parameter("velocity_scaling").value)
        self.acceleration_scaling = float(
            self.get_parameter("acceleration_scaling").value
        )
        self.planning_time = float(self.get_parameter("planning_time").value)
        self.planning_attempts = int(self.get_parameter("planning_attempts").value)
        self.interface_wait_timeout = float(
            self.get_parameter("interface_wait_timeout").value
        )
        self.cartesian_max_step = float(self.get_parameter("cartesian_max_step").value)
        self.cartesian_jump_threshold = float(
            self.get_parameter("cartesian_jump_threshold").value
        )
        self.cartesian_min_fraction = float(
            self.get_parameter("cartesian_min_fraction").value
        )
        self.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)
        self.hover_height = float(self.get_parameter("hover_height").value)

        self.gripper_joint_name = str(self.get_parameter("gripper.joint_name").value)
        self.gripper_open_position = float(
            self.get_parameter("gripper.open_position").value
        )
        self.gripper_close_position = float(
            self.get_parameter("gripper.close_position").value
        )
        self.hardware_gripper_open_position = float(
            self.get_parameter("gripper.hardware_open_position").value
        )
        self.hardware_gripper_close_position = float(
            self.get_parameter("gripper.hardware_close_position").value
        )
        self.gripper_command_duration = float(
            self.get_parameter("gripper.command_duration").value
        )
        self.gripper_wait_timeout = float(
            self.get_parameter("gripper.wait_timeout").value
        )

        self.sim_gripper_action = str(self.get_parameter("sim_gripper_action").value)
        self.hardware_gripper_open_service = str(
            self.get_parameter("hardware_gripper_open_service").value
        )
        self.hardware_gripper_close_service = str(
            self.get_parameter("hardware_gripper_close_service").value
        )
        self.motion_plan_service = str(self.get_parameter("motion_plan_service").value)
        self.cartesian_path_service = str(
            self.get_parameter("cartesian_path_service").value
        )
        self.execute_trajectory_action = str(
            self.get_parameter("execute_trajectory_action").value
        )

        if not 0.0 < self.velocity_scaling <= 0.2:
            raise ValueError("velocity_scaling must be in (0.0, 0.2]")
        if not 0.0 < self.acceleration_scaling <= 1.0:
            raise ValueError("acceleration_scaling must be in (0.0, 1.0]")
        if self.hover_height <= 0.0:
            raise ValueError("hover_height must be positive")
        if self.cartesian_max_step <= 0.0:
            raise ValueError("cartesian_max_step must be positive")
        if len(self.home_joint_names) != len(self.home_joint_positions):
            raise ValueError("home_joint_names and home_joint_positions length mismatch")
        if not self.home_joint_names:
            raise ValueError("home_joint_names must not be empty")

        self.pick_pose = self._read_pose("pick_pose")
        self.place_pose = self._read_pose("place_pose")
        self.pick_approach_pose = self._with_hover(self.pick_pose)
        self.place_approach_pose = self._with_hover(self.place_pose)

    def _read_pose(self, prefix: str) -> Pose:
        """从参数树读取一个笛卡尔位姿。"""
        pose = Pose()
        pose.position.x = float(self.get_parameter(f"{prefix}.position.x").value)
        pose.position.y = float(self.get_parameter(f"{prefix}.position.y").value)
        pose.position.z = float(self.get_parameter(f"{prefix}.position.z").value)
        roll = float(self.get_parameter(f"{prefix}.rpy.roll").value)
        pitch = float(self.get_parameter(f"{prefix}.rpy.pitch").value)
        yaw = float(self.get_parameter(f"{prefix}.rpy.yaw").value)
        qx, qy, qz, qw = _quaternion_from_rpy(roll, pitch, yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def _with_hover(self, pose: Pose) -> Pose:
        """生成目标点正上方安全抬升位。"""
        hover = deepcopy(pose)
        hover.position.z += self.hover_height
        return hover

    def run_task(self) -> None:
        """严格按 9 步标准时序阻塞执行。"""
        self._wait_for_interfaces()

        self.get_logger().info("1/9 张开夹爪")
        self._command_gripper(opening=True)

        self.get_logger().info("2/9 移动至预抓取位")
        self._plan_and_execute_pose(self.pick_approach_pose)

        self.get_logger().info("3/9 直线下降至抓取位")
        self._execute_cartesian([self.pick_pose])

        self.get_logger().info("4/9 闭合夹爪并等待")
        self._command_gripper(opening=False)

        self.get_logger().info("5/9 垂直抬升")
        self._execute_cartesian([self.pick_approach_pose])

        self.get_logger().info("6/9 平移至预放置位")
        self._plan_and_execute_pose(self.place_approach_pose)

        self.get_logger().info("7/9 下降至放置位")
        self._execute_cartesian([self.place_pose])

        self.get_logger().info("8/9 张开夹爪释放物料")
        self._command_gripper(opening=True)

        self.get_logger().info("9/9 安全退出并回归 Home")
        self._execute_cartesian([self.place_approach_pose])
        self._plan_and_execute_named_home()

    def _wait_for_interfaces(self) -> None:
        """等待所需服务/action 上线，保证后续动作串行可靠。"""
        for client, name in (
            (self._plan_client, self.motion_plan_service),
            (self._cartesian_client, self.cartesian_path_service),
        ):
            if not client.wait_for_service(timeout_sec=self.interface_wait_timeout):
                raise TimeoutError(f"service unavailable: {name}")
        if not self._execute_client.wait_for_server(
            timeout_sec=self.interface_wait_timeout
        ):
            raise TimeoutError(f"action unavailable: {self.execute_trajectory_action}")

        if self.sim:
            if not self._sim_gripper_client.wait_for_server(
                timeout_sec=self.interface_wait_timeout
            ):
                raise TimeoutError(f"action unavailable: {self.sim_gripper_action}")
        else:
            for client, name in (
                (self._hardware_open_client, self.hardware_gripper_open_service),
                (self._hardware_close_client, self.hardware_gripper_close_service),
            ):
                if not client.wait_for_service(timeout_sec=self.interface_wait_timeout):
                    raise TimeoutError(f"service unavailable: {name}")

    def _pose_stamped(self, pose: Pose) -> PoseStamped:
        stamped = PoseStamped()
        stamped.header.frame_id = self.reference_frame
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose = pose
        return stamped

    def _goal_constraints_for_pose(self, pose: Pose) -> Constraints:
        """构造 MoveIt 位姿目标约束。"""
        constraints = Constraints()

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01]

        volume = BoundingVolume()
        volume.primitives.append(primitive)
        volume.primitive_poses.append(pose)

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.reference_frame
        position_constraint.link_name = self.end_effector_link
        position_constraint.constraint_region = volume
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.reference_frame
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.02
        orientation_constraint.absolute_y_axis_tolerance = 0.02
        orientation_constraint.absolute_z_axis_tolerance = 0.02
        orientation_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        return constraints

    def _plan_and_execute_pose(self, pose: Pose) -> None:
        """规划到目标位姿并阻塞执行。"""
        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = self.arm_group
        request.motion_plan_request.num_planning_attempts = self.planning_attempts
        request.motion_plan_request.allowed_planning_time = self.planning_time
        request.motion_plan_request.max_velocity_scaling_factor = self.velocity_scaling
        request.motion_plan_request.max_acceleration_scaling_factor = (
            self.acceleration_scaling
        )
        request.motion_plan_request.start_state = RobotState(is_diff=True)
        request.motion_plan_request.goal_constraints.append(
            self._goal_constraints_for_pose(pose)
        )
        response = self._call_service(self._plan_client, request, self.planning_time + 5.0)
        error_code = response.motion_plan_response.error_code
        if int(error_code.val) != int(MoveItErrorCodes.SUCCESS):
            raise RuntimeError(f"MoveIt planning failed, code={int(error_code.val)}")
        self._execute_trajectory(response.motion_plan_response.trajectory)

    def _plan_and_execute_named_home(self) -> None:
        """回 Home。当前 SRDF 的 ready_pose 已在 YAML 中命名为 home_named_target。"""
        # MoveIt 服务接口不能直接接收 named target；YAML 将 SRDF named target 展开为关节约束。
        constraints = Constraints()
        for joint_name, value in zip(self.home_joint_names, self.home_joint_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = value
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = self.arm_group
        request.motion_plan_request.num_planning_attempts = self.planning_attempts
        request.motion_plan_request.allowed_planning_time = self.planning_time
        request.motion_plan_request.max_velocity_scaling_factor = self.velocity_scaling
        request.motion_plan_request.max_acceleration_scaling_factor = (
            self.acceleration_scaling
        )
        request.motion_plan_request.start_state = RobotState(is_diff=True)
        request.motion_plan_request.goal_constraints.append(constraints)
        response = self._call_service(self._plan_client, request, self.planning_time + 5.0)
        error_code = response.motion_plan_response.error_code
        if int(error_code.val) != int(MoveItErrorCodes.SUCCESS):
            raise RuntimeError(f"Home planning failed, code={int(error_code.val)}")
        self._execute_trajectory(response.motion_plan_response.trajectory)

    def _execute_cartesian(self, waypoints: Iterable[Pose]) -> None:
        """计算并执行直线笛卡尔路径。"""
        request = GetCartesianPath.Request()
        request.header.frame_id = self.reference_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state = RobotState(is_diff=True)
        request.group_name = self.arm_group
        request.link_name = self.end_effector_link
        request.waypoints = list(waypoints)
        request.max_step = self.cartesian_max_step
        request.jump_threshold = self.cartesian_jump_threshold
        request.avoid_collisions = self.avoid_collisions
        response = self._call_service(self._cartesian_client, request, self.planning_time + 5.0)
        if response.fraction < self.cartesian_min_fraction:
            raise RuntimeError(
                "Cartesian path incomplete: "
                f"fraction={response.fraction:.3f}, "
                f"required={self.cartesian_min_fraction:.3f}"
            )
        self._execute_trajectory(response.solution)

    def _execute_trajectory(self, trajectory) -> None:
        """调用 MoveIt 执行动作并阻塞等待结果。"""
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._execute_client.send_goal_async(goal)
        goal_handle = self._wait_future(future, 10.0)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("execute_trajectory goal rejected")
        result = self._wait_future(
            goal_handle.get_result_async(),
            self._trajectory_timeout(trajectory) + 10.0,
        )
        error_code = result.result.error_code
        if int(error_code.val) != int(MoveItErrorCodes.SUCCESS):
            raise RuntimeError(f"execute_trajectory failed, code={int(error_code.val)}")

    def _trajectory_timeout(self, trajectory) -> float:
        points = trajectory.joint_trajectory.points
        if not points:
            return 5.0
        duration = points[-1].time_from_start
        return float(duration.sec) + float(duration.nanosec) * 1e-9

    def _command_gripper(self, *, opening: bool) -> None:
        """根据 sim 参数选择夹爪接口，并阻塞等待完成。"""
        if self.sim:
            self._command_sim_gripper(
                self.gripper_open_position if opening else self.gripper_close_position
            )
        else:
            client = (
                self._hardware_open_client if opening else self._hardware_close_client
            )
            request = GripperCommand.Request()
            request.position = (
                self.hardware_gripper_open_position
                if opening
                else self.hardware_gripper_close_position
            )
            request.timeout = self.gripper_wait_timeout
            response = self._call_service(client, request, self.gripper_wait_timeout + 2.0)
            if not response.success:
                raise RuntimeError(f"hardware gripper command failed: {response.message}")

    def _command_sim_gripper(self, position: float) -> None:
        """仿真夹爪通过 FollowJointTrajectory 控制 gripper_joint。"""
        trajectory = JointTrajectory()
        trajectory.joint_names = [self.gripper_joint_name]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = _duration_from_seconds(self.gripper_command_duration)
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._sim_gripper_client.send_goal_async(goal)
        goal_handle = self._wait_future(future, 10.0)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("gripper trajectory goal rejected")
        result = self._wait_future(
            goal_handle.get_result_async(),
            self.gripper_wait_timeout + self.gripper_command_duration,
        )
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                "gripper trajectory failed: "
                f"code={result.result.error_code} {result.result.error_string}"
            )

    def _call_service(self, client, request, timeout_s: float):
        future = client.call_async(request)
        result = self._wait_future(future, timeout_s)
        if result is None:
            raise TimeoutError(
                f"service timeout: {getattr(client, 'srv_name', 'unknown')}"
            )
        return result

    def _wait_future(self, future, timeout_s: float):
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_s
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds / 1e9 > deadline:
                return None
        return future.result()


def main() -> None:
    rclpy.init()
    node = PickAndPlaceNode()
    try:
        if bool(node.get_parameter("auto_start").value):
            node.run_task()
            node.get_logger().info("Pick and Place 教学任务完成")
        else:
            try:
                rclpy.spin(node)
            except ExternalShutdownException:
                pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
