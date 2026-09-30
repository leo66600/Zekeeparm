#!/usr/bin/env python3
"""固定点取放教学节点。

给初学者阅读的顺序：
1. 在 run_task() 看完整动作流程；
2. 在 move_to()/move_straight() 看机械臂如何运动；
3. 在 open_gripper()/grasp() 看夹爪如何控制。
"""

from __future__ import annotations

import math
import threading
import time
from copy import deepcopy

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
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from zekeep_msgs.action import GripperGrasp
from zekeep_msgs.srv import GripperCommand


ARM_GROUP = "arm"
TCP_LINK = "grasp_tcp"
BASE_FRAME = "base_link"
HOME_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")

PLANNING_ATTEMPTS = 5
INTERFACE_TIMEOUT = 60.0
GOAL_TIMEOUT = 10.0
GRIPPER_TIMEOUT = 5.0
SIM_GRIPPER_DURATION = 2.0
# A shorter path changes the configured descent distance. Reject it instead of
# executing a trajectory that stops above the requested TCP target.
MIN_CARTESIAN_FRACTION = 0.999999

SIM_GRIPPER_ACTION = "/gripper_controller/follow_joint_trajectory"
HARDWARE_OPEN_SERVICE = "/zekeep/gripper/open"
HARDWARE_GRASP_ACTION = "/zekeep/gripper/grasp"
PLAN_SERVICE = "/plan_kinematic_path"
CARTESIAN_SERVICE = "/compute_cartesian_path"
EXECUTE_ACTION = "/execute_trajectory"
START_SERVICE = "/carry/start"


def duration(seconds: float) -> Duration:
    """把秒数转换为 ROS 消息使用的 Duration。"""
    value = Duration()
    value.sec = int(seconds)
    value.nanosec = int((seconds - value.sec) * 1_000_000_000)
    return value


def quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """把更容易填写的 RPY 欧拉角转换成四元数。"""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class CarryNode(Node):
    """读取 YAML 参数，执行固定的抓取和放置流程。"""

    def __init__(self) -> None:
        super().__init__("carry_node")
        self._declare_parameters()
        self._read_parameters()
        self._running = False
        self._lock = threading.Lock()

        # MoveIt: 关节规划、直线笛卡尔路径、执行轨迹。
        self.plan_client = self.create_client(GetMotionPlan, PLAN_SERVICE)
        self.cartesian_client = self.create_client(GetCartesianPath, CARTESIAN_SERVICE)
        self.execute_client = ActionClient(self, ExecuteTrajectory, EXECUTE_ACTION)

        # 夹爪：仿真用轨迹 Action；真机用张开服务和接触检测抓取 Action。
        self.sim_gripper_client = ActionClient(
            self, FollowJointTrajectory, SIM_GRIPPER_ACTION
        )
        self.grasp_client = ActionClient(self, GripperGrasp, HARDWARE_GRASP_ACTION)
        self.open_client = self.create_client(GripperCommand, HARDWARE_OPEN_SERVICE)

        self.create_service(Trigger, START_SERVICE, self._start_callback)
        self.auto_start_timer = None
        if self.get_parameter("auto_start").value:
            self.auto_start_timer = self.create_timer(0.5, self._start_once)

    # ------------------------------------------------------------------
    # 参数：实际数值都放在 config/carry_params.yaml，便于初学者调试。
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        defaults = {
            "sim": True,
            "auto_start": False,
            "preview_only": False,
            "home_joint_positions": [0.0] * 6,
            "velocity_scaling": 0.4,
            "acceleration_scaling": 0.15,
            "planning_time": 5.0,
            "cartesian_max_step": 0.01,
            "hover_height": 0.05,
            "position_tolerance": 0.005,
            "orientation_tolerance": 0.02,
            "pick_pose.position.x": 0.0,
            "pick_pose.position.y": 0.0,
            "pick_pose.position.z": 0.0,
            "place_pose.position.x": 0.0,
            "place_pose.position.y": 0.0,
            "place_pose.position.z": 0.0,
            "tool_rpy.roll": 0.0,
            "tool_rpy.pitch": 0.0,
            "tool_rpy.yaw": 0.0,
            "gripper.open_position": 0.035,
            "gripper.close_position": 0.0,
            "gripper.hardware_open_position": 1.15,
            "gripper.closing_torque": 1.0,
            "gripper.hold_torque": 0.30,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        get = lambda name: self.get_parameter(name).value

        self.sim = bool(get("sim"))
        self.preview_only = bool(get("preview_only"))
        self.home_joint_positions = list(get("home_joint_positions"))

        self.velocity_scaling = float(get("velocity_scaling"))
        self.acceleration_scaling = float(get("acceleration_scaling"))
        self.planning_time = float(get("planning_time"))
        self.cartesian_max_step = float(get("cartesian_max_step"))
        self.hover_height = float(get("hover_height"))
        self.position_tolerance = float(get("position_tolerance"))
        self.orientation_tolerance = float(get("orientation_tolerance"))

        self.gripper_open_position = float(get("gripper.open_position"))
        self.gripper_close_position = float(get("gripper.close_position"))
        self.hardware_open_position = float(get("gripper.hardware_open_position"))
        self.closing_torque = float(get("gripper.closing_torque"))
        self.hold_torque = float(get("gripper.hold_torque"))

        self._check_parameters()

        orientation = quaternion_from_rpy(
            float(get("tool_rpy.roll")),
            float(get("tool_rpy.pitch")),
            float(get("tool_rpy.yaw")),
        )
        self.pick_pose = self._read_pose("pick_pose", orientation)
        self.place_pose = self._read_pose("place_pose", orientation)
        self.pick_above = self._above(self.pick_pose)
        self.place_above = self._above(self.place_pose)

    def _check_parameters(self) -> None:
        """在启动前拒绝明显不安全的参数。"""
        if not 0 < self.velocity_scaling <= 1:
            raise ValueError("velocity_scaling 必须在 (0, 1] 内")
        if not 0 < self.acceleration_scaling <= 1:
            raise ValueError("acceleration_scaling 必须在 (0, 1] 内")
        if self.hover_height <= 0 or self.cartesian_max_step <= 0:
            raise ValueError("hover_height 和 cartesian_max_step 必须大于 0")
        if len(HOME_JOINT_NAMES) != len(self.home_joint_positions):
            raise ValueError("Home 关节名和关节位置数量不一致")

    def _read_pose(
        self,
        name: str,
        orientation: tuple[float, float, float, float],
    ) -> Pose:
        pose = Pose()
        pose.position.x = float(self.get_parameter(f"{name}.position.x").value)
        pose.position.y = float(self.get_parameter(f"{name}.position.y").value)
        pose.position.z = float(self.get_parameter(f"{name}.position.z").value)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
            orientation
        )
        return pose

    def _above(self, pose: Pose) -> Pose:
        """返回目标点正上方的安全位，避免水平移动时擦碰物料。"""
        result = deepcopy(pose)
        result.position.z += self.hover_height
        return result

    # ------------------------------------------------------------------
    # 任务流程：这里就是初学者最需要阅读和修改的部分。
    # ------------------------------------------------------------------

    def _start_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        response.success = self._start_task()
        response.message = "carry task started" if response.success else "task is running"
        return response

    def _start_once(self) -> None:
        self.auto_start_timer.cancel()
        self._start_task()

    def _start_task(self) -> bool:
        with self._lock:
            if self._running:
                return False
            self._running = True
        threading.Thread(target=self._run_in_background, daemon=True).start()
        return True

    def _run_in_background(self) -> None:
        try:
            self.run_task()
            self.get_logger().info("搬运完成")
        except Exception as error:
            self.get_logger().error(f"搬运失败: {error}")
        finally:
            with self._lock:
                self._running = False

    def run_task(self) -> None:
        """固定动作顺序。改变点位请改 YAML，不要在这里写坐标。"""
        self._wait_for_interfaces()

        self.open_gripper()
        self.move_to(self.pick_above)
        self.wait(0.5, "预抓取位")

        self.move_straight(self.pick_pose)
        self.wait(1.0, "抓取位")
        self.grasp()
        self.wait(1.0, "夹紧后")

        self.move_straight(self.pick_above)
        self.move_to(self.place_above, keep_orientation=True)
        self.wait(1.0, "预放置位")

        self.move_straight(self.place_pose)
        self.open_gripper()
        self.wait(1.0, "释放后")

        self.move_straight(self.place_above)
        self.wait(0.5, "放置点上方")
        self.move_home()

    def wait(self, seconds: float, place: str) -> None:
        if seconds > 0:
            self.get_logger().info(f"在{place}等待 {seconds:.1f}s")
            if not self.preview_only:
                time.sleep(seconds)

    # ------------------------------------------------------------------
    # MoveIt：move_to 是普通规划；move_straight 是末端直线运动。
    # ------------------------------------------------------------------

    def move_to(self, pose: Pose, keep_orientation: bool = False) -> None:
        self._plan_and_execute(
            self._pose_constraints(pose),
            "规划",
            pose if keep_orientation else None,
        )

    def _plan_and_execute(
        self,
        goal_constraints: Constraints,
        action: str,
        pose: Pose | None = None,
    ) -> None:
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name = ARM_GROUP
        plan.start_state = RobotState(is_diff=True)
        plan.num_planning_attempts = PLANNING_ATTEMPTS
        plan.allowed_planning_time = self.planning_time
        plan.max_velocity_scaling_factor = self.velocity_scaling
        plan.max_acceleration_scaling_factor = self.acceleration_scaling
        plan.goal_constraints.append(goal_constraints)
        if pose is not None:
            plan.path_constraints = self._orientation_constraint(pose)

        response = self._call(
            self.plan_client, request, self.planning_time + 5.0
        ).motion_plan_response
        self._check_moveit(response.error_code.val, action)
        self._execute(response.trajectory)

    def move_straight(self, pose: Pose) -> None:
        request = GetCartesianPath.Request()
        request.header.frame_id = BASE_FRAME
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state = RobotState(is_diff=True)
        request.group_name = ARM_GROUP
        request.link_name = TCP_LINK
        request.waypoints = [pose]
        request.max_step = self.cartesian_max_step
        request.jump_threshold = 0.0
        request.avoid_collisions = True

        response = self._call(
            self.cartesian_client, request, self.planning_time + 5.0
        )
        if response.fraction < MIN_CARTESIAN_FRACTION:
            raise RuntimeError(
                f"直线轨迹不完整: {response.fraction:.1%}，"
                f"要求至少 {MIN_CARTESIAN_FRACTION:.1%}"
            )
        self._execute(response.solution)

    def move_home(self) -> None:
        constraints = Constraints()
        for name, position in zip(HOME_JOINT_NAMES, self.home_joint_positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = 0.01
            joint.tolerance_below = 0.01
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)

        self._plan_and_execute(constraints, "回 Home 规划")

    def _pose_constraints(self, pose: Pose) -> Constraints:
        constraints = Constraints()
        size = 2 * self.position_tolerance

        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[size, size, size])
        volume = BoundingVolume(primitives=[box], primitive_poses=[pose])

        position = PositionConstraint()
        position.header.frame_id = BASE_FRAME
        position.link_name = TCP_LINK
        position.constraint_region = volume
        position.weight = 1.0

        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(self._orientation(pose))
        return constraints

    def _orientation_constraint(self, pose: Pose) -> Constraints:
        constraints = Constraints()
        constraints.orientation_constraints.append(self._orientation(pose))
        return constraints

    def _orientation(self, pose: Pose) -> OrientationConstraint:
        orientation = OrientationConstraint()
        orientation.header.frame_id = BASE_FRAME
        orientation.link_name = TCP_LINK
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation.weight = 1.0
        return orientation

    def _execute(self, trajectory) -> None:
        if self.preview_only:
            self.get_logger().info("preview_only=true：轨迹已规划，未执行")
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        handle = self._wait(self.execute_client.send_goal_async(goal), GOAL_TIMEOUT)
        if handle is None or not handle.accepted:
            raise RuntimeError("机械臂拒绝执行轨迹")
        result = self._wait(
            handle.get_result_async(), self._trajectory_time(trajectory) + GOAL_TIMEOUT
        )
        self._check_moveit(result.result.error_code.val, "执行")

    # ------------------------------------------------------------------
    # 夹爪：仿真是关节位置控制；真机抓取时会等待接触检测结果。
    # ------------------------------------------------------------------

    def open_gripper(self) -> None:
        if self.preview_only:
            self.get_logger().info("preview_only=true：夹爪未执行")
            return
        if self.sim:
            self._set_sim_gripper(self.gripper_open_position)
            return

        request = GripperCommand.Request()
        request.position = self.hardware_open_position
        request.timeout = GRIPPER_TIMEOUT
        response = self._call(
            self.open_client, request, GRIPPER_TIMEOUT + 2.0
        )
        if not response.success:
            raise RuntimeError(f"夹爪命令失败: {response.message}")

    def grasp(self) -> None:
        if self.preview_only:
            self.get_logger().info("preview_only=true：夹爪未执行")
            return
        if self.sim:
            self._set_sim_gripper(self.gripper_close_position)
            return

        goal = GripperGrasp.Goal()
        goal.closing_torque = self.closing_torque
        goal.hold_torque = self.hold_torque
        goal.timeout = GRIPPER_TIMEOUT
        handle = self._wait(self.grasp_client.send_goal_async(goal), GOAL_TIMEOUT)
        if handle is None or not handle.accepted:
            raise RuntimeError("夹爪拒绝抓取请求")
        result = self._wait(handle.get_result_async(), GRIPPER_TIMEOUT + 2.0)
        if result is None:
            raise TimeoutError("夹爪抓取超时")
        if not result.result.object_detected:
            self.get_logger().warn(f"未检测到物料: {result.result.message}")

    def _set_sim_gripper(self, position: float) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = ["gripper_joint"]
        trajectory.points = [
            JointTrajectoryPoint(
                positions=[position],
                time_from_start=duration(SIM_GRIPPER_DURATION),
            )
        ]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        handle = self._wait(
            self.sim_gripper_client.send_goal_async(goal), GOAL_TIMEOUT
        )
        if handle is None or not handle.accepted:
            raise RuntimeError("仿真夹爪拒绝轨迹")
        result = self._wait(
            handle.get_result_async(), GRIPPER_TIMEOUT + SIM_GRIPPER_DURATION
        )
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"仿真夹爪失败: {result.result.error_string}")

    # ------------------------------------------------------------------
    # ROS 等待与错误处理：所有动作都经过这些小函数，避免静默失败。
    # ------------------------------------------------------------------

    def _wait_for_interfaces(self) -> None:
        for client, name in (
            (self.plan_client, PLAN_SERVICE),
            (self.cartesian_client, CARTESIAN_SERVICE),
        ):
            if not client.wait_for_service(timeout_sec=INTERFACE_TIMEOUT):
                raise TimeoutError(f"服务未上线: {name}")

        if not self.preview_only and not self.execute_client.wait_for_server(
            timeout_sec=INTERFACE_TIMEOUT
        ):
            raise TimeoutError(f"Action 未上线: {EXECUTE_ACTION}")

        if self.preview_only:
            return
        if self.sim:
            if not self.sim_gripper_client.wait_for_server(
                timeout_sec=INTERFACE_TIMEOUT
            ):
                raise TimeoutError(f"Action 未上线: {SIM_GRIPPER_ACTION}")
            return

        if not self.grasp_client.wait_for_server(timeout_sec=INTERFACE_TIMEOUT):
            raise TimeoutError(f"Action 未上线: {HARDWARE_GRASP_ACTION}")
        if not self.open_client.wait_for_service(timeout_sec=INTERFACE_TIMEOUT):
            raise TimeoutError(f"服务未上线: {HARDWARE_OPEN_SERVICE}")

    def _call(self, client, request, timeout: float):
        result = self._wait(client.call_async(request), timeout)
        if result is None:
            raise TimeoutError(f"服务超时: {client.srv_name}")
        return result

    def _wait(self, future, timeout: float):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.05)
        return future.result()

    @staticmethod
    def _trajectory_time(trajectory) -> float:
        points = trajectory.joint_trajectory.points
        if not points:
            return 5.0
        value = points[-1].time_from_start
        return value.sec + value.nanosec * 1e-9

    @staticmethod
    def _check_moveit(code: int, action: str) -> None:
        if int(code) != int(MoveItErrorCodes.SUCCESS):
            raise RuntimeError(f"{action}失败，MoveIt 错误码: {int(code)}")


def main() -> None:
    rclpy.init()
    node = CarryNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
