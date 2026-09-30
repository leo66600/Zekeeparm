"""Perception capture and motion gates for direct SDK execution."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
from typing import Any

import numpy as np

from .cube_perception import fuse_depth_frames
from .ordinary_grasp import estimate_grasp
from .transforms import transform_grasp_pose_to_base_with_retreat


class MotionGateError(RuntimeError):
    """A planned official-route motion is unsafe or incomplete."""


class ExecutionResult(str, Enum):
    HOLDING = "holding"
    EMPTY = "empty"


@dataclass(frozen=True)
class OfficialMotionPlan:
    poses: tuple[np.ndarray, np.ndarray, np.ndarray]
    joint_targets: tuple[np.ndarray, np.ndarray, np.ndarray]
    jaw_width_m: float
    open_width_m: float = 0.070


def execute_motion_plan(
    robot: Any,
    plan: OfficialMotionPlan,
    *,
    segment_duration_s: float,
) -> ExecutionResult:
    """Execute outward path once, then reverse it; never auto-retry faults."""
    pregrasp, grasp, retreat = plan.poses
    q_pregrasp, q_grasp, q_retreat = plan.joint_targets
    robot.open_gripper(plan.open_width_m)
    robot.move_to_traj_and_wait(
        pregrasp, q_pregrasp, segment_duration_s
    )
    robot.move_to_traj_and_wait(grasp, q_grasp, segment_duration_s)
    held = bool(robot.grasp())
    robot.move_to_traj_and_wait(
        retreat, q_retreat, segment_duration_s
    )
    robot.move_ready()
    if held:
        return ExecutionResult.HOLDING
    robot.open_gripper(plan.jaw_width_m)
    return ExecutionResult.EMPTY


def execute_place_plan(
    robot: Any,
    plan: OfficialMotionPlan,
    *,
    segment_duration_s: float,
) -> None:
    preplace, release, retreat = plan.poses
    q_preplace, q_release, q_retreat = plan.joint_targets
    robot.move_to_traj_and_wait(preplace, q_preplace, segment_duration_s)
    robot.move_to_traj_and_wait(release, q_release, segment_duration_s)
    robot.release()
    robot.move_to_traj_and_wait(retreat, q_retreat, segment_duration_s)
    robot.move_ready()


def build_motion_plan(
    result: Any,
    detection_index: int,
    depth_mm: np.ndarray,
    camera_matrix: np.ndarray,
    camera_to_base: np.ndarray,
    config: dict,
    kinematics: Any,
    start_joints: np.ndarray,
) -> OfficialMotionPlan:
    grasp_cfg = (config.get("grasp_pipeline") or {}).get("grasp") or {}
    grasp = estimate_grasp(
        result,
        int(detection_index),
        depth_mm,
        camera_matrix,
        depth_quantile=float(grasp_cfg.get("depth_quantile", 0.5)),
    )
    if not grasp.is_valid or grasp.position is None or grasp.tcp_rotation is None:
        raise MotionGateError(grasp.rejected_reason or "target grasp pose is invalid")
    offset = float(grasp_cfg.get("pregrasp_offset_m", 0.05))
    grasp_pose, pregrasp_pose, _ = transform_grasp_pose_to_base_with_retreat(
        grasp.position,
        grasp.tcp_rotation,
        camera_to_base,
        offset,
        offset,
        float(grasp_cfg.get("insertion_depth_m", 0.0)),
    )
    poses = tuple(
        np.asarray(pose, dtype=np.float64)
        for pose in (pregrasp_pose, grasp_pose, pregrasp_pose)
    )
    joint_targets = tuple(
        kinematics.solve_pose_sequence(poses, start_joints)
    )
    official_cfg = config.get("official_sdk") or {}
    robot_cfg = config.get("robot") or {}
    validate_execution_targets(
        poses,
        joint_targets,
        jaw_width_m=float(grasp.jaw_width_m),
        joint_limits=kinematics.joint_limits,
        workspace=official_cfg.get("workspace") or {
            "x": [0.10, 0.55],
            "y": [-0.35, 0.35],
            "z": [0.03, 0.60],
        },
        min_tcp_z_m=float(official_cfg.get("min_tcp_z_m", 0.03)),
        max_segment_joint_delta_rad=float(
            official_cfg.get("max_segment_joint_delta_rad", 1.0)
        ),
        gripper_max_width_m=float(robot_cfg.get("gripper_max_width_m", 0.07)),
        start_joints=start_joints,
    )
    return OfficialMotionPlan(
        poses,
        joint_targets,
        float(grasp.jaw_width_m),
        float(robot_cfg.get("gripper_max_width_m", 0.07)),
    )


def build_place_motion_plan(
    container_mask: np.ndarray,
    depth_mm: np.ndarray,
    camera_matrix: np.ndarray,
    camera_to_base: np.ndarray,
    config: dict,
    kinematics: Any,
    start_joints: np.ndarray,
    orientation_rpy: tuple[float, float, float],
    jaw_width_m: float,
) -> OfficialMotionPlan:
    place_cfg = config.get("place") or {}
    release = container_release_pose(
        container_mask,
        depth_mm,
        camera_matrix,
        camera_to_base,
        orientation_rpy=orientation_rpy,
        clearance_m=float(place_cfg.get("release_clearance_m", 0.03)),
    )
    release[2] -= float(place_cfg.get("insertion_depth_m", 0.0))
    preplace = release.copy()
    preplace[2] += float(place_cfg.get("approach_clearance_m", 0.05))
    poses = (preplace, release, preplace.copy())
    joint_targets = tuple(
        kinematics.solve_pose_sequence(poses, start_joints)
    )
    official_cfg = config.get("official_sdk") or {}
    robot_cfg = config.get("robot") or {}
    validate_execution_targets(
        poses,
        joint_targets,
        jaw_width_m=jaw_width_m,
        joint_limits=kinematics.joint_limits,
        workspace=official_cfg["workspace"],
        min_tcp_z_m=float(official_cfg.get("min_tcp_z_m", 0.03)),
        max_segment_joint_delta_rad=float(
            official_cfg.get("max_segment_joint_delta_rad", 1.0)
        ),
        gripper_max_width_m=float(robot_cfg.get("gripper_max_width_m", 0.07)),
        start_joints=start_joints,
    )
    return OfficialMotionPlan(poses, joint_targets, jaw_width_m)


def capture_fused_depth(
    camera: Any,
    target_mask: np.ndarray,
    *,
    frame_count: int = 5,
    minimum_votes: int = 3,
) -> np.ndarray:
    mask = np.asarray(target_mask) > 0
    if mask.ndim != 2 or not np.any(mask):
        raise MotionGateError("target mask is empty")
    if mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any():
        raise MotionGateError("target mask touches image edge")
    depths = []
    for _ in range(int(frame_count)):
        _color, depth = camera.get_frame()
        if depth is not None:
            array = np.asarray(depth)
            if array.shape != mask.shape:
                raise MotionGateError("depth frame does not match target mask")
            depths.append(array)
    if len(depths) != int(frame_count):
        raise MotionGateError(
            f"depth capture incomplete: got {len(depths)}/{int(frame_count)} frames"
        )
    fused = fuse_depth_frames(depths, minimum_votes=int(minimum_votes))
    if not np.any((fused > 0) & mask):
        raise MotionGateError("target has no fused depth with required votes")
    return fused


def container_release_pose(
    container_mask: np.ndarray,
    depth_mm: np.ndarray,
    camera_matrix: np.ndarray,
    camera_to_base: np.ndarray,
    *,
    orientation_rpy: tuple[float, float, float],
    clearance_m: float,
) -> np.ndarray:
    mask = np.asarray(container_mask) > 0
    depth = np.asarray(depth_mm, dtype=np.float64)
    valid = mask & np.isfinite(depth) & (depth > 0)
    if mask.shape != depth.shape or int(valid.sum()) < 3:
        raise MotionGateError("container mask/depth is incomplete")
    rows, cols = np.nonzero(valid)
    # 深度图以毫米为单位保存。这里先转换为米，保证反投影得到的三维坐标
    # 与机器人位姿、运动规划使用同一套长度单位。
    z = depth[valid] / 1000.0
    intrinsic = np.asarray(camera_matrix, dtype=np.float64)
    # 将有效像素反投影到相机坐标系：
    #   Z = depth_m
    #   X = (u - cx) * Z / fx
    #   Y = (v - cy) * Z / fy
    # 其中 u/cols 表示图像横向像素坐标，v/rows 表示图像纵向像素坐标。
    # 这正是 RGB-D 实训中手动验证的关系：
    # 相机内参 + 深度值 + 像素坐标 -> 相机坐标系下的三维点。
    x = (cols - intrinsic[0, 2]) * z / intrinsic[0, 0]
    y = (rows - intrinsic[1, 2]) * z / intrinsic[1, 1]
    points_cam = np.column_stack([x, y, z, np.ones_like(z)])
    # 机器人运动规划与执行命令使用 base 坐标系，因此在计算释放位姿前，
    # 需要先把相机坐标系下的点转换到机器人基坐标系。
    points_base = (
        np.asarray(camera_to_base, dtype=np.float64) @ points_cam.T
    ).T[:, :3]
    center_xy = np.median(points_base[:, :2], axis=0)
    rim_z = float(np.quantile(points_base[:, 2], 0.95))
    return np.array(
        [center_xy[0], center_xy[1], rim_z + float(clearance_m), *orientation_rpy],
        dtype=np.float64,
    )


def validate_execution_targets(
    poses: Sequence[np.ndarray],
    joint_targets: Sequence[np.ndarray],
    *,
    jaw_width_m: float,
    joint_limits: np.ndarray,
    workspace: dict,
    min_tcp_z_m: float,
    max_segment_joint_delta_rad: float,
    gripper_max_width_m: float = 0.070,
    start_joints: np.ndarray | None = None,
) -> None:
    if not np.isfinite(jaw_width_m):
        raise MotionGateError("grasp width must be finite")
    if jaw_width_m < 0.0:
        raise MotionGateError("grasp width must be non-negative")
    if jaw_width_m > gripper_max_width_m:
        raise MotionGateError(
            f"grasp width {jaw_width_m * 1000.0:.1f}mm exceeds "
            f"{gripper_max_width_m * 1000.0:.1f}mm"
        )
    if len(poses) != 3 or len(joint_targets) != 3:
        raise MotionGateError("pregrasp, grasp, and retreat targets are required")
    limits = np.asarray(joint_limits, dtype=np.float64)
    if limits.shape != (6, 2):
        raise MotionGateError("six joint limit pairs are required")
    for label, pose, joints in zip(
        ("pregrasp", "grasp", "retreat"), poses, joint_targets
    ):
        target = np.asarray(pose, dtype=np.float64).reshape(-1)
        q = np.asarray(joints, dtype=np.float64).reshape(-1)
        if target.size != 6 or q.size != 6 or not np.all(np.isfinite(target)) or not np.all(np.isfinite(q)):
            raise MotionGateError(f"{label} contains non-finite or incomplete values")
        for axis, value in zip(("x", "y", "z"), target[:3]):
            low, high = [float(item) for item in workspace[axis]]
            if value < low or value > high:
                raise MotionGateError(f"{label} {axis}={value:.4f} outside workspace")
        if target[2] < float(min_tcp_z_m):
            raise MotionGateError(f"{label} TCP below minimum height")
        if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
            raise MotionGateError(f"{label} joint target outside limits")
    segment_targets = list(joint_targets)
    if start_joints is not None:
        segment_targets.insert(0, np.asarray(start_joints, dtype=np.float64))
    for start, end in zip(segment_targets, segment_targets[1:]):
        delta = float(np.max(np.abs(np.asarray(end) - np.asarray(start))))
        if delta > float(max_segment_joint_delta_rad):
            raise MotionGateError(
                f"joint segment delta {delta:.4f}rad exceeds limit"
            )
