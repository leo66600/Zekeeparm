"""Fail-closed validation for robot-affecting visual grasp configuration."""

from __future__ import annotations

import math
from typing import Any


def _finite(mapping: dict[str, Any], key: str, default: float) -> float:
    value = float(mapping.get(key, default))
    if not math.isfinite(value):
        raise ValueError(f"{key} must be finite")
    return value


def _nonnegative(mapping: dict[str, Any], key: str, default: float) -> float:
    value = _finite(mapping, key, default)
    if value < 0.0:
        raise ValueError(f"{key} must be nonnegative")
    return value


def _positive(mapping: dict[str, Any], key: str, default: float) -> float:
    value = _finite(mapping, key, default)
    if value <= 0.0:
        raise ValueError(f"{key} must be positive")
    return value


def _positive_int(mapping: dict[str, Any], key: str, default: int) -> int:
    raw = mapping.get(key, default)
    if isinstance(raw, bool):
        raise ValueError(f"{key} must be a positive integer")
    value = int(raw)
    if value <= 0 or float(raw) != float(value):
        raise ValueError(f"{key} must be a positive integer")
    return value


def _nonnegative_int(mapping: dict[str, Any], key: str, default: int) -> int:
    raw = mapping.get(key, default)
    if isinstance(raw, bool):
        raise ValueError(f"{key} must be a nonnegative integer")
    value = int(raw)
    if value < 0 or float(raw) != float(value):
        raise ValueError(f"{key} must be a nonnegative integer")
    return value


def _finite_sequence(mapping: dict[str, Any], key: str, default) -> tuple[float, ...]:
    raw = mapping.get(key, default)
    if isinstance(raw, (str, bytes)):
        raise ValueError(f"{key} must contain finite numbers")
    try:
        values = tuple(float(value) for value in raw)
    except TypeError:
        values = (float(raw),)
    if not values or any(not math.isfinite(value) for value in values):
        raise ValueError(f"{key} must contain finite numbers")
    return values


def runtime_motion_values_are_finite(values: dict[str, Any]) -> bool:
    for value in values.values():
        if isinstance(value, (str, bytes)):
            return False
        try:
            candidates = tuple(float(item) for item in value)
        except TypeError:
            candidates = (float(value),)
        if not candidates or any(not math.isfinite(item) for item in candidates):
            return False
    return True


def _unit_interval(mapping: dict[str, Any], key: str, default: float) -> float:
    value = _finite(mapping, key, default)
    if not 0.0 < value <= 1.0:
        raise ValueError(f"{key} must be in (0, 1]")
    return value


def validate_grasp_runtime_config(config: dict[str, Any]) -> None:
    if not isinstance(config, dict):
        raise ValueError("grasp configuration must be a mapping")
    robot = config.get("robot", {}) or {}
    ready = robot.get("ready_pose", {}) or {}
    moveit = robot.get("moveit", {}) or {}
    pipeline = config.get("grasp_pipeline", {}) or {}
    grasp = pipeline.get("grasp", {}) or {}
    ik = pipeline.get("ik", {}) or {}
    shape_yaw = pipeline.get("shape_yaw", {}) or {}
    cube = pipeline.get("cube", {}) or {}
    safety = config.get("safety", {}) or {}
    graspnet = config.get("graspnet", {}) or {}
    official_sdk = config.get("official_sdk", {}) or {}
    if not all(
        isinstance(value, dict)
        for value in (
            robot,
            ready,
            moveit,
            pipeline,
            grasp,
            ik,
            shape_yaw,
            cube,
            safety,
            graspnet,
            official_sdk,
        )
    ):
        raise ValueError(
            "robot, ready_pose, moveit, grasp_pipeline, grasp, "
            "ik, shape_yaw, cube, safety, graspnet, and official_sdk "
            "must be mappings"
        )

    _positive(robot, "ros_timeout_s", 8.0)
    _positive(robot, "joint_state_max_age_s", 0.5)
    _nonnegative(robot, "rgbd_joint_sync_tolerance_s", 0.10)
    _positive(robot, "gripper_max_width_m", 0.06517241379310346)
    planning_pipeline_id = str(
        moveit.get("planning_pipeline_id", "ompl")
    ).strip()
    planner_id = str(moveit.get("planner_id", "RRTConnect")).strip()
    if not planning_pipeline_id:
        raise ValueError("planning_pipeline_id must be nonempty")
    if not planner_id:
        raise ValueError("planner_id must be nonempty")
    _positive_int(moveit, "planning_attempts", 5)
    _positive(moveit, "planning_time_s", 5.0)
    _positive(moveit, "joint_goal_tolerance_rad", 0.005)
    _nonnegative(moveit, "pregrasp_collision_fallback_raise_m", 0.030)
    _unit_interval(moveit, "max_velocity_scaling_factor", 0.15)
    _unit_interval(moveit, "max_acceleration_scaling_factor", 0.10)
    _positive(moveit, "cartesian_max_step_m", 0.005)
    _unit_interval(moveit, "cartesian_fallback_min_fraction", 0.90)
    _positive(moveit, "revolute_jump_threshold_rad", 0.25)
    _positive(moveit, "max_joint_velocity_rad_s", 0.25)
    _positive(moveit, "execution_timeout_s", 30.0)
    worktable_width = _positive(moveit, "worktable_width_m", 0.9)
    _positive(moveit, "worktable_length_m", 1.2)
    worktable_mount_inset = _nonnegative(
        moveit,
        "worktable_mount_inset_m",
        0.062,
    )
    if worktable_mount_inset >= worktable_width / 2.0:
        raise ValueError(
            "worktable_mount_inset_m must be less than half "
            "worktable_width_m"
        )
    _positive(moveit, "worktable_thickness_m", 0.02)
    for key, default in (
        ("x", 0.25),
        ("y", 0.0),
        ("z", 0.35),
        ("roll", 0.0),
        ("pitch", 1.2),
        ("yaw", 0.0),
    ):
        _finite(ready, key, default)
    _positive(ready, "duration", 3.0)
    if "ready_joints" in official_sdk:
        ready_joints = _finite_sequence(official_sdk, "ready_joints", ())
        if len(ready_joints) != 6:
            raise ValueError("ready_joints must contain six values")

    _nonnegative(grasp, "pregrasp_offset_m", 0.08)
    pregrasp_offsets = _finite_sequence(
        grasp,
        "pregrasp_offset_candidates_m",
        (grasp.get("pregrasp_offset_m", 0.08),),
    )
    if any(value < 0.0 for value in pregrasp_offsets):
        raise ValueError("pregrasp_offset_candidates_m must be nonnegative")
    transit_clearance = _nonnegative(grasp, "transit_clearance_m", 0.06)
    orientation_clearance = _nonnegative(
        grasp,
        "orientation_clearance_m",
        0.03,
    )
    _positive(grasp, "orientation_clearance_step_m", 0.01)
    if orientation_clearance > transit_clearance:
        raise ValueError(
            "orientation_clearance_m must not exceed transit_clearance_m"
        )
    _nonnegative(grasp, "insertion_depth_m", 0.0)
    _nonnegative(grasp, "grasp_center_xy_tolerance_m", 0.005)
    _nonnegative(grasp, "grasp_center_height_tolerance_m", 0.005)
    _finite(grasp, "grasp_center_z_offset_m", 0.0)
    allow_shorter_descent_fallback = grasp.get(
        "allow_shorter_descent_fallback", False
    )
    if not isinstance(allow_shorter_descent_fallback, bool):
        raise ValueError("allow_shorter_descent_fallback must be boolean")
    _finite_sequence(grasp, "local_yaw_search_deg", (0.0,))
    _positive(grasp, "oblique_first_radius_m", 0.43)
    far_pregrasp_offsets = _finite_sequence(
        grasp,
        "far_pregrasp_offset_candidates_m",
        (0.030, 0.012),
    )
    if any(value < 0.0 for value in far_pregrasp_offsets):
        raise ValueError("far_pregrasp_offset_candidates_m must be nonnegative")
    _positive_int(grasp, "far_candidate_attempt_limit", 8)
    _finite(grasp, "min_base_z_m", 0.03)
    planning_budget = _positive(pipeline, "planning_budget_s", 10.0)
    candidate_search_budget = _positive(
        pipeline,
        "candidate_search_budget_s",
        5.0,
    )
    local_yaw_reserve = _positive(
        pipeline,
        "local_yaw_reserve_s",
        2.0,
    )
    final_planning_reserve = _positive(
        pipeline,
        "final_planning_reserve_s",
        5.0,
    )
    if candidate_search_budget > (
        planning_budget - final_planning_reserve - local_yaw_reserve
    ):
        raise ValueError(
            "candidate_search_budget_s must fit before "
            "local_yaw_reserve_s and final_planning_reserve_s"
        )
    _positive_int(pipeline, "maximum_planning_candidates", 8)
    _positive_int(ik, "max_iterations", 600)
    _nonnegative_int(ik, "multistart_retries", 8)
    _positive_int(ik, "candidate_max_iterations", 250)
    _nonnegative_int(ik, "candidate_multistart_retries", 2)
    _positive(ik, "timeout_s", 0.25)
    _positive_int(shape_yaw, "minimum_points", 200)
    minimum_anisotropy = _finite(
        shape_yaw,
        "minimum_anisotropy",
        1.35,
    )
    if minimum_anisotropy < 1.0:
        raise ValueError("minimum_anisotropy must be at least one")
    _nonnegative(shape_yaw, "minimum_major_span_m", 0.012)
    _nonnegative(shape_yaw, "minimum_minor_span_m", 0.004)
    frame_count = _positive_int(cube, "depth_frame_count", 5)
    minimum_votes = _positive_int(cube, "depth_minimum_votes", 3)
    if minimum_votes > frame_count:
        raise ValueError(
            "depth_minimum_votes must not exceed depth_frame_count"
        )
    if bool(cube.get("enforce_size_bounds", False)):
        nominal_size = _positive(cube, "nominal_size_m", 0.030)
        for key, default in (
            ("relaxed_size_bounds_m", (0.022, 0.038)),
            ("strict_size_bounds_m", (0.025, 0.035)),
        ):
            bounds = _finite_sequence(cube, key, default)
            if (
                len(bounds) != 2
                or bounds[0] <= 0.0
                or bounds[0] > nominal_size
                or bounds[1] < nominal_size
                or bounds[0] >= bounds[1]
            ):
                raise ValueError(
                    f"{key} must be increasing and contain nominal_size_m"
                )
    tcp_to_grasp_center = _finite_sequence(
        grasp,
        "tcp_to_grasp_center_m",
        (0.0, 0.0, 0.0),
    )
    if len(tcp_to_grasp_center) != 3:
        raise ValueError("tcp_to_grasp_center_m must contain three values")
    depth_corrections = _finite_sequence(
        grasp,
        "depth_correction_candidates_m",
        (0.0,),
    )
    _positive_int(grasp, "depth_correction_max_variants", 2)
    if any(abs(value) > 0.05 for value in depth_corrections):
        raise ValueError("depth corrections must remain within 50 mm")
    insertion_values = _finite_sequence(
        cube,
        "insertion_depth_candidates_m",
        (0.008, 0.012, 0.016),
    )
    if any(value < 0.0 for value in insertion_values):
        raise ValueError(
            "insertion_depth_candidates_m values must be nonnegative"
        )
    _positive(graspnet, "nms_translation_thresh_m", 0.005)
    _nonnegative(graspnet, "grasp_width_tolerance_m", 0.0)
    _nonnegative(graspnet, "target_min_edge_distance_px", 6.0)
    center_weight = _nonnegative(
        graspnet,
        "target_center_score_weight",
        0.20,
    )
    if center_weight > 1.0:
        raise ValueError("target_center_score_weight must not exceed 1")
    _nonnegative_int(graspnet, "target_depth_window_radius_px", 4)
    _unit_interval(graspnet, "target_min_mask_support", 0.80)
    _unit_interval(
        graspnet,
        "target_min_valid_depth_fraction",
        0.80,
    )
    _nonnegative(
        graspnet,
        "target_max_local_depth_deviation_m",
        0.005,
    )
    _nonnegative(
        graspnet,
        "target_max_center_depth_error_m",
        0.015,
    )
    nms_rotation_thresh_deg = _positive(
        graspnet,
        "nms_rotation_thresh_deg",
        10.0,
    )
    if nms_rotation_thresh_deg > 180.0:
        raise ValueError("nms_rotation_thresh_deg must not exceed 180")
    if _positive_int(safety, "capture_stability_samples", 8) < 2:
        raise ValueError("capture_stability_samples must be at least 2")
    _nonnegative(safety, "capture_sample_interval_s", 0.04)
    _nonnegative(safety, "capture_max_position_span_rad", 0.005)
    _nonnegative(safety, "capture_max_velocity_rad_s", 0.02)
    _nonnegative(safety, "motion_position_tolerance_rad", 0.005)
    _nonnegative(safety, "motion_velocity_tolerance_rad_s", 0.03)
    _positive_int(safety, "motion_settle_samples", 5)
    _nonnegative(safety, "motion_sample_interval_s", 0.05)
    _positive(safety, "motion_settle_timeout_s", 3.0)

    table_z = safety.get("table_z_m")
    if table_z is not None and not math.isfinite(float(table_z)):
        raise ValueError("table_z_m must be null or finite")
    _positive(safety, "table_bin_size_m", 0.005)
    _nonnegative(safety, "table_inlier_tolerance_m", 0.008)
    _positive_int(safety, "table_min_inliers", 200)
    _nonnegative(safety, "table_max_drift_m", 0.015)
    _nonnegative(safety, "table_clearance_m", 0.012)
    _nonnegative(safety, "candidate_table_penetration_tolerance_m", 0.005)
    _nonnegative(safety, "environment_clearance_m", 0.005)
    _nonnegative(safety, "gripper_self_filter_distance_m", 0.015)
    _positive(safety, "environment_voxel_size_m", 0.005)
    _nonnegative(safety, "target_depth_tolerance_m", 0.04)
    _nonnegative_int(safety, "target_mask_dilation_px", 0)
    _positive(safety, "sweep_linear_step_m", 0.01)
    _positive(safety, "sweep_angular_step_rad", 0.10)
