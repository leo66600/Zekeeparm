"""Pure safety checks shared by the visual grasp execution path."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Callable, Iterable, Optional, Sequence

import numpy as np


class RobotUnstableError(RuntimeError):
    """Raised when an eye-in-hand image cannot be paired with a stable pose."""


@dataclass(frozen=True)
class JointStabilityReport:
    stable: bool
    worst_joint: int
    max_position_span_rad: float
    max_velocity_rad_s: float


@dataclass(frozen=True)
class StableRgbdCapture:
    color_bgr: np.ndarray
    depth_mm: np.ndarray
    q_at_capture: np.ndarray
    stability: JointStabilityReport
    capture_monotonic: float


@dataclass(frozen=True)
class MotionSettleReport:
    settled: bool
    max_position_error_rad: float
    max_velocity_rad_s: float
    consecutive_samples: int


@dataclass(frozen=True)
class PoseSequenceResult:
    success: bool
    solutions: tuple[Any, ...]
    failed_index: Optional[int]
    error: float


@dataclass(frozen=True)
class OverheadGraspPath:
    poses: tuple[tuple[float, ...], ...]
    waypoint_names: tuple[str, ...]
    segment_names: tuple[str, ...]
    transit_z_m: float
    orientation_z_m: float
    depth_correction_m: float = 0.0
    network_grasp_depth_m: float = 0.0


def descending_clearance_candidates(
    maximum_clearance_m: float,
    step_m: float,
) -> tuple[float, ...]:
    """Return deterministic clearance candidates from the maximum down to zero."""

    maximum = float(maximum_clearance_m)
    step = float(step_m)
    if not np.isfinite(maximum) or maximum < 0.0:
        raise ValueError("maximum_clearance_m must be finite and nonnegative")
    if not np.isfinite(step) or step <= 0.0:
        raise ValueError("step_m must be finite and positive")

    candidates: list[float] = []
    index = 0
    while True:
        candidate = maximum - index * step
        if candidate <= 1e-12:
            break
        candidates.append(round(candidate, 12))
        index += 1
    candidates.append(0.0)
    return tuple(candidates)


def build_overhead_grasp_path(
    ready_pose: Sequence[float],
    pregrasp_pose: Sequence[float],
    grasp_pose: Sequence[float],
    retreat_pose: Sequence[float],
    *,
    transit_clearance_m: float,
    orientation_clearance_m: float,
) -> OverheadGraspPath:
    """Build a lift-transfer-descend grasp path and its reverse return."""

    named_input = (
        ("ready", ready_pose),
        ("pregrasp", pregrasp_pose),
        ("grasp", grasp_pose),
        ("retreat", retreat_pose),
    )
    normalized: dict[str, tuple[float, ...]] = {}
    for name, pose in named_input:
        values = tuple(float(value) for value in pose)
        if len(values) != 6 or not np.all(np.isfinite(values)):
            raise ValueError(f"{name} pose must contain six finite values")
        normalized[name] = values

    clearance = float(transit_clearance_m)
    if not np.isfinite(clearance) or clearance < 0.0:
        raise ValueError("transit_clearance_m must be finite and nonnegative")
    orientation_clearance = float(orientation_clearance_m)
    if (
        not np.isfinite(orientation_clearance)
        or orientation_clearance < 0.0
    ):
        raise ValueError(
            "orientation_clearance_m must be finite and nonnegative"
        )

    ready = normalized["ready"]
    pregrasp = normalized["pregrasp"]
    grasp = normalized["grasp"]
    retreat = normalized["retreat"]
    transit_z = max(pose[2] for pose in normalized.values()) + clearance
    orientation_z = (
        max(pregrasp[2], grasp[2], retreat[2]) + orientation_clearance
    )
    if orientation_z > transit_z + 1e-12:
        raise ValueError("orientation height must not exceed transit height")
    ready_lift = (
        ready[0],
        ready[1],
        transit_z,
        ready[3],
        ready[4],
        ready[5],
    )
    target_transit = (
        pregrasp[0],
        pregrasp[1],
        transit_z,
        ready[3],
        ready[4],
        ready[5],
    )
    target_oriented = (
        pregrasp[0],
        pregrasp[1],
        orientation_z,
        pregrasp[3],
        pregrasp[4],
        pregrasp[5],
    )
    target_orientation_entry = (
        pregrasp[0],
        pregrasp[1],
        orientation_z,
        ready[3],
        ready[4],
        ready[5],
    )
    waypoint_names = (
        "ready",
        "ready_lift",
        "target_transit",
        "target_orientation_entry",
        "target_oriented",
        "pregrasp",
        "grasp",
        "retreat",
        "target_oriented_return",
        "target_orientation_exit",
        "target_transit_return",
        "ready_lift_return",
        "ready_return",
    )
    poses = (
        ready,
        ready_lift,
        target_transit,
        target_orientation_entry,
        target_oriented,
        pregrasp,
        grasp,
        retreat,
        target_oriented,
        target_orientation_entry,
        target_transit,
        ready_lift,
        ready,
    )
    segment_names = tuple(
        f"{start}_to_{end}"
        for start, end in zip(waypoint_names, waypoint_names[1:])
    )
    return OverheadGraspPath(
        poses=poses,
        waypoint_names=waypoint_names,
        segment_names=segment_names,
        transit_z_m=transit_z,
        orientation_z_m=orientation_z,
    )


def resolve_effective_min_tcp_z(
    *,
    configured_min_base_z_m: float,
    table_z_m: float,
    table_penetration_tolerance_m: float = 0.0,
) -> float:
    """Return the stricter configured or MoveIt-table TCP floor."""

    configured = float(configured_min_base_z_m)
    table = float(table_z_m)
    tolerance = float(table_penetration_tolerance_m)
    if (
        not np.isfinite(configured)
        or not np.isfinite(table)
        or not np.isfinite(tolerance)
        or tolerance < 0.0
    ):
        raise ValueError("TCP and table Z limits must be finite")
    return max(configured, table - tolerance)


def _as_joint_matrix(
    samples: Sequence[np.ndarray],
    name: str,
) -> np.ndarray:
    if not samples:
        raise ValueError(f"{name} requires at least one sample")
    matrix = np.asarray(samples, dtype=np.float64)
    if matrix.ndim != 2 or matrix.shape[1] == 0:
        raise ValueError(f"{name} samples must have shape (N, joints)")
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} samples contain non-finite values")
    return matrix


def assess_joint_stability(
    positions: Sequence[np.ndarray],
    velocities: Sequence[np.ndarray],
    *,
    max_position_span_rad: float,
    max_velocity_rad_s: float,
) -> JointStabilityReport:
    q = _as_joint_matrix(positions, "position")
    qd = _as_joint_matrix(velocities, "velocity")
    if q.shape != qd.shape:
        raise ValueError("position and velocity samples must have matching shapes")

    spans = np.ptp(q, axis=0)
    peak_velocities = np.max(np.abs(qd), axis=0)
    span_ratio = spans / max(float(max_position_span_rad), 1e-12)
    velocity_ratio = peak_velocities / max(float(max_velocity_rad_s), 1e-12)
    worst_index = int(np.argmax(np.maximum(span_ratio, velocity_ratio)))
    max_span = float(np.max(spans))
    max_velocity = float(np.max(peak_velocities))
    return JointStabilityReport(
        stable=(
            max_span <= float(max_position_span_rad)
            and max_velocity <= float(max_velocity_rad_s)
        ),
        worst_joint=worst_index + 1,
        max_position_span_rad=max_span,
        max_velocity_rad_s=max_velocity,
    )


def capture_stable_rgbd(
    camera: Any,
    arm: Any,
    *,
    joint_count: int,
    sample_count: int,
    sample_interval_s: float,
    max_position_span_rad: float,
    max_velocity_rad_s: float,
    frame_retries: int = 3,
    frame_retry_interval_s: float = 0.05,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> StableRgbdCapture:
    if sample_count < 2:
        raise ValueError("sample_count must be at least 2")
    if frame_retries < 0:
        raise ValueError("frame_retries must be nonnegative")

    positions: list[np.ndarray] = []
    velocities: list[np.ndarray] = []
    for index in range(sample_count):
        q, qd, _ = arm.get_state(request_feedback=False)
        positions.append(np.asarray(q[:joint_count], dtype=np.float64).copy())
        velocities.append(np.asarray(qd[:joint_count], dtype=np.float64).copy())
        if index + 1 < sample_count and sample_interval_s > 0.0:
            sleep_fn(float(sample_interval_s))

    q_before = positions[-1]
    capture_started = time.monotonic()
    color_bgr = depth_mm = None
    for attempt in range(frame_retries + 1):
        color_bgr, depth_mm = camera.get_frame()
        if color_bgr is not None and depth_mm is not None:
            break
        if attempt < frame_retries and frame_retry_interval_s > 0.0:
            sleep_fn(float(frame_retry_interval_s))
    capture_finished = time.monotonic()
    q_after_raw, qd_after_raw, _ = arm.get_state(request_feedback=False)
    q_after = np.asarray(q_after_raw[:joint_count], dtype=np.float64).copy()
    qd_after = np.asarray(qd_after_raw[:joint_count], dtype=np.float64).copy()
    positions.append(q_after)
    velocities.append(qd_after)

    if color_bgr is None or depth_mm is None:
        reason = str(getattr(camera, "last_frame_failure_reason", "")).strip()
        detail = f": {reason}" if reason else ""
        raise RuntimeError(
            "RGB-D frame capture failed after "
            f"{frame_retries + 1} attempts{detail}"
        )

    report = assess_joint_stability(
        positions,
        velocities,
        max_position_span_rad=max_position_span_rad,
        max_velocity_rad_s=max_velocity_rad_s,
    )
    if not report.stable:
        raise RobotUnstableError(
            "robot is moving during eye-in-hand capture: "
            f"joint{report.worst_joint} span={report.max_position_span_rad:.5f}rad "
            f"velocity={report.max_velocity_rad_s:.5f}rad/s"
        )

    return StableRgbdCapture(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        q_at_capture=0.5 * (q_before + q_after),
        stability=report,
        capture_monotonic=0.5 * (capture_started + capture_finished),
    )


def assess_motion_settled(
    target: np.ndarray,
    positions: Sequence[np.ndarray],
    velocities: Sequence[np.ndarray],
    *,
    position_tolerance_rad: float,
    velocity_tolerance_rad_s: float,
    required_consecutive_samples: int,
) -> MotionSettleReport:
    q = _as_joint_matrix(positions, "position")
    qd = _as_joint_matrix(velocities, "velocity")
    target = np.asarray(target, dtype=np.float64).reshape(-1)
    if q.shape != qd.shape or q.shape[1] != target.size:
        raise ValueError("target, position, and velocity dimensions must match")
    if required_consecutive_samples <= 0:
        raise ValueError("required_consecutive_samples must be positive")

    position_errors = np.max(np.abs(q - target.reshape(1, -1)), axis=1)
    peak_velocities = np.max(np.abs(qd), axis=1)
    consecutive = 0
    for position_error, velocity in zip(position_errors, peak_velocities):
        if (
            position_error <= float(position_tolerance_rad)
            and velocity <= float(velocity_tolerance_rad_s)
        ):
            consecutive += 1
        else:
            consecutive = 0

    return MotionSettleReport(
        settled=consecutive >= required_consecutive_samples,
        max_position_error_rad=float(position_errors[-1]),
        max_velocity_rad_s=float(peak_velocities[-1]),
        consecutive_samples=consecutive,
    )


def solve_pose_sequence(
    solve_fn: Callable[[Any, np.ndarray], Any],
    poses: Iterable[Any],
    q_seed: np.ndarray,
    *,
    solve_order: Sequence[int] | None = None,
) -> PoseSequenceResult:
    pose_list = tuple(poses)
    order = (
        tuple(range(len(pose_list)))
        if solve_order is None
        else tuple(int(index) for index in solve_order)
    )
    if sorted(order) != list(range(len(pose_list))):
        raise ValueError("solve_order must be a permutation of pose indices")
    seed = np.asarray(q_seed, dtype=np.float64).copy()
    solutions_by_index: dict[int, Any] = {}
    for index in order:
        pose = pose_list[index]
        result = solve_fn(pose, seed)
        if not bool(result.success):
            return PoseSequenceResult(
                success=False,
                solutions=tuple(
                    solutions_by_index[index]
                    for index in range(len(pose_list))
                    if index in solutions_by_index
                ),
                failed_index=index,
                error=float(result.error),
            )
        solutions_by_index[index] = result
        seed = np.asarray(result.q, dtype=np.float64).copy()
    return PoseSequenceResult(
        success=True,
        solutions=tuple(solutions_by_index[index] for index in range(len(pose_list))),
        failed_index=None,
        error=0.0,
    )


def solve_grasp_pose_sequence(
    solve_fn: Callable[[Any, np.ndarray], Any],
    *,
    pregrasp_pose: Any,
    grasp_pose: Any,
    retreat_pose: Any,
    q_seed: np.ndarray,
) -> PoseSequenceResult:
    """Solve grasp first, with a pregrasp-seeded recovery path.

    Returned solutions always follow execution order: pregrasp, grasp,
    retreat. Retreat is seeded from grasp because that is the preceding
    execution waypoint.
    """
    initial_seed = np.asarray(q_seed, dtype=np.float64).copy()
    bootstrap_pregrasp = None
    grasp_result = solve_fn(grasp_pose, initial_seed)
    if not bool(grasp_result.success):
        bootstrap_pregrasp = solve_fn(pregrasp_pose, initial_seed)
        if not bool(bootstrap_pregrasp.success):
            return PoseSequenceResult(
                success=False,
                solutions=(),
                failed_index=0,
                error=float(bootstrap_pregrasp.error),
            )
        grasp_result = solve_fn(
            grasp_pose,
            np.asarray(bootstrap_pregrasp.q, dtype=np.float64),
        )
        if not bool(grasp_result.success):
            return PoseSequenceResult(
                success=False,
                solutions=(bootstrap_pregrasp,),
                failed_index=1,
                error=float(grasp_result.error),
            )

    grasp_seed = np.asarray(grasp_result.q, dtype=np.float64)
    pregrasp_result = solve_fn(pregrasp_pose, grasp_seed)
    if not bool(pregrasp_result.success):
        if bootstrap_pregrasp is None:
            bootstrap_pregrasp = solve_fn(pregrasp_pose, initial_seed)
            if not bool(bootstrap_pregrasp.success):
                return PoseSequenceResult(
                    success=False,
                    solutions=(grasp_result,),
                    failed_index=0,
                    error=float(bootstrap_pregrasp.error),
                )
            grasp_result = solve_fn(
                grasp_pose,
                np.asarray(bootstrap_pregrasp.q, dtype=np.float64),
            )
            if not bool(grasp_result.success):
                return PoseSequenceResult(
                    success=False,
                    solutions=(bootstrap_pregrasp,),
                    failed_index=1,
                    error=float(grasp_result.error),
                )
            grasp_seed = np.asarray(grasp_result.q, dtype=np.float64)
            pregrasp_result = solve_fn(pregrasp_pose, grasp_seed)
        if not bool(pregrasp_result.success):
            pregrasp_result = bootstrap_pregrasp

    retreat_result = solve_fn(retreat_pose, grasp_seed)
    if not bool(retreat_result.success):
        return PoseSequenceResult(
            success=False,
            solutions=(pregrasp_result, grasp_result),
            failed_index=2,
            error=float(retreat_result.error),
        )
    return PoseSequenceResult(
        success=True,
        solutions=(pregrasp_result, grasp_result, retreat_result),
        failed_index=None,
        error=0.0,
    )
