"""Direct official SDK robot adapter.

Only this module may connect official SDK in grasp route. ROS client remains
separate and owns no motor port.
"""

from __future__ import annotations

from pathlib import Path
import threading
import time
from types import MethodType
from typing import Any

import numpy as np
import pinocchio as pin

from .grasp_driver import (
    GraspDriver,
    ensure_rebot_sdk_in_syspath,
    find_rebot_repo_root,
    selected_arm_config,
)
try:
    from ...utils.sdk_exclusive import assert_sdk_exclusive
except ImportError:
    from utils.sdk_exclusive import assert_sdk_exclusive


READY_JOINTS = np.array(
    [
        -0.041390419006347656,
        0.697718620300293,
        0.8133068084716797,
        -0.7553215026855469,
        -0.019837379455566406,
        -0.02651214599609375,
    ],
    dtype=np.float64,
)
# FK of READY_JOINTS at official_tcp in config/sixaxis.urdf after the joint6
# local-axis -90 degree installation correction.
READY_POSE = np.array(
    [
        0.2542153175174602,
        0.00684814707127781,
        0.22925405726651077,
        -1.5591314277201471,
        0.6395942614734124,
        0.016673713583102438,
    ],
    dtype=np.float64,
)
SDK_GRAVITY_TAU_SCALE = (1.0, 1.55, 1.55, 1.0, 1.0, 1.0)


class OfficialKinematics:
    """Six-axis local model whose official_tcp equals SDK end_link."""

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        multistart_retries: int = 0,
    ) -> None:
        self.urdf_path = Path(urdf_path).resolve()
        self.model = pin.buildModelFromUrdf(str(self.urdf_path))
        if self.model.nq != 6:
            raise ValueError(f"official SDK model must have 6 joints, got {self.model.nq}")
        self.frame_id = int(self.model.getFrameId("official_tcp"))
        if self.frame_id >= self.model.nframes:
            raise ValueError("official SDK model is missing official_tcp")
        self.multistart_retries = int(multistart_retries)
        if self.multistart_retries < 0:
            raise ValueError("multistart_retries must not be negative")

    @property
    def joint_limits(self) -> np.ndarray:
        return np.column_stack(
            [self.model.lowerPositionLimit, self.model.upperPositionLimit]
        ).astype(np.float64)

    def fk(self, joints: np.ndarray) -> np.ndarray:
        return self.fk_frame(joints, "official_tcp")

    def fk_frame(self, joints: np.ndarray, frame_name: str) -> np.ndarray:
        """Return base-to-frame FK for a named frame in the shared SDK model."""
        q = np.asarray(joints, dtype=np.float64).reshape(6)
        frame = str(frame_name).strip()
        frame_id = int(self.model.getFrameId(frame))
        if not frame or frame_id >= self.model.nframes:
            raise ValueError(f"official SDK model is missing frame {frame!r}")
        data = self.model.createData()
        pin.forwardKinematics(self.model, data, q)
        pin.updateFramePlacements(self.model, data)
        return data.oMf[frame_id].homogeneous.copy()

    def solve_pose_sequence(
        self,
        poses: list[np.ndarray] | tuple[np.ndarray, ...],
        seed_joints: np.ndarray,
        *,
        max_iterations: int = 600,
        tolerance: float = 1e-4,
    ) -> list[np.ndarray]:
        q = np.asarray(seed_joints, dtype=np.float64).reshape(6).copy()
        solutions = []
        for index, pose in enumerate(poses):
            values = np.asarray(pose, dtype=np.float64).reshape(6)
            if not np.all(np.isfinite(values)):
                raise ValueError(f"pose {index} contains non-finite values")
            from utils.transforms import pose6d_to_mat4

            matrix = pose6d_to_mat4(*values)
            target = pin.SE3(matrix[:3, :3], matrix[:3, 3])
            seeds = [q.copy()]
            rng = np.random.default_rng(42 + index)
            for _ in range(self.multistart_retries):
                seeds.append(
                    rng.uniform(
                        self.model.lowerPositionLimit,
                        self.model.upperPositionLimit,
                    )
                )
            solution = None
            solution_delta = float("inf")
            solution_residual = float("inf")
            best_residual = float("inf")
            best_q = q.copy()
            for seed in seeds:
                candidate, residual, candidate_q = self._solve_target(
                    target,
                    seed,
                    max_iterations=int(max_iterations),
                    tolerance=float(tolerance),
                )
                if residual < best_residual:
                    best_residual = residual
                    best_q = candidate_q
                if candidate is not None:
                    candidate_delta = float(
                        np.max(np.abs(np.asarray(candidate) - q))
                    )
                    if (candidate_delta, residual) < (
                        solution_delta,
                        solution_residual,
                    ):
                        solution = candidate
                        solution_delta = candidate_delta
                        solution_residual = residual
            if solution is None:
                target_text = np.array2string(
                    values,
                    precision=4,
                    separator=",",
                    floatmode="fixed",
                )
                best_q_text = np.array2string(
                    best_q,
                    precision=4,
                    separator=",",
                    floatmode="fixed",
                )
                raise RuntimeError(
                    f"IK failed for pose {index} target={target_text} "
                    f"best_residual={best_residual:.6f} best_q={best_q_text}"
                )
            q = solution
            solutions.append(q.copy())
        return solutions

    def _solve_target(
        self,
        target: pin.SE3,
        seed: np.ndarray,
        *,
        max_iterations: int,
        tolerance: float,
    ) -> tuple[np.ndarray | None, float, np.ndarray]:
        q = np.asarray(seed, dtype=np.float64).reshape(6).copy()
        data = self.model.createData()
        best_residual = float("inf")
        best_q = q.copy()
        for _ in range(max_iterations):
            pin.forwardKinematics(self.model, data, q)
            pin.updateFramePlacements(self.model, data)
            error = pin.log(data.oMf[self.frame_id].actInv(target)).vector
            residual = float(np.linalg.norm(error))
            if residual < best_residual:
                best_residual = residual
                best_q = q.copy()
            if residual < tolerance:
                return q, residual, q.copy()
            # Match zekeeparm_SDK's official damped CLIK solver:
            # explicitly refresh joint Jacobians and accept only decreasing
            # steps through a short backtracking line search.
            pin.computeJointJacobians(self.model, data, q)
            jacobian = pin.computeFrameJacobian(
                self.model,
                data,
                q,
                self.frame_id,
                pin.ReferenceFrame.LOCAL,
            )
            velocity = 0.5 * jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + 1e-6 * np.eye(6),
                error,
            )
            accepted = False
            for _ in range(4):
                candidate = pin.integrate(self.model, q, velocity)
                candidate = np.clip(
                    candidate,
                    self.model.lowerPositionLimit,
                    self.model.upperPositionLimit,
                )
                pin.forwardKinematics(self.model, data, candidate)
                pin.updateFramePlacements(self.model, data)
                candidate_error = pin.log(
                    data.oMf[self.frame_id].actInv(target)
                ).vector
                if float(np.linalg.norm(candidate_error)) < residual:
                    q = candidate
                    accepted = True
                    break
                velocity *= 0.5
            if not accepted:
                continue
        return None, best_residual, best_q


class OfficialSdkRobot:
    """SDK ownership, FK/IK, trajectory, gripper facade."""

    def __init__(self, cfg: dict) -> None:
        self.cfg = cfg
        robot_cfg = cfg.get("robot") or {}
        self.repo_root = find_rebot_repo_root(robot_cfg.get("repo_root"))
        official_cfg = cfg.get("official_sdk") or {}
        self.arm_control_mode = official_cfg.get("arm_control_mode")
        self.mit_kp = self._optional_joint_vector(official_cfg, "mit_kp")
        self.mit_kd = self._optional_joint_vector(official_cfg, "mit_kd")
        self.gravity_tau_scale = self._optional_joint_vector(
            official_cfg,
            "gravity_tau_scale",
            default=SDK_GRAVITY_TAU_SCALE,
        )
        self.gravity_tau_scale_positive = self._optional_joint_vector(
            official_cfg,
            "gravity_tau_scale_positive",
        )
        if self.gravity_tau_scale_positive is None:
            self.gravity_tau_scale_positive = self.gravity_tau_scale.copy()
        self.tcp_frame = str(official_cfg.get("tcp_frame", "official_tcp"))
        urdf_path = Path(str(official_cfg.get("urdf", "config/sixaxis.urdf")))
        if not urdf_path.is_absolute():
            urdf_path = Path(__file__).resolve().parents[2] / urdf_path
        ik_cfg = (cfg.get("grasp_pipeline") or {}).get("ik") or {}
        self.kinematics = OfficialKinematics(
            urdf_path,
            multistart_retries=int(ik_cfg.get("multistart_retries", 0)),
        )
        self._arm: Any = None
        self._controller: Any = None
        self.grasp_driver: GraspDriver | None = None

    @staticmethod
    def _optional_joint_vector(
        config: dict,
        name: str,
        *,
        default: tuple[float, ...] | None = None,
    ) -> np.ndarray | None:
        value = config.get(name)
        if value is None:
            value = default
        if value is None:
            return None
        try:
            vector = np.asarray(value, dtype=np.float64).reshape(-1)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"official_sdk.{name} must contain 6 numbers") from exc
        if vector.size != 6:
            raise ValueError(f"official_sdk.{name} must contain exactly 6 values")
        if not np.all(np.isfinite(vector)):
            raise ValueError(f"official_sdk.{name} must contain finite values")
        if np.any(vector < 0.0):
            raise ValueError(f"official_sdk.{name} must not contain negative values")
        return vector.copy()

    def _apply_mit_config(self, arm_group: Any) -> None:
        if int(arm_group.num_joints) != 6:
            raise RuntimeError(
                f"official SDK arm group must have 6 joints, got {arm_group.num_joints}"
            )
        if self.mit_kp is not None:
            arm_group._mit_kp = self.mit_kp.copy()
        if self.mit_kd is not None:
            arm_group._mit_kd = self.mit_kd.copy()

    def _apply_gravity_tau_scale(self, controller: Any) -> None:
        """Scale arm MIT feed-forward torque, which this route uses for gravity."""
        if controller._arm_control_mode != "mit":
            raise RuntimeError("gravity_tau_scale requires MIT arm control")
        arm_group = controller._arm_group
        sdk_scale = np.asarray(SDK_GRAVITY_TAU_SCALE)
        correction_negative = self.gravity_tau_scale / sdk_scale
        correction_positive = self.gravity_tau_scale_positive / sdk_scale
        raw_send_mit = arm_group.send_mit

        def scaled_send_mit(
            _group: Any,
            pos: np.ndarray,
            vel: np.ndarray | None = None,
            kp: np.ndarray | None = None,
            kd: np.ndarray | None = None,
            tau: np.ndarray | None = None,
        ) -> None:
            scaled_tau = None
            if tau is not None:
                raw_tau = np.asarray(tau, dtype=np.float64)
                correction = np.where(
                    raw_tau > 0.0,
                    correction_positive,
                    correction_negative,
                )
                scaled_tau = raw_tau * correction
            raw_send_mit(
                pos,
                vel=vel,
                kp=kp,
                kd=kd,
                tau=scaled_tau,
            )

        arm_group.send_mit = MethodType(scaled_send_mit, arm_group)

    def _validate_private_sdk_api(self) -> None:
        configured = int((self.cfg.get("official_sdk") or {}).get("sdk_private_api_version", 0))
        if configured != 1:
            raise RuntimeError(
                f"unsupported SDK private API version {configured}; expected 1"
            )
        required = (
            "_q_target",
            "_qd_target",
            "_send_thread",
            "_moving",
            "_arm_control_mode",
            "_arm_group",
            "_loop_cb",
        )
        missing = [name for name in required if not hasattr(self._controller, name)]
        if missing:
            raise RuntimeError(
                "official SDK private API mismatch; missing " + ", ".join(missing)
            )

    @property
    def control_loop_active(self) -> bool:
        return bool(self._arm is not None and self._arm.control_loop_active)

    @staticmethod
    def ready_joints() -> np.ndarray:
        return READY_JOINTS.copy()

    @staticmethod
    def ready_pose() -> np.ndarray:
        return READY_POSE.copy()

    def connect(self) -> None:
        robot_cfg = self.cfg.get("robot") or {}
        hw = selected_arm_config(
            str(self.repo_root),
            controller_mode=self.arm_control_mode,
        )
        serial = self._serial_from_sdk_config()
        assert_sdk_exclusive(serial_path=serial)
        ensure_rebot_sdk_in_syspath(str(self.repo_root))
        from zekeeparm_SDK.actuator import RebotArm
        from zekeeparm_SDK.controllers import RebotArmEndPose

        self._arm = RebotArm()
        if hw.controller_mode == "mit":
            self._apply_mit_config(self._arm.groups["arm"])
        self._controller = RebotArmEndPose(
            self._arm,
            arm_control_mode=hw.controller_mode,
            urdf_path=self.kinematics.urdf_path,
            end_effector_frame=self.tcp_frame,
        )
        self._validate_private_sdk_api()
        gripper_config = dict(robot_cfg.get("gripper") or {})
        gripper_config["max_distance_m"] = float(
            robot_cfg.get("gripper_max_width_m", 0.070)
        )
        self.grasp_driver = GraspDriver(
            self._arm,
            self._controller,
            gripper_config=gripper_config,
            joint_mapping_config=robot_cfg.get("joint_mapping"),
            repo_root=str(self.repo_root),
            arm_control_mode=hw.controller_mode,
        )
        if hw.controller_mode == "mit":
            self._apply_gravity_tau_scale(self._controller)
        self.grasp_driver.start()

    def move_to_traj(self, pose6d: np.ndarray, duration_s: float) -> bool:
        if self._controller is None:
            raise RuntimeError("SDK robot is not connected")
        pose = np.asarray(pose6d, dtype=np.float64).reshape(6)
        if not np.all(np.isfinite(pose)):
            raise ValueError("trajectory pose contains non-finite values")
        return bool(self._controller.move_to_traj(*pose, duration=float(duration_s)))

    def move_to_traj_and_wait(
        self,
        pose6d: np.ndarray,
        expected_joints: np.ndarray,
        duration_s: float,
        cancel_event=None,
    ) -> None:
        if not self.move_to_traj(pose6d, duration_s):
            raise RuntimeError("official Cartesian trajectory planning failed")
        thread = self._controller._send_thread
        if thread is None:
            raise RuntimeError("official Cartesian trajectory thread missing")
        deadline = time.monotonic() + float(self._controller._traj_duration) + 5.0
        while thread.is_alive() and time.monotonic() < deadline:
            thread.join(timeout=0.05)
            if cancel_event is not None and cancel_event.is_set():
                self.hold_current()
                raise RuntimeError("motion cancelled: control client disconnected")
        if thread.is_alive() or self._controller._moving:
            raise RuntimeError("official Cartesian trajectory interrupted")
        if not self._arm.control_loop_active:
            raise RuntimeError("official SDK control loop stopped")
        self._wait_for_target_reached(expected_joints)

    def move_joints_and_wait(
        self,
        target_joints: np.ndarray,
        duration_s: float,
        cancel_event=None,
    ) -> None:
        if self._controller is None:
            raise RuntimeError("SDK robot is not connected")
        start = self.current_joints()
        target = np.asarray(target_joints, dtype=np.float64).reshape(6)
        limits = self.kinematics.joint_limits
        if not np.all(np.isfinite(target)) or np.any(target < limits[:, 0]) or np.any(target > limits[:, 1]):
            raise ValueError("joint target is non-finite or outside official model limits")
        steps = max(2, int(float(duration_s) * 50.0))
        fractions = np.linspace(0.0, 1.0, steps)
        blends = 10.0 * fractions**3 - 15.0 * fractions**4 + 6.0 * fractions**5
        points = start + blends[:, None] * (target - start)
        duration_s = self._controller._posvel_duration(points, start, float(duration_s))
        for point in points:
            if cancel_event is not None and cancel_event.is_set():
                self.hold_current()
                raise RuntimeError("motion cancelled: control client disconnected")
            self._controller._q_target[:] = point
            time.sleep(float(duration_s) / steps)
            if not self._arm.control_loop_active:
                raise RuntimeError("official SDK control loop stopped")
        self._wait_for_target_reached(target)

    def kinematics_pose(self, joints: np.ndarray) -> np.ndarray:
        from utils.transforms import mat4_to_pose6d

        return np.asarray(mat4_to_pose6d(self.kinematics.fk(joints)), dtype=np.float64)

    def move_ready(self, cancel_event=None) -> None:
        duration = float((self.cfg.get("robot") or {}).get("ready_pose", {}).get("duration", 8.0))
        self.move_joints_and_wait(
            self.ready_joints(),
            duration,
            cancel_event=cancel_event,
        )

    def hold_current(self) -> None:
        """Cancel trajectory updates and hold the measured arm position."""
        if self._controller is None or self.grasp_driver is None:
            return
        self._controller._stop_send.set()
        thread = self._controller._send_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._controller._moving = False
        joints = np.asarray(
            self._arm.get_state(request_feedback=True)[0][:6],
            dtype=np.float64,
        )
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise RuntimeError("cannot hold current pose: invalid joint feedback")
        self._controller._q_target[:] = joints
        self._controller._qd_target[:] = 0.0

    def _assert_target_reached(self, expected_joints: np.ndarray) -> None:
        actual = self.current_joints()
        expected = np.asarray(expected_joints, dtype=np.float64).reshape(6)
        errors = np.abs(actual - expected)
        worst_joint = int(np.argmax(errors))
        error = float(errors[worst_joint])
        tolerance = float((self.cfg.get("safety") or {}).get("motion_position_tolerance_rad", 0.005))
        if error > tolerance:
            raise RuntimeError(
                f"joint target did not settle: joint{worst_joint + 1} "
                f"actual={actual[worst_joint]:.4f}rad target={expected[worst_joint]:.4f}rad "
                f"error={error:.4f}rad limit={tolerance:.4f}rad"
            )

    def _wait_for_target_reached(self, expected_joints: np.ndarray) -> None:
        safety = self.cfg.get("safety") or {}
        expected = np.asarray(expected_joints, dtype=np.float64).reshape(6)
        position_tolerance = float(safety.get("motion_position_tolerance_rad", 0.005))
        velocity_tolerance = float(safety.get("motion_velocity_tolerance_rad_s", 0.03))
        required_samples = int(safety.get("motion_settle_samples", 5))
        sample_interval = float(safety.get("motion_sample_interval_s", 0.05))
        deadline = time.monotonic() + float(safety.get("motion_settle_timeout_s", 3.0))
        consecutive = 0
        actual = np.zeros(6, dtype=np.float64)
        velocity = np.zeros(6, dtype=np.float64)
        while time.monotonic() < deadline:
            actual, velocity, _ = self._arm.get_state(request_feedback=True)
            actual = np.asarray(actual[:6], dtype=np.float64)
            velocity = np.asarray(velocity[:6], dtype=np.float64)
            if not np.all(np.isfinite(actual)) or not np.all(np.isfinite(velocity)):
                raise RuntimeError("invalid joint feedback while waiting for motion settle")
            if (np.max(np.abs(actual - expected)) <= position_tolerance
                    and np.max(np.abs(velocity)) <= velocity_tolerance):
                consecutive += 1
                if consecutive >= required_samples:
                    return
            else:
                consecutive = 0
            time.sleep(sample_interval)

        errors = np.abs(actual - expected)
        worst_joint = int(np.argmax(errors))
        raise RuntimeError(
            f"joint target did not settle: joint{worst_joint + 1} "
            f"actual={actual[worst_joint]:.4f}rad target={expected[worst_joint]:.4f}rad "
            f"error={errors[worst_joint]:.4f}rad limit={position_tolerance:.4f}rad "
            f"max_velocity={np.max(np.abs(velocity)):.4f}rad/s "
            f"velocity_limit={velocity_tolerance:.4f}rad/s"
        )

    def current_joints(self) -> np.ndarray:
        if self.grasp_driver is None:
            raise RuntimeError("SDK robot is not connected")
        return self.grasp_driver.read_stable_arm_positions()

    def get_tcp_pose(self) -> np.ndarray:
        return self.kinematics.fk(self.current_joints())

    def solve_pose_sequence(self, poses) -> list[np.ndarray]:
        return self.kinematics.solve_pose_sequence(list(poses), self.current_joints())

    def open_gripper(self, width_m: float = 0.07, cancel_event=None) -> None:
        if self.grasp_driver is None:
            raise RuntimeError("SDK robot is not connected")
        if cancel_event is None:
            self.grasp_driver.open_gripper(width_m)
        else:
            self.grasp_driver.open_gripper(width_m, cancel_event=cancel_event)

    def grasp(self, cancel_event=None) -> bool:
        if self.grasp_driver is None:
            raise RuntimeError("SDK robot is not connected")
        if cancel_event is None:
            return self.grasp_driver.grasp()
        return self.grasp_driver.grasp(cancel_event=cancel_event)

    def release(self, cancel_event=None) -> None:
        if self.grasp_driver is None:
            raise RuntimeError("SDK robot is not connected")
        if cancel_event is None:
            self.grasp_driver.release_gripper()
        else:
            self.grasp_driver.release_gripper(cancel_event=cancel_event)

    def close(self) -> None:
        # RebotArmEndPose.end() calls safe_home(), which violates route fault
        # policy and can create an unplanned motion during shutdown.
        if self._arm is not None:
            self._arm.disconnect()
        self._controller = None
        self._arm = None
        self.grasp_driver = None

    def _serial_from_sdk_config(self) -> str | None:
        config_path = self.repo_root / "config" / "rebotarm_dm.yaml"
        if not config_path.exists():
            return None
        import yaml

        data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        channel = data.get("channel")
        return str(channel) if channel else None
