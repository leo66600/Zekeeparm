from __future__ import annotations

import functools
import math
import threading
import time

import numpy as np

from .hardware_config import resolve_hardware_config
from .dynamics_validation import validate_dynamics_model
from .gripper_grasp import GripperGraspConfig, GripperGraspState
from .joint_mapping import joint_mapping_from_config

_GRIPPER_GOAL_TOLERANCE_RAD = 0.12
_GRIPPER_CLOSED_POSITION = 0.0


def _load_gripper_grasp_profile(hardware_data):
    gripper = hardware_data.get("gripper", {}) or {}
    limits = gripper.get("position_limits", {}) or {}
    open_position = float(limits.get("open", 0.0))
    close_position = float(limits.get("close", 0.0))
    max_width = float(gripper.get("max_width_m", 0.0))
    grasp = gripper.get("grasp", {}) or {}
    config = GripperGraspConfig(
        open_position=open_position,
        close_position=close_position,
        closing_torque=float(grasp.get("closing_torque", 1.0)),
        hold_torque=float(grasp.get("hold_torque", 0.30)),
        max_torque=float(grasp.get("max_torque", 1.5)),
        stall_velocity_rad_s=float(grasp.get("stall_velocity_rad_s", 0.05)),
        contact_torque=float(grasp.get("contact_torque", 0.10)),
        minimum_motion_rad=float(grasp.get("minimum_motion_rad", 0.30)),
        hard_stop_margin_rad=float(grasp.get("hard_stop_margin_rad", 0.05)),
        kp_move=float(grasp.get("kp_move", 5.0)),
        kd_move=float(grasp.get("kd_move", 1.0)),
        kd_close=float(grasp.get("kd_close", 0.5)),
    )
    timeout = float(grasp.get("timeout_s", 5.0))
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("gripper grasp timeout_s must be finite and positive")
    if not all(
        math.isfinite(value)
        for value in (open_position, close_position, max_width)
    ):
        raise ValueError("gripper positions and max_width_m must be finite")
    if max_width < 0.0:
        raise ValueError("gripper max_width_m must be nonnegative")
    return open_position, close_position, max_width, config, timeout


def _load_configured_dynamics_model(load_robot_model, hardware_data):
    urdf_path = str(hardware_data.get("urdf_path", "")).strip()
    if not urdf_path:
        raise ValueError("hardware config must define urdf_path for dynamics")
    return load_robot_model(urdf_path)


def _locked(method):
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        with self._cmd_lock:
            return method(self, *args, **kwargs)

    return wrapper


class HardwareManager:
    """ROS-facing adapter for the new grouped Zekeep SDK."""

    def __init__(
        self,
        hardware_config: str | None = None,
        model: str = "",
        channel: str = "",
        auto_enable: bool = False,
    ) -> None:
        self._cmd_lock = threading.RLock()
        hardware_config_path, hardware_data = resolve_hardware_config(
            hardware_config,
            model,
            channel,
        )

        from zekeeparm_SDK.actuator import RebotArm
        from zekeeparm_SDK.controllers import RebotArmEndPose
        from zekeeparm_SDK.dynamics import compute_generalized_gravity
        from zekeeparm_SDK.kinematics import (
            compute_fk,
            load_robot_model,
            pad_q_for_model,
            pos_rot_to_se3,
            solve_ik,
        )

        self._compute_fk = compute_fk
        self._pad_q_for_model = pad_q_for_model
        self._pos_rot_to_se3 = pos_rot_to_se3
        self._solve_ik = solve_ik

        runtime_config = hardware_data["_runtime"]
        control_runtime = runtime_config["control"]
        self._arm_control_mode = control_runtime["arm_control_mode"]
        self._gravity_feedforward_enabled = bool(
            control_runtime.get("gravity_feedforward_enabled", False)
        )
        self._gravity_feedforward_joints = list(
            control_runtime.get("gravity_feedforward_joints", [])
        )
        self._motion_authorized = bool(
            (hardware_data.get("control", {}) or {}).get("motion_authorized", False)
        )
        self._auto_enable = bool(auto_enable)
        self._allow_raw_arm_mit = bool(
            (hardware_data.get("control", {}) or {}).get(
                "allow_raw_arm_mit", False
            )
        )
        (
            self.gripper_open_position,
            self.gripper_close_position,
            self.gripper_max_width,
            self._gripper_grasp_config,
            self.gripper_grasp_timeout,
        ) = _load_gripper_grasp_profile(hardware_data)
        self._gripper_grasp_state: GripperGraspState | None = None

        self._robot = RebotArm(hw_yaml=str(hardware_config_path))
        self._arm_group = self._robot.groups.get("arm")
        if self._arm_group is None:
            raise ValueError("hardware config must define groups.arm")
        self._gravity_feedforward_mask = np.asarray(
            control_runtime.get(
                "gravity_feedforward_mask",
                [
                    name in self._gravity_feedforward_joints
                    for name in self._arm_group.joint_names
                ],
            ),
            dtype=bool,
        )
        if self._gravity_feedforward_mask.shape != (len(self._arm_group.joint_names),):
            raise ValueError("gravity feedforward mask does not match arm joints")
        self._gripper_group = self._robot.groups.get("gripper")
        self._joint_mapping = joint_mapping_from_config(
            hardware_data,
            self._arm_group.joint_names,
        )
        self._robot_get_state = self._robot.get_state
        self._robot.get_state = self._get_arm_state
        self._arm_mit_kp = np.array(control_runtime["mit_kp"], dtype=np.float64)
        self._arm_mit_kd = np.array(control_runtime["mit_kd"], dtype=np.float64)
        configured_vlim = control_runtime.get("pos_vel_vlim")
        self._arm_pos_vel_vlim = np.array(
            configured_vlim
            if configured_vlim is not None
            else self._joint_mapping.ros_velocity_limits,
            dtype=np.float64,
        )
        self._endpos_ctrl = RebotArmEndPose(
            self._robot,
            arm_control_mode=self._arm_control_mode,
        )
        self._hybrid_arm_groups = {}
        if self._arm_control_mode == "posvel" and self._gravity_feedforward_enabled:
            for name in self.joint_names:
                group_name = f"_hybrid_{name}"
                self._hybrid_arm_groups[name] = self._robot.add_group(
                    group_name, [name]
                )
                # These are internal views of the arm, not user-facing groups;
                # keep global enable/disable operations from touching motors twice.
                self._robot._groups.pop(group_name, None)

        self._gripper_name = (
            self._gripper_group.joint_names[0]
            if self.has_gripper and self._gripper_group.joint_names
            else ""
        )
        self._gripper_mapping = (
            joint_mapping_from_config(hardware_data, [self._gripper_name])
            if self._gripper_name
            else None
        )
        self._gripper_target_position: float | None = None

        gravity_config = hardware_data.get("gravity_compensation", {}) or {}
        self._gravity_compensation_enabled = bool(gravity_config.get("enabled", True))
        self._gravity_comp_trial_authorized = bool(
            gravity_config.get("supervised_trial_authorized", False)
        )
        self._gravity_comp_dynamics_verified = bool(
            gravity_config.get("dynamics_verified", False)
        )
        self._gravity_comp_dynamics_source = str(
            gravity_config.get("dynamics_source", "")
        ).strip()
        stop_assist = gravity_config.get("stop_assist", {}) or {}
        if not isinstance(stop_assist, dict):
            raise ValueError("gravity_compensation.stop_assist must be a mapping")
        self._gravity_stop_assist_enabled = bool(stop_assist.get("enabled", False))
        configured_stop_assist_joints = stop_assist.get("joints")
        if configured_stop_assist_joints is None:
            stop_assist_joints = [str(stop_assist.get("joint", "joint2")).strip()]
        elif not isinstance(configured_stop_assist_joints, list):
            raise ValueError(
                "gravity_compensation.stop_assist.joints must be a list"
            )
        else:
            stop_assist_joints = [
                str(name).strip() for name in configured_stop_assist_joints
            ]
        if self._gravity_stop_assist_enabled and (
            not stop_assist_joints or any(not name for name in stop_assist_joints)
        ):
            raise ValueError(
                "gravity_compensation.stop_assist.joints must not be empty"
            )
        if len(stop_assist_joints) != len(set(stop_assist_joints)):
            raise ValueError(
                "gravity_compensation.stop_assist.joints must not contain duplicates"
            )
        unknown_stop_assist_joints = sorted(
            set(stop_assist_joints) - set(self.joint_names)
        )
        if self._gravity_stop_assist_enabled and unknown_stop_assist_joints:
            raise ValueError(
                "gravity_compensation.stop_assist.joints reference unknown arm "
                "joints: " + ", ".join(unknown_stop_assist_joints)
            )
        self._gravity_stop_assist_indices = (
            np.asarray(
                [self.joint_names.index(name) for name in stop_assist_joints],
                dtype=np.int64,
            )
            if self._gravity_stop_assist_enabled
            else np.zeros(0, dtype=np.int64)
        )
        self._gravity_stop_assist_engage_velocity = float(
            stop_assist.get("engage_velocity", 0.03)
        )
        release_velocity = np.asarray(
            stop_assist.get("release_velocity", 0.08), dtype=np.float64
        ).reshape(-1)
        if len(release_velocity) == 1:
            release_velocity = np.full(
                len(stop_assist_joints),
                float(release_velocity[0]),
                dtype=np.float64,
            )
        elif len(release_velocity) != len(stop_assist_joints):
            raise ValueError(
                "gravity_compensation.stop_assist.release_velocity must be a "
                "scalar or match the number of configured joints"
            )
        self._gravity_stop_assist_release_velocity = release_velocity
        position_gain = np.asarray(
            stop_assist.get("position_gain", 2.0), dtype=np.float64
        ).reshape(-1)
        if len(position_gain) == 1:
            position_gain = np.full(
                len(stop_assist_joints), float(position_gain[0]), dtype=np.float64
            )
        elif len(position_gain) != len(stop_assist_joints):
            raise ValueError(
                "gravity_compensation.stop_assist.position_gain must be a scalar "
                "or match the number of configured joints"
            )
        self._gravity_stop_assist_position_gain = position_gain
        torque_limit = np.asarray(
            stop_assist.get("torque_limit", 0.2), dtype=np.float64
        ).reshape(-1)
        if len(torque_limit) == 1:
            torque_limit = np.full(
                len(stop_assist_joints),
                float(torque_limit[0]),
                dtype=np.float64,
            )
        elif len(torque_limit) != len(stop_assist_joints):
            raise ValueError(
                "gravity_compensation.stop_assist.torque_limit must be a scalar "
                "or match the number of configured joints"
            )
        self._gravity_stop_assist_torque_limit = torque_limit
        stop_assist_scalar_values = (
            self._gravity_stop_assist_engage_velocity,
        )
        if not np.all(np.isfinite(stop_assist_scalar_values)) or not np.all(
            np.isfinite(self._gravity_stop_assist_release_velocity)
        ) or not np.all(
            np.isfinite(self._gravity_stop_assist_position_gain)
        ) or not np.all(
            np.isfinite(self._gravity_stop_assist_torque_limit)
        ):
            raise ValueError(
                "gravity_compensation.stop_assist values must be finite"
            )
        if self._gravity_stop_assist_engage_velocity < 0.0:
            raise ValueError(
                "gravity_compensation.stop_assist.engage_velocity must be nonnegative"
            )
        if np.any(
            self._gravity_stop_assist_release_velocity
            <= self._gravity_stop_assist_engage_velocity
        ):
            raise ValueError(
                "gravity_compensation.stop_assist.release_velocity must exceed "
                "engage_velocity"
            )
        if np.any(self._gravity_stop_assist_position_gain < 0.0):
            raise ValueError(
                "gravity_compensation.stop_assist.position_gain must be nonnegative"
            )
        if np.any(self._gravity_stop_assist_torque_limit < 0.0):
            raise ValueError(
                "gravity_compensation.stop_assist.torque_limit must be nonnegative"
            )
        self._motion_generation = 0
        self._shutdown_pending = False
        self._home_cancel = threading.Event()
        self.validate_home_path = None
        safe_home = hardware_data.get("safe_home", {}) or {}
        configured_home = safe_home.get("positions")
        self._safe_home_positions = (
            np.asarray(configured_home, dtype=np.float64).reshape(-1)
            if configured_home is not None
            else None
        )
        configured_ready = hardware_data.get("ready_pose", {}) or {}
        ready_positions = configured_ready.get("positions")
        self._ready_pose_positions = (
            np.asarray(ready_positions, dtype=np.float64).reshape(-1)
            if ready_positions is not None
            else None
        )

        self._gc_model = _load_configured_dynamics_model(
            load_robot_model,
            hardware_data,
        )
        self._gc_data = self._gc_model.createData()
        self._gc_compute_generalized_gravity = compute_generalized_gravity
        gc_runtime = runtime_config["gravity_compensation"]
        self._gravity_comp_kp = np.array(gc_runtime["kp"], dtype=np.float64)
        self._gravity_comp_kd = np.array(gc_runtime["kd"], dtype=np.float64)
        self._gravity_comp_tau_scale = np.array(
            gc_runtime["tau_scale"],
            dtype=np.float64,
        )
        self._gravity_comp_tau_scale_positive = np.array(
            gc_runtime["tau_scale_positive"],
            dtype=np.float64,
        )
        self._gravity_comp_tau_limit = np.array(
            gc_runtime["tau_limit"],
            dtype=np.float64,
        )
        self._gravity_comp_tau_rate_limit = np.array(
            gc_runtime["tau_rate_limit"],
            dtype=np.float64,
        )
        if self._gravity_feedforward_enabled:
            if not self._gravity_comp_dynamics_verified:
                raise RuntimeError(
                    "gravity feedforward requires verified mass, center-of-mass, "
                    "and inertia parameters"
                )
            if not self._gravity_comp_dynamics_source:
                raise RuntimeError(
                    "gravity feedforward requires dynamics provenance"
                )
            validate_dynamics_model(self._gc_model, self.joint_names)

        self._connected = False
        self._enabled = False
        self._control_output_enabled = False
        self._state_machine = "IDLE"
        self._error_codes: list[str] = []
        self._gravity_comp_active = False
        self._gravity_comp_q_last: np.ndarray | None = None
        self._gravity_comp_tau_last: np.ndarray | None = None
        self._gravity_stop_assist_positions: np.ndarray | None = None
        self._gravity_stop_assist_active = np.zeros(
            len(self._gravity_stop_assist_indices), dtype=bool
        )
        self._gravity_feedforward_tau_last: np.ndarray | None = None
        self._gravity_feedforward_q_last: np.ndarray | None = None
        self._gravity_feedforward_faulted = False
        self._homing_thread: int | None = None

    # ------------------------------------------------------------------
    # state
    # ------------------------------------------------------------------

    @property
    def joint_names(self) -> list[str]:
        return list(self._arm_group.joint_names)

    @property
    def mode(self) -> str:
        return str(self._arm_group.mode)

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def control_loop_active(self) -> bool:
        return bool(self._robot.control_loop_active)

    @property
    def has_gripper(self) -> bool:
        return bool(self._robot.has_gripper)

    @property
    def state_machine(self) -> str:
        return self._state_machine

    @property
    def error_codes(self) -> list[str]:
        return list(self._error_codes)

    @_locked
    def set_state_machine(self, state: str) -> None:
        if state not in (
            "IDLE",
            "TRAJ_RUNNING",
            "SERVO_RUNNING",
            "LOWLEVEL_STREAMING",
            "GRAVITY_COMP",
            "SAFE_HOMING",
            "SHUTDOWN_FAILED",
        ):
            raise ValueError(f"unsupported state machine value: {state}")
        self._state_machine = state

    # ------------------------------------------------------------------
    # arm
    # ------------------------------------------------------------------

    @_locked
    def connect(self) -> None:
        if self._connected:
            return
        try:
            self._robot.connect()
            if self.has_gripper:
                self._gripper_target_position = self.get_gripper_state()[0]
            self._connected = True
            if self._motion_authorized and self._auto_enable:
                self._validate_startup_feedback()
                self._start_endpos_loop()
                self._enabled = True
            else:
                self._control_output_enabled = False
                self._enabled = False
        except Exception:
            self._control_output_enabled = False
            self._endpos_ctrl._running = False
            try:
                self._robot.stop_control_loop()
                self._robot.disconnect()
            finally:
                self._connected = False
                self._enabled = False
            raise

    def shutdown(self, disable_after_safe_home: bool = True) -> None:
        if not self._connected:
            return
        with self._cmd_lock:
            self._shutdown_pending = True
            self._shutdown_thread = threading.get_ident()
            self._motion_generation += 1
        try:
            if self._enabled:
                if not disable_after_safe_home:
                    raise RuntimeError("shutdown refused: enabled arm requires safe_home before disconnect")
                self.safe_home()
                self.disable()
            with self._cmd_lock:
                self._robot.disconnect()
        except Exception as exc:
            # This arm drops without power. Failed homing must keep its control
            # loop and transport alive, not run an unconditional disable/finally.
            failures = []
            if self._enabled:
                for action in (self.stop_motion, self.stop_gravity_compensation, self.hold_current_position):
                    try:
                        action()
                    except Exception as hold_error:
                        failures.append(str(hold_error))
            self._shutdown_pending = False
            self.set_state_machine("SHUTDOWN_FAILED")
            raise RuntimeError(f"shutdown aborted: {exc}; hold failures={failures}") from exc
        self._connected = False
        self._enabled = False
        self._control_output_enabled = False
        self.set_state_machine("IDLE")

    def get_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self._get_arm_state()

    def _get_arm_state(
        self,
        request_feedback: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        pos, vel, torq = self._robot_get_state(request_feedback=request_feedback)
        n = len(self.joint_names)
        return (
            self._joint_mapping.positions_to_ros(pos[:n]),
            self._joint_mapping.velocities_to_ros(vel[:n]),
            self._joint_mapping.efforts_to_ros(torq[:n]),
        )

    def get_joint_positions(self, request: bool = False) -> np.ndarray:
        positions = self._arm_group.get_positions(request_feedback=request)
        return self._joint_mapping.positions_to_ros(positions)

    def get_joint_velocities(self, request: bool = False) -> np.ndarray:
        velocities = self._arm_group.get_velocities(request_feedback=request)
        return self._joint_mapping.velocities_to_ros(velocities)

    @_locked
    def hold_current_position(self) -> np.ndarray:
        current = self.get_joint_positions(request=True).copy()
        if self._state_machine != "SAFE_HOMING":
            self._endpos_ctrl._q_target[:] = current
            self._endpos_ctrl._qd_target[:] = 0.0
        return current

    @_locked
    def set_joint_position_target(self, positions, velocities=None, *, generation=None) -> None:
        self._require_motion_available()
        if generation is not None and generation != self._motion_generation:
            raise RuntimeError("trajectory interrupted by stop or homing")
        if self._shutdown_pending:
            raise RuntimeError("shutdown in progress")
        if self._state_machine == "SAFE_HOMING":
            raise RuntimeError("rejecting joint target during safe home")
        target = np.asarray(positions, dtype=np.float64).reshape(-1)
        if len(target) != len(self.joint_names):
            raise ValueError(
                f"expected {len(self.joint_names)} joint targets, got {len(target)}"
            )
        self._joint_mapping.validate_positions(target)
        target_velocity = (
            np.zeros(len(self.joint_names), dtype=np.float64)
            if velocities is None
            else self._joint_mapping.clamp_velocities(velocities)
        )
        self.stop_motion()
        self._endpos_ctrl._q_target[:] = target
        self._endpos_ctrl._qd_target[:] = target_velocity

    @property
    def joint_velocity_limits(self):
        return self._joint_mapping.ros_velocity_limits

    def validate_joint_positions(self, positions) -> None:
        self._joint_mapping.validate_positions(positions)

    def _require_motion_available(self):
        homing_thread = getattr(self, "_homing_thread", None)
        if homing_thread is not None and threading.get_ident() != homing_thread:
            raise RuntimeError("safe_home has not finished")
        if getattr(self, "_shutdown_pending", False) and threading.get_ident() != self._shutdown_thread:
            raise RuntimeError("shutdown in progress; new motion refused")
        if self._state_machine == "SHUTDOWN_FAILED":
            raise RuntimeError("shutdown fault; correct the fault and retry safe_home")

    @property
    def trajectory_generation(self):
        return self._motion_generation

    @_locked
    def start_endpos_control(self, *, allow_motor_enable: bool = False) -> None:
        self._require_motion_available()
        if not self._motion_authorized:
            raise RuntimeError(
                "motor output is locked until control.motion_authorized is true"
            )
        if not self._enabled and not allow_motor_enable:
            raise RuntimeError("arm is disabled; call /zekeep/enable before motion")
        if self._gravity_comp_active:
            raise RuntimeError("stop gravity compensation before starting endpos control")
        if self._state_machine in ("SAFE_HOMING", "TRAJ_RUNNING", "SERVO_RUNNING"):
            raise RuntimeError(f"rejecting endpos control in state {self._state_machine}")

        if self.control_loop_active:
            self.set_state_machine("IDLE")
            return

        self._start_endpos_loop()
        self._enabled = True
        self.set_state_machine("IDLE")

    def enable(self) -> None:
        self.start_endpos_control(allow_motor_enable=True)

    @_locked
    def disable(self) -> None:
        self._home_cancel.set()
        self._motion_generation += 1
        if self._gravity_comp_active:
            raise RuntimeError("stop gravity compensation before disable")
        self._control_output_enabled = False
        self.stop_motion()
        self._endpos_ctrl._running = False
        self._robot.stop_control_loop()
        self._reset_gravity_feedforward_state()
        self._robot.disable_all()
        self._enabled = False
        self.set_state_machine("IDLE")

    def stop_and_hold(self) -> None:
        """Interrupt homing without removing holding torque."""
        self._home_cancel.set()
        with self._cmd_lock:
            self._motion_generation += 1
            self.stop_motion()
            if self._gravity_comp_active:
                if not self._motion_authorized:
                    raise RuntimeError("position hold requires locally authorized motor output")
                self.stop_gravity_compensation()
            self.set_state_machine("IDLE")
            self.hold_current_position()
            if getattr(self, "_robot", None) is not None and self.has_gripper:
                self.stop_gripper_grasp()

    def safe_home(self) -> None:
        self._safe_home(close_gripper=True)

    def return_ready(self) -> None:
        if self._ready_pose_positions is None:
            raise RuntimeError("return_ready requires ready_pose.positions configuration")
        self._safe_home(close_gripper=False, positions=self._ready_pose_positions)

    def _safe_home(self, *, close_gripper: bool, positions=None) -> None:
        target_positions = self._safe_home_positions if positions is None else positions
        if target_positions is None:
            raise RuntimeError(
                "safe_home is disabled until a collision-checked pose is configured"
            )
        with self._cmd_lock:
            if positions is not None and self._state_machine != "IDLE":
                raise RuntimeError(
                    f"rejecting return_ready in state {self._state_machine}"
                )
            if self._homing_thread is not None:
                raise RuntimeError("safe_home already running")
            self._home_cancel.clear()
            self._motion_generation += 1
            self.stop_motion()
            self.set_state_machine("IDLE")
            if self._gravity_comp_active:
                if self._arm_control_mode == "mit" and self.control_loop_active:
                    hold_target = (
                        self._gravity_comp_q_last.copy()
                        if self._gravity_comp_q_last is not None
                        else self._arm_group.get_positions(request_feedback=False).copy()
                    )
                    self._arm_group._mit_kp = self._arm_mit_kp.copy()
                    self._arm_group._mit_kd = self._arm_mit_kd.copy()
                    self._endpos_ctrl._q_target[:] = hold_target
                    self._endpos_ctrl._qd_target[:] = 0.0
                    self._endpos_ctrl._running = True
                    self._control_output_enabled = True
                    self._reset_gravity_feedforward_state()
                    self._send_endpos_hold_once()
                    self._robot._ctrl_fn = self._endpos_loop_cb
                    self._gravity_comp_active = False
                    self._gravity_comp_q_last = None
                else:
                    self.stop_gravity_compensation()
            else:
                self.start_endpos_control()
            # Freeze the current pose before potentially slow scene checks.
            self.hold_current_position()
            self.set_state_machine("SAFE_HOMING")
            self._homing_thread = threading.get_ident()
        try:
            if close_gripper and self.has_gripper:
                self.set_gripper_position(_GRIPPER_CLOSED_POSITION)
            self._move_to_safe_home(target_positions)
        except Exception:
            self.stop_and_hold()
            raise
        finally:
            with self._cmd_lock:
                self._homing_thread = None
                self.set_state_machine("IDLE")

    def _move_to_safe_home(self, positions=None) -> None:
        target = np.asarray(
            self._safe_home_positions if positions is None else positions,
            dtype=np.float64,
        )
        self._joint_mapping.validate_positions(target)
        current = self.get_joint_positions(request=True)
        max_error = float(np.max(np.abs(target - current)))
        if self.validate_home_path is None:
            raise RuntimeError("safe_home requires MoveIt path collision validation")
        # Check the full direct joint path, at most 0.01 rad between samples.
        samples = np.linspace(current, target, max(2, int(np.ceil(max_error / 0.01)) + 1))
        self.validate_home_path(samples)
        if np.max(np.abs(self.get_joint_positions(request=True) - current)) > 0.01:
            raise RuntimeError("arm moved during safe_home path validation")
        duration = max(1.0, 2.0 * max_error / 0.5)
        steps = max(2, int(duration * 50.0))
        for ratio in np.linspace(0.0, 1.0, steps):
            with self._cmd_lock:
                if self._home_cancel.is_set() or not self._enabled:
                    raise RuntimeError("safe_home interrupted")
                blend = 10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5
                self._endpos_ctrl._q_target[:] = current + (target - current) * blend
                self._endpos_ctrl._qd_target[:] = 0.0
            time.sleep(duration / steps)
        deadline = time.monotonic() + 5.0
        settled = 0
        while time.monotonic() < deadline:
            if self._home_cancel.is_set() or not self._enabled:
                raise RuntimeError("safe_home interrupted")
            position = self.get_joint_positions(request=True)
            velocity = self.get_joint_velocities(request=True)
            valid = np.all(np.isfinite(position)) and np.all(np.isfinite(velocity))
            reached = valid and np.max(np.abs(position - target)) < 0.01 and np.max(np.abs(velocity)) < 0.03
            settled = settled + 1 if reached else 0
            if settled >= 3:
                return
            time.sleep(0.02)
        raise RuntimeError("safe_home target did not settle; motors remain enabled")

    @_locked
    def set_zero(self, joint_name: str = "") -> bool:
        self._require_motion_available()
        if self._state_machine == "SERVO_RUNNING":
            raise RuntimeError("rejecting set_zero while servo is running")
        self.stop_motion()
        self._robot.stop_control_loop()
        self._endpos_ctrl._running = False
        if joint_name:
            if joint_name not in self._robot._motor_map:
                raise KeyError(f"unknown joint: {joint_name}")
            self._set_zero_single(joint_name)
        else:
            self._robot.set_zero()
        self._enabled = False
        self.set_state_machine("IDLE")
        return True

    @_locked
    def send_joint_mit_cmd(
        self,
        joint_name: str,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        tau: float,
    ) -> None:
        if self._hybrid_arm_groups:
            raise RuntimeError(
                "raw arm MIT/POS_VEL commands are disabled during hybrid control"
            )
        if not self._allow_raw_arm_mit:
            raise RuntimeError(
                "raw arm MIT is disabled; use independent gravity compensation"
            )
        index = self._joint_index(joint_name)
        self._begin_lowlevel_streaming("mit")
        q = self.get_joint_positions(request=True)
        target_pos = np.array(q, dtype=np.float64, copy=True)
        target_vel = np.zeros(len(self.joint_names), dtype=np.float64)
        target_tau = np.zeros(len(self.joint_names), dtype=np.float64)
        target_kp = np.array(self._arm_mit_kp, dtype=np.float64, copy=True)
        target_kd = np.array(self._arm_mit_kd, dtype=np.float64, copy=True)
        target_pos[index] = float(pos)
        target_vel[index] = float(vel)
        target_kp[index] = float(kp)
        target_kd[index] = float(kd)
        target_tau[index] = float(tau)
        self._joint_mapping.validate_positions(target_pos)
        target_vel = self._joint_mapping.clamp_velocities(target_vel)
        self._arm_group.send_mit(
            self._joint_mapping.positions_to_motor(target_pos),
            vel=self._joint_mapping.velocities_to_motor(target_vel),
            kp=target_kp,
            kd=target_kd,
            tau=self._joint_mapping.efforts_to_motor(target_tau),
        )
        self.set_state_machine("LOWLEVEL_STREAMING")

    @_locked
    def send_joint_pos_vel_cmd(
        self,
        joint_name: str,
        pos: float,
        vlim: float,
    ) -> None:
        if self._hybrid_arm_groups:
            raise RuntimeError(
                "raw arm MIT/POS_VEL commands are disabled during hybrid control"
            )
        index = self._joint_index(joint_name)
        self._begin_lowlevel_streaming("pos_vel")
        q = self.get_joint_positions(request=True)
        target_pos = np.array(q, dtype=np.float64, copy=True)
        target_vlim = self._joint_mapping.ros_velocity_limits
        target_pos[index] = float(pos)
        target_vlim[index] = min(abs(float(vlim)), target_vlim[index])
        self._joint_mapping.validate_positions(target_pos)
        self._arm_group.send_pos_vel(
            self._joint_mapping.positions_to_motor(target_pos),
            vlim=np.abs(self._joint_mapping.velocities_to_motor(target_vlim)),
        )
        self.set_state_machine("LOWLEVEL_STREAMING")

    def current_pose(self):
        from .conversions import fk_to_pose

        q, _, _ = self.get_joint_state()
        q_padded = self._pad_q_for_model(self._gc_model, q, len(self.joint_names))
        position, rotation, _ = self._compute_fk(self._gc_model, q_padded)
        return fk_to_pose(position, rotation)

    def _require_idle(self, what: str) -> None:
        state = self._state_machine
        if self._shutdown_pending or state in ("TRAJ_RUNNING", "SERVO_RUNNING", "GRAVITY_COMP", "SAFE_HOMING", "SHUTDOWN_FAILED"):
            raise RuntimeError(f"rejecting {what} in state {state}")

    @_locked
    def begin_trajectory_stream(self) -> None:
        self._require_idle("trajectory stream")
        self.start_endpos_control()
        self.set_state_machine("TRAJ_RUNNING")
        return self._motion_generation

    @_locked
    def begin_servo_stream(self) -> None:
        self._require_idle("servo stream")
        self.start_endpos_control()
        self.set_state_machine("SERVO_RUNNING")

    @_locked
    def set_servo_target(self, positions, velocities) -> None:
        if self._state_machine != "SERVO_RUNNING":
            raise RuntimeError("servo command received while servo stream is inactive")
        target = np.asarray(positions, dtype=np.float64).reshape(-1)
        velocity = np.asarray(velocities, dtype=np.float64).reshape(-1)
        if target.size != len(self.joint_names) or velocity.size != len(self.joint_names):
            raise ValueError("servo command must contain all arm joints")
        if not np.all(np.isfinite(target)) or not np.all(np.isfinite(velocity)):
            raise ValueError("servo command contains non-finite values")
        self._joint_mapping.validate_positions(target)
        self._endpos_ctrl._q_target[:] = target
        self._endpos_ctrl._qd_target[:] = self._joint_mapping.clamp_velocities(velocity)

    @_locked
    def stop_servo_stream(self) -> None:
        if self._state_machine != "SERVO_RUNNING":
            return
        try:
            self._endpos_ctrl._q_target[:] = self.get_joint_positions(request=True)
        except Exception:
            pass  # Keep last accepted position as hold target if feedback is unavailable.
        self._endpos_ctrl._qd_target[:] = 0.0
        self.set_state_machine("IDLE")

    @property
    def planned_motion_duration(self) -> float:
        return float(self._endpos_ctrl._traj_duration)

    @property
    def planned_joint_target(self) -> np.ndarray:
        return np.asarray(self._endpos_ctrl._traj[-1], dtype=np.float64).copy()

    def move_to_pose_traj(
        self, x, y, z, roll, pitch, yaw, duration: float
    ) -> bool:
        self.begin_trajectory_stream()
        ok = False
        try:
            ok = bool(
                self._endpos_ctrl.move_to_traj(x, y, z, roll, pitch, yaw, duration)
            )
        finally:
            if not ok and self._state_machine == "TRAJ_RUNNING":
                self.set_state_machine("IDLE")
        return ok

    @_locked
    def move_to_pose_ik(self, x, y, z, roll, pitch, yaw) -> tuple[bool, list[float]]:
        self._require_idle("IK target")
        self.start_endpos_control()
        ok = self._endpos_ctrl.move_to_ik(x, y, z, roll, pitch, yaw)
        return bool(ok), [float(v) for v in self._endpos_ctrl._q_target]

    @_locked
    def solve_pose_ik(self, x, y, z, roll, pitch, yaw) -> tuple[bool, list[float]]:
        state = self._state_machine
        if self._shutdown_pending or state in (
            "TRAJ_RUNNING", "GRAVITY_COMP", "SAFE_HOMING", "SHUTDOWN_FAILED"
        ):
            raise RuntimeError(f"rejecting IK target in state {state}")
        if not self._enabled or not self.control_loop_active:
            raise RuntimeError("arm is disabled; call /zekeep/enable before motion")

        current = self.get_joint_positions(request=True)
        q_curr = self._pad_q_for_model(
            self._endpos_ctrl._model,
            current,
            len(self.joint_names),
        )
        target = self._pos_rot_to_se3(
            np.array([x, y, z]),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )
        result = self._solve_ik(
            self._endpos_ctrl._model,
            self._endpos_ctrl._data,
            self._endpos_ctrl._end_frame_id,
            target,
            q_curr,
            self._endpos_ctrl._ik_solver_params,
            controlled_joints=len(self.joint_names),
        )
        if not result.success:
            return False, []

        solution = np.asarray(result.q[: len(self.joint_names)], dtype=np.float64)
        self.validate_joint_positions(solution)
        return True, [float(v) for v in solution]

    def get_joint_status_codes(self) -> list[int]:
        codes: list[int] = []
        for name in self.joint_names:
            try:
                st = self._robot._motor_map[name].get_state()
                codes.append(int(st.status_code if st is not None else 0))
            except Exception:
                codes.append(0)
        return codes

    # ------------------------------------------------------------------
    # gravity compensation
    # ------------------------------------------------------------------

    @_locked
    def start_gravity_compensation(self) -> None:
        self._require_motion_available()
        if not self._gravity_compensation_enabled:
            raise RuntimeError("gravity compensation is disabled for this robot model")
        if not (
            self._motion_authorized or self._gravity_comp_trial_authorized
        ):
            raise RuntimeError("gravity compensation requires authorized motor output")
        if not (
            self._gravity_comp_dynamics_verified
            or self._gravity_comp_trial_authorized
        ):
            raise RuntimeError(
                "gravity compensation requires verified mass, center-of-mass, "
                "and inertia parameters"
            )
        if not self._gravity_comp_dynamics_source:
            raise RuntimeError("gravity compensation requires dynamics provenance")
        validate_dynamics_model(self._gc_model, self.joint_names)
        if self._state_machine in ("TRAJ_RUNNING", "SERVO_RUNNING", "SAFE_HOMING"):
            raise RuntimeError(
                f"rejecting gravity compensation in state {self._state_machine}"
            )
        self.stop_gravity_compensation()
        self.stop_motion()
        self._robot.stop_control_loop()
        self._endpos_ctrl._running = False
        self._reset_gravity_feedforward_state()

        self._gravity_comp_q_last = self.get_joint_positions(request=True).copy()
        self._gravity_comp_tau_last = np.zeros(len(self.joint_names), dtype=np.float64)
        if self._gravity_stop_assist_enabled:
            self._gravity_stop_assist_positions = self._gravity_comp_q_last[
                self._gravity_stop_assist_indices
            ].copy()
            self._gravity_stop_assist_active[:] = True
        self._robot.disable_all()
        time.sleep(0.1)
        mode_ok = self._arm_group.mode_mit(
            kp=self._gravity_comp_kp,
            kd=self._gravity_comp_kd,
        )
        if not mode_ok:
            self._control_output_enabled = False
            self._robot.stop_control_loop()
            raise RuntimeError("arm did not enter mit mode for gravity compensation")
        if self.has_gripper and not self._gripper_group.mode_mit(
            kp=np.zeros(1, dtype=np.float64),
            kd=np.zeros(1, dtype=np.float64),
        ):
            self._control_output_enabled = False
            self._robot.stop_control_loop()
            raise RuntimeError("gripper did not enter free MIT mode")
        self._robot.enable_all()
        self._enabled = True
        self._gravity_comp_active = True
        arm_rate = float(getattr(self._robot, "_rate", 500.0))
        self._gravity_comp_tick(self._robot, 1.0 / arm_rate)
        self._robot.start_control_loop(self._gravity_comp_tick, rate=arm_rate)
        self.set_state_machine("GRAVITY_COMP")

    @_locked
    def stop_gravity_compensation(self) -> None:
        if not self._gravity_comp_active:
            return
        hold_target = (
            self._gravity_comp_q_last.copy()
            if self._gravity_comp_q_last is not None
            else None
        )
        gravity_tau = (
            self._gravity_comp_tau_last.copy()
            if self._gravity_comp_tau_last is not None
            else None
        )
        handed_off = self._handoff_gravity_to_mit_hold(
            hold_target,
            gravity_tau,
        )
        if not handed_off:
            self._robot.stop_control_loop()
            self._control_output_enabled = False
        self._gravity_comp_active = False
        self._gravity_comp_q_last = None
        self._gravity_comp_tau_last = None
        self._gravity_stop_assist_positions = None
        self._gravity_stop_assist_active[:] = False
        if not handed_off:
            self._reset_gravity_feedforward_state()
            if self._enabled and self._motion_authorized:
                self._start_endpos_hold(target=hold_target)
            else:
                self._robot.disable_all()
                self._enabled = False
        self.set_state_machine("IDLE")

    def _handoff_gravity_to_mit_hold(
        self,
        hold_target: np.ndarray | None,
        gravity_tau: np.ndarray | None,
    ) -> bool:
        """Replace the active gravity callback with MIT hold without disabling."""
        if not (
            self._enabled
            and self._motion_authorized
            and self._arm_control_mode == "mit"
            and not self._hybrid_arm_groups
            and self.control_loop_active
            and hold_target is not None
        ):
            return False

        target = np.asarray(hold_target, dtype=np.float64)
        self._joint_mapping.validate_positions(target)
        self._arm_group._mit_kp = self._arm_mit_kp.copy()
        self._arm_group._mit_kd = self._arm_mit_kd.copy()
        self._endpos_ctrl._q_target[:] = target
        self._endpos_ctrl._qd_target[:] = 0.0
        self._gravity_feedforward_tau_last = (
            np.asarray(gravity_tau, dtype=np.float64).copy()
            if self._gravity_feedforward_enabled and gravity_tau is not None
            else np.zeros(len(self.joint_names), dtype=np.float64)
        )
        self._gravity_feedforward_q_last = target.copy()
        self._gravity_feedforward_faulted = False
        self._control_output_enabled = True

        # Send the first hold command while the gravity callback still owns the
        # active loop. Only after that command succeeds do we transfer ownership.
        self._send_endpos_hold_once()
        self._endpos_ctrl._running = True
        self._robot._ctrl_fn = self._endpos_loop_cb
        return True

    def gravity_compensation_active(self) -> bool:
        return self._gravity_comp_active

    @staticmethod
    def _angles_near_reference(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
        delta = values - reference
        delta = (delta + np.pi) % (2.0 * np.pi) - np.pi
        return reference + delta

    def _read_gravity_comp_positions(
        self,
        *,
        request: bool = False,
        reference: np.ndarray | None = None,
    ) -> np.ndarray:
        motor_q = self._arm_group.get_positions(request_feedback=request)
        q = self._joint_mapping.positions_to_ros(motor_q)
        ref = reference if reference is not None else self._gravity_comp_q_last
        if ref is not None:
            q = self._angles_near_reference(q, ref)
        self._gravity_comp_q_last = np.array(q, dtype=np.float64, copy=True)
        return self._gravity_comp_q_last.copy()

    def _gravity_comp_tick(self, _robot, dt: float) -> None:
        if not self._cmd_lock.acquire(blocking=False):
            return
        try:
            if not self._gravity_comp_active:
                return

            q = self._read_gravity_comp_positions(request=True)
            velocity = self.get_joint_velocities(request=False)
            q_model = self._pad_q_for_model(
                self._gc_model, q, len(self.joint_names)
            )
            tau_requested = self._compute_gravity_torque(q_model)
            tau_requested += self._compute_gravity_stop_assist(q, velocity)
            tau_ros = self._limit_gravity_comp_torque(
                tau_requested,
                dt,
            )
            tau_motor = self._joint_mapping.efforts_to_motor(tau_ros)
            motor_q = self._joint_mapping.positions_to_motor(q)

            self._arm_group.send_mit(
                motor_q,
                vel=np.zeros(len(self.joint_names), dtype=np.float64),
                kp=self._gravity_comp_kp,
                kd=self._gravity_comp_kd,
                tau=tau_motor,
            )
            if self.has_gripper:
                self._gripper_group.send_mit(
                    np.zeros(1, dtype=np.float64),
                    vel=np.zeros(1, dtype=np.float64),
                    kp=np.zeros(1, dtype=np.float64),
                    kd=np.zeros(1, dtype=np.float64),
                    tau=np.zeros(1, dtype=np.float64),
                )
        except Exception:
            self._control_output_enabled = False
            self._gravity_comp_active = False
            self._enabled = False
            self._robot.disable_all()
            self.set_state_machine("IDLE")
            raise
        finally:
            self._cmd_lock.release()

    def _compute_gravity_stop_assist(
        self,
        q: np.ndarray,
        velocity: np.ndarray,
    ) -> np.ndarray:
        assist = np.zeros(len(self.joint_names), dtype=np.float64)
        if not self._gravity_stop_assist_enabled:
            return assist

        indices = self._gravity_stop_assist_indices
        speeds = np.abs(np.asarray(velocity[indices], dtype=np.float64))
        positions = np.asarray(q[indices], dtype=np.float64)
        if self._gravity_stop_assist_positions is None:
            self._gravity_stop_assist_positions = positions.copy()
            self._gravity_stop_assist_active[:] = True

        released = speeds >= self._gravity_stop_assist_release_velocity
        self._gravity_stop_assist_active[released] = False

        tracking = ~self._gravity_stop_assist_active
        self._gravity_stop_assist_positions[tracking] = positions[tracking]
        engaged = tracking & (
            speeds <= self._gravity_stop_assist_engage_velocity
        )
        self._gravity_stop_assist_active[engaged] = True

        position_error = self._gravity_stop_assist_positions - positions
        joint_assist = np.clip(
            self._gravity_stop_assist_position_gain * position_error,
            -self._gravity_stop_assist_torque_limit,
            self._gravity_stop_assist_torque_limit,
        )
        assist[indices] = np.where(
            self._gravity_stop_assist_active,
            joint_assist,
            0.0,
        )
        return assist

    def _limit_gravity_comp_torque(
        self,
        tau_ros: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        self._gravity_comp_tau_last = self._limit_gravity_torque(
            tau_ros,
            dt,
            self._gravity_comp_tau_last,
        )
        return self._gravity_comp_tau_last.copy()

    def _reset_gravity_feedforward_state(self) -> None:
        self._gravity_feedforward_tau_last = np.zeros(
            len(self.joint_names),
            dtype=np.float64,
        )
        self._gravity_feedforward_q_last = None
        self._gravity_feedforward_faulted = False

    def _read_gravity_feedforward_positions(self) -> np.ndarray:
        q = self.get_joint_positions(request=False)
        reference = self._gravity_feedforward_q_last
        if reference is not None:
            q = self._angles_near_reference(q, reference)
        self._gravity_feedforward_q_last = np.array(q, dtype=np.float64, copy=True)
        return self._gravity_feedforward_q_last.copy()

    def _compute_gravity_feedforward(self, dt: float) -> np.ndarray:
        if not getattr(self, "_gravity_feedforward_enabled", False):
            return np.zeros(len(self.joint_names), dtype=np.float64)
        if getattr(self, "_gravity_feedforward_faulted", False):
            return np.zeros(len(self.joint_names), dtype=np.float64)

        try:
            q = self._read_gravity_feedforward_positions()
            q_model = self._pad_q_for_model(
                self._gc_model,
                q,
                len(self.joint_names),
            )
            tau_ros = self._limit_gravity_torque(
                self._compute_gravity_torque(q_model),
                dt,
                self._gravity_feedforward_tau_last,
            )
            self._gravity_feedforward_tau_last = tau_ros.copy()
            return tau_ros
        except Exception:
            self._gravity_feedforward_faulted = True
            self._gravity_feedforward_tau_last = np.zeros(
                len(self.joint_names),
                dtype=np.float64,
            )
            self._gravity_feedforward_q_last = None
            if "GRAVITY_FEEDFORWARD_FAULT" not in self._error_codes:
                self._error_codes.append("GRAVITY_FEEDFORWARD_FAULT")
            return np.zeros(len(self.joint_names), dtype=np.float64)

    def _compute_gravity_torque(self, q_model: np.ndarray) -> np.ndarray:
        tau_model = self._gc_compute_generalized_gravity(
            self._gc_model,
            q_model,
            self._gc_data,
        )[: len(self.joint_names)]
        tau_model = np.asarray(tau_model, dtype=np.float64)
        scale = np.where(
            tau_model > 0.0,
            self._gravity_comp_tau_scale_positive,
            self._gravity_comp_tau_scale,
        )
        return tau_model * scale

    def _limit_gravity_torque(
        self,
        tau_ros: np.ndarray,
        dt: float,
        previous: np.ndarray | None,
    ) -> np.ndarray:
        tau = np.asarray(tau_ros, dtype=np.float64).reshape(-1)
        if len(tau) != len(self.joint_names):
            raise ValueError(
                f"gravity torque must contain {len(self.joint_names)} values, "
                f"got {len(tau)}"
            )
        if not np.all(np.isfinite(tau)):
            raise ValueError("gravity torque must contain only finite values")

        limit = np.abs(self._gravity_comp_tau_limit)
        tau = np.clip(tau, -limit, limit)

        if previous is not None:
            dt_safe = max(float(dt), 0.0)
            max_delta = np.abs(self._gravity_comp_tau_rate_limit) * dt_safe
            delta = np.clip(tau - previous, -max_delta, max_delta)
            tau = previous + delta
        return np.array(tau, dtype=np.float64, copy=True)

    # ------------------------------------------------------------------
    # gripper
    # ------------------------------------------------------------------

    @_locked
    def set_gripper_target(self, position: float) -> None:
        self._begin_gripper_command(allow_endpos=True)
        target = float(position)
        self._gripper_mapping.validate_positions([target])
        self._gripper_grasp_state = None
        self._endpos_ctrl.set_gripper_target(target)
        self._gripper_group.send_mit(
            self._gripper_mapping.positions_to_motor([target]),
            kp=getattr(self._gripper_group, "_mit_kp"),
            kd=getattr(self._gripper_group, "_mit_kd"),
        )
        self._gripper_target_position = target

    def wait_gripper_target(self, timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.gripper_reached_target():
                return True
            time.sleep(0.02)
        return False

    def set_gripper_position(
        self,
        position: float,
        timeout: float = 3.0,
    ) -> tuple[bool, float]:
        self.set_gripper_target(position)
        reached = self.wait_gripper_target(timeout)
        return reached, self.get_gripper_state()[0]

    def get_gripper_state(self) -> tuple[float, float, float, int]:
        if not self.has_gripper or not self._gripper_name:
            return 0.0, 0.0, 0.0, 0
        pos = float(self._gripper_group.get_positions()[0])
        vel = float(self._gripper_group.get_velocities(request_feedback=False)[0])
        status = 0
        torque = 0.0
        try:
            st = self._robot._motor_map[self._gripper_name].get_state()
            if st is not None:
                torque = float(st.torq)
                status = int(st.status_code)
        except Exception:
            status = 0
        return (
            float(self._gripper_mapping.positions_to_ros([pos])[0]),
            float(self._gripper_mapping.velocities_to_ros([vel])[0]),
            float(self._gripper_mapping.efforts_to_ros([torque])[0]),
            status,
        )

    def gripper_reached_target(self) -> bool:
        if self._gripper_target_position is None:
            return True
        pos = self.get_gripper_state()[0]
        return abs(pos - self._gripper_target_position) < _GRIPPER_GOAL_TOLERANCE_RAD

    @_locked
    def start_gripper_grasp(
        self,
        closing_torque: float = 0.0,
        hold_torque: float = 0.0,
    ) -> None:
        self._begin_gripper_command(allow_endpos=True)
        state = GripperGraspState(self._gripper_grasp_config)
        state.start(
            self.get_gripper_state()[0],
            closing_torque=closing_torque,
            hold_torque=hold_torque,
        )
        self._gripper_grasp_state = state
        self._gripper_target_position = None

    @_locked
    def get_gripper_grasp_status(self) -> tuple[bool, bool, str, float, float, float]:
        state = self._gripper_grasp_state
        position, velocity, torque, _ = self.get_gripper_state()
        if state is None:
            return True, False, "idle", position, velocity, torque
        # Status polling must also advance contact detection. The real-time
        # control callback deliberately skips a cycle when the command lock is
        # busy, so relying on that callback alone can leave a physically
        # closed gripper reported as "closing" until the action times out.
        state.update(
            position=position,
            velocity=velocity,
            torque=torque,
        )
        return (
            state.done,
            state.object_detected,
            state.state,
            position,
            velocity,
            torque,
        )

    @_locked
    def stop_gripper_grasp(self) -> None:
        state = self._gripper_grasp_state
        self._gripper_grasp_state = None
        try:
            position = self.get_gripper_state()[0]
        except Exception:
            if state is None:
                raise
            position = state.contact_position
        self._endpos_ctrl.set_gripper_target(position)
        self._gripper_target_position = position

    @_locked
    def send_gripper_mit_cmd(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        tau: float,
    ) -> None:
        self._begin_gripper_command()
        self._begin_gripper_lowlevel("mit")
        self._gripper_mapping.validate_positions([pos])
        motor_velocity = self._gripper_mapping.velocities_to_motor(
            self._gripper_mapping.clamp_velocities([vel])
        )
        self._gripper_group.send_mit(
            self._gripper_mapping.positions_to_motor([pos]),
            vel=motor_velocity,
            kp=np.array([float(kp)], dtype=np.float64),
            kd=np.array([float(kd)], dtype=np.float64),
            tau=self._gripper_mapping.efforts_to_motor([tau]),
        )
        self._gripper_target_position = None

    @_locked
    def send_gripper_pos_vel_cmd(self, pos: float, vlim: float) -> None:
        self._begin_gripper_command()
        self._begin_gripper_lowlevel("pos_vel")
        self._gripper_mapping.validate_positions([pos])
        ros_vlim = min(
            abs(float(vlim)),
            float(self._gripper_mapping.ros_velocity_limits[0]),
        )
        self._gripper_group.send_pos_vel(
            self._gripper_mapping.positions_to_motor([pos]),
            vlim=np.abs(self._gripper_mapping.velocities_to_motor([ros_vlim])),
        )
        self._gripper_target_position = None

    def _begin_gripper_command(self, *, allow_endpos: bool = False) -> None:
        self._require_motion_available()
        if not self._enabled:
            raise RuntimeError("rejecting gripper command while arm is disabled")
        if self._gravity_comp_active or self.state_machine == "GRAVITY_COMP":
            raise RuntimeError("rejecting gripper command during gravity compensation")
        if (
            self.state_machine == "SAFE_HOMING"
            and threading.get_ident() != self._homing_thread
        ):
            raise RuntimeError("rejecting gripper command during safe home")
        if self.state_machine in ("TRAJ_RUNNING", "SERVO_RUNNING"):
            raise RuntimeError("rejecting gripper command while trajectory is running")
        if not self.has_gripper or not self._gripper_name:
            raise RuntimeError("gripper is not initialized")
        if not allow_endpos and self.control_loop_active:
            self.stop_motion()
            self._robot.stop_control_loop()
            self._endpos_ctrl._running = False

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _validate_startup_feedback(self) -> None:
        deadline = time.monotonic() + 2.0
        missing: list[str] = []
        while True:
            self._robot_get_state(request_feedback=True)
            missing = []
            for name in self._robot.joint_names:
                state = self._robot._motor_map[name].get_state()
                if state is None or int(state.status_code) != 0:
                    missing.append(name)
            if not missing or time.monotonic() >= deadline:
                break
            time.sleep(0.05)
        if missing:
            raise RuntimeError(
                "startup feedback invalid for motors: " + ", ".join(missing)
            )

        arm_positions = self._joint_mapping.positions_to_ros(
            self._arm_group.get_positions(request_feedback=False)
        )
        self._joint_mapping.validate_positions(arm_positions)
        if self.has_gripper:
            gripper_position = self._gripper_mapping.positions_to_ros(
                self._gripper_group.get_positions(request_feedback=False)
            )
            self._gripper_mapping.validate_positions(gripper_position)

    def _begin_lowlevel_streaming(self, required_mode: str) -> None:
        self._require_motion_available()
        if not self._enabled:
            raise RuntimeError("rejecting low-level command while arm is disabled")
        if self._gravity_comp_active or self.state_machine == "GRAVITY_COMP":
            raise RuntimeError("rejecting low-level command during gravity compensation")
        if self.state_machine == "SAFE_HOMING":
            raise RuntimeError("rejecting low-level command during safe home")
        if self.state_machine == "SERVO_RUNNING":
            raise RuntimeError("rejecting low-level command while servo is running")
        if self.state_machine == "TRAJ_RUNNING":
            self._motion_generation += 1
            self.stop_motion()
        self._robot.stop_control_loop()
        self._endpos_ctrl._running = False
        self._reset_gravity_feedforward_state()
        if required_mode != self.mode:
            self._enter_mode(
                self._arm_group,
                required_mode,
                "arm",
                kp=self._arm_mit_kp,
                kd=self._arm_mit_kd,
            )
        self.set_state_machine("LOWLEVEL_STREAMING")

    def _begin_gripper_lowlevel(self, required_mode: str) -> None:
        self._enter_mode(self._gripper_group, required_mode, "gripper")
        self.set_state_machine("LOWLEVEL_STREAMING")

    @staticmethod
    def _enter_mode(group, required_mode: str, label: str, **mit_gains) -> None:
        if required_mode == "mit":
            ok = group.mode_mit(**mit_gains)
        elif required_mode == "pos_vel":
            ok = group.mode_pos_vel()
        else:
            raise ValueError(f"unsupported low-level mode: {required_mode}")
        if not ok:
            raise RuntimeError(f"{label} did not enter {required_mode} mode")

    def _start_endpos_hold(self, target: np.ndarray | None = None) -> None:
        if self.control_loop_active:
            return
        self._start_endpos_loop(target)

    def _start_endpos_loop(self, target: np.ndarray | None = None) -> None:
        self._configure_groups_for_endpos()
        self._reset_gravity_feedforward_state()
        if target is None:
            self.hold_current_position()
        else:
            self._endpos_ctrl._q_target[:] = np.asarray(target, dtype=np.float64)
            self._endpos_ctrl._qd_target[:] = 0.0
        self._control_output_enabled = True
        self._send_endpos_hold_once()
        self._robot.start_control_loop(self._endpos_loop_cb)
        self._endpos_ctrl._running = True

    def _configure_groups_for_endpos(self) -> None:
        if self.control_loop_active:
            self._robot.stop_control_loop()
        if self._endpos_modes_ready():
            return
        if self._hybrid_arm_groups:
            # Configure every motor independently: JointGroup mode setters are
            # deliberately group-wide, so the normal arm group cannot be used.
            motor_vlim = np.abs(
                self._joint_mapping.velocities_to_motor(self._arm_pos_vel_vlim)
            )
            for index, name in enumerate(self.joint_names):
                group = self._hybrid_arm_groups[name]
                if self._gravity_feedforward_mask[index]:
                    ok = group.mode_mit(
                        kp=np.array([self._arm_mit_kp[index]]),
                        kd=np.array([self._arm_mit_kd[index]]),
                    )
                else:
                    ok = group.mode_pos_vel(vlim=np.array([motor_vlim[index]]))
                if not ok:
                    mode = "mit" if self._gravity_feedforward_mask[index] else "pos_vel"
                    raise RuntimeError(f"{name} did not enter {mode} mode")
            for group in self._hybrid_arm_groups.values():
                group.enable()
        else:
            if self._arm_control_mode == "mit":
                ok = self._arm_group.mode_mit(
                    kp=self._arm_mit_kp,
                    kd=self._arm_mit_kd,
                )
            else:
                motor_vlim = np.abs(
                    self._joint_mapping.velocities_to_motor(self._arm_pos_vel_vlim)
                )
                ok = self._arm_group.mode_pos_vel(vlim=motor_vlim)
            if not ok:
                mode = "mit" if self._arm_control_mode == "mit" else "pos_vel"
                raise RuntimeError(f"arm did not enter {mode} mode")
            self._arm_group.enable()
        if self.has_gripper:
            if not self._gripper_group.mode_mit():
                raise RuntimeError("gripper did not enter mit mode")
            self._gripper_group.enable()

    def _endpos_modes_ready(self) -> bool:
        if not self._enabled:
            return False
        if self._hybrid_arm_groups:
            for index, name in enumerate(self.joint_names):
                expected = "mit" if self._gravity_feedforward_mask[index] else "pos_vel"
                if self._hybrid_arm_groups[name].mode != expected:
                    return False
        else:
            expected = "mit" if self._arm_control_mode == "mit" else "pos_vel"
            if self._arm_group.mode != expected:
                return False
        return not self.has_gripper or self._gripper_group.mode == "mit"

    @_locked
    def stop_motion(self) -> None:
        self._endpos_ctrl._stop_send.set()
        if self._endpos_ctrl._send_thread is not None:
            self._endpos_ctrl._send_thread.join(timeout=5.0)
        self._endpos_ctrl._moving = False
        self._endpos_ctrl._stop_send.clear()

    def motion_active(self) -> bool:
        return bool(self._endpos_ctrl._moving)

    def _endpos_loop_cb(self, robot, dt: float) -> None:
        del robot
        if not self._cmd_lock.acquire(blocking=False):
            return
        try:
            if not self._control_output_enabled:
                return
            self._send_endpos_hold_once(dt)
            if self.has_gripper:
                if self._gripper_grasp_state is None:
                    self._send_gripper_hold_once()
                else:
                    self._send_gripper_grasp_once()
        finally:
            self._cmd_lock.release()

    def _send_gripper_hold_once(self) -> None:
        motor_target = self._gripper_mapping.positions_to_motor(
            [self._endpos_ctrl._gripper_target]
        )
        self._gripper_group.send_mit(
            motor_target,
            kp=getattr(self._gripper_group, "_mit_kp"),
            kd=getattr(self._gripper_group, "_mit_kd"),
        )

    def _send_gripper_grasp_once(self) -> None:
        position, velocity, torque, _ = self.get_gripper_state()
        command = self._gripper_grasp_state.update(
            position=position,
            velocity=velocity,
            torque=torque,
        )
        self._gripper_group.send_mit(
            self._gripper_mapping.positions_to_motor([command.position]),
            vel=self._gripper_mapping.velocities_to_motor([command.velocity]),
            kp=np.array([command.kp], dtype=np.float64),
            kd=np.array([command.kd], dtype=np.float64),
            tau=self._gripper_mapping.efforts_to_motor([command.torque]),
        )

    def _send_endpos_hold_once(self, dt: float = 0.0) -> None:
        motor_target = self._joint_mapping.positions_to_motor(
            self._endpos_ctrl._q_target
        )
        ros_velocity = self._joint_mapping.clamp_velocities(
            self._endpos_ctrl._qd_target
        )
        motor_velocity = self._joint_mapping.velocities_to_motor(ros_velocity)
        if self._hybrid_arm_groups:
            tau_ros = np.asarray(self._compute_gravity_feedforward(dt), dtype=np.float64)
            tau_ros = np.where(self._gravity_feedforward_mask, tau_ros, 0.0)
            motor_tau = self._joint_mapping.efforts_to_motor(tau_ros)
            motor_vlim = np.minimum(
                np.abs(self._joint_mapping.velocities_to_motor(
                    self._arm_pos_vel_vlim
                )),
                self._joint_mapping.motor_velocity_limits,
            )
            for index, name in enumerate(self.joint_names):
                group = self._hybrid_arm_groups[name]
                if self._gravity_feedforward_mask[index]:
                    group.send_mit(
                        np.array([motor_target[index]]),
                        vel=np.array([motor_velocity[index]]),
                        kp=np.array([self._arm_mit_kp[index]]),
                        kd=np.array([self._arm_mit_kd[index]]),
                        tau=np.array([motor_tau[index]]),
                    )
                else:
                    group.send_pos_vel(
                        np.array([motor_target[index]]),
                        vlim=np.array([motor_vlim[index]]),
                    )
        elif self._arm_control_mode == "mit":
            tau_ros = self._compute_gravity_feedforward(dt)
            self._arm_group.send_mit(
                motor_target,
                vel=motor_velocity,
                kp=getattr(self._arm_group, "_mit_kp"),
                kd=getattr(self._arm_group, "_mit_kd"),
                tau=self._joint_mapping.efforts_to_motor(tau_ros),
            )
        else:
            motor_vlim = np.minimum(
                np.abs(self._joint_mapping.velocities_to_motor(
                    self._arm_pos_vel_vlim
                )),
                self._joint_mapping.motor_velocity_limits,
            )
            self._arm_group.send_pos_vel(
                motor_target,
                vlim=motor_vlim,
            )

    def _joint_index(self, joint_name: str) -> int:
        try:
            return self.joint_names.index(joint_name)
        except ValueError as exc:
            raise KeyError(f"unknown joint: {joint_name}") from exc

    def _set_zero_single(self, joint_name: str) -> None:
        self._robot.disable_all()
        time.sleep(0.3)
        motor = self._robot._motor_map[joint_name]
        ctrl = None
        for joint in self._robot._all_joints:
            if joint.name == joint_name:
                ctrl = self._robot._ctrl_map[str(joint.vendor)]
                break
        if ctrl is None:
            raise KeyError(f"unknown joint: {joint_name}")
        for _ in range(200):
            try:
                motor.request_feedback()
                ctrl.poll_feedback_once()
            except Exception:
                pass
            st = motor.get_state()
            if st is not None and st.status_code == 0:
                break
            time.sleep(0.05)
        motor.set_zero_position()
