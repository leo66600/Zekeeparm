"""Small grasp-side helper for Zekeep visual grasping.

The SDK owns arm connection, mode switching, Cartesian planning, gravity
compensation, and the control loop. This module provides only the extra
gripper and pose helpers used by the vision workflows.

selected_arm_config(): read the SDK hardware YAML and choose controller mode.

GraspDriver:
  start(): start SDK control and attach gripper tick handling.
  open_gripper(): open to a requested jaw distance.
  grasp(): close with force control and report object contact.
  release_gripper(): open and return the gripper to closed rest.
  get_gripper_state(): return cached position, velocity, and torque.
  get_tcp_pose(): return the current TCP pose as a 4x4 matrix.
"""

from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from types import MethodType
from typing import Any, Optional

import numpy as np


_CAMERAWS_ROOT = Path(__file__).resolve().parents[2]
_REBOT_REPO_NAME = "zekeeparm_SDK"
_DEFAULT_REBOT_REPO = _CAMERAWS_ROOT.parents[1] / _REBOT_REPO_NAME

GRIPPER_MAX_DISTANCE_M = 0.075
GRIPPER_OPEN_POSITION_RAD = 1.5


def gripper_distance_to_motor_position(
    distance_m: float,
    *,
    max_distance_m: float = GRIPPER_MAX_DISTANCE_M,
    open_position_rad: float = GRIPPER_OPEN_POSITION_RAD,
    close_position_rad: float = 0.0,
) -> float:
    max_distance = max(float(max_distance_m), 0.0)
    if max_distance == 0.0:
        return float(close_position_rad)
    distance = float(np.clip(distance_m, 0.0, max_distance))
    ratio = distance / max_distance
    return float(
        close_position_rad
        + (float(open_position_rad) - float(close_position_rad)) * ratio
    )


@dataclass(frozen=True)
class SelectedArmConfig:
    arm_type: str
    controller_mode: str


@dataclass(frozen=True)
class ArmJointMapping:
    names: tuple[str, ...]
    directions: np.ndarray
    encoder_zeros: np.ndarray
    model_zeros: np.ndarray
    lower_limits: np.ndarray
    upper_limits: np.ndarray

    @classmethod
    def from_config(
        cls,
        config: Optional[dict],
        joint_names: list[str],
    ) -> "ArmJointMapping":
        if not isinstance(config, dict):
            raise ValueError(
                "robot.joint_mapping is required for direct SDK control"
            )

        directions = []
        encoder_zeros = []
        model_zeros = []
        lower_limits = []
        upper_limits = []
        for name in joint_names:
            values = config.get(name)
            if not isinstance(values, dict):
                raise ValueError(f"robot.joint_mapping missing {name}")
            direction = float(values.get("direction", 0.0))
            if direction not in (-1.0, 1.0):
                raise ValueError(f"{name} direction must be -1 or 1")
            ratio = float(values.get("gear_ratio", 1.0))
            if not np.isclose(ratio, 1.0):
                raise ValueError(
                    f"{name} gear_ratio={ratio:g} is unsupported by SDK MIT gains"
                )
            directions.append(direction)
            encoder_zeros.append(float(values.get("encoder_zero", 0.0)))
            model_zeros.append(float(values.get("model_zero", 0.0)))
            lower_limits.append(float(values.get("lower", -np.inf)))
            upper_limits.append(float(values.get("upper", np.inf)))

        mapping = cls(
            names=tuple(joint_names),
            directions=np.asarray(directions, dtype=np.float64),
            encoder_zeros=np.asarray(encoder_zeros, dtype=np.float64),
            model_zeros=np.asarray(model_zeros, dtype=np.float64),
            lower_limits=np.asarray(lower_limits, dtype=np.float64),
            upper_limits=np.asarray(upper_limits, dtype=np.float64),
        )
        if np.any(mapping.lower_limits > mapping.upper_limits):
            raise ValueError("robot.joint_mapping contains invalid limits")
        return mapping

    def motor_to_model_positions(self, values: np.ndarray) -> np.ndarray:
        motor = self._vector(values, "motor positions")
        return self.directions * (motor - self.encoder_zeros) + self.model_zeros

    def model_to_motor_positions(self, values: np.ndarray) -> np.ndarray:
        model = self._vector(values, "model positions")
        if np.any(model < self.lower_limits) or np.any(model > self.upper_limits):
            bad = np.flatnonzero(
                (model < self.lower_limits) | (model > self.upper_limits)
            )[0]
            raise ValueError(
                f"{self.names[int(bad)]} target {model[bad]:.4f} outside "
                f"[{self.lower_limits[bad]:.4f}, {self.upper_limits[bad]:.4f}]"
            )
        return (
            (model - self.model_zeros) / self.directions
            + self.encoder_zeros
        )

    def motor_to_model_vectors(self, values: np.ndarray) -> np.ndarray:
        return self.directions * self._vector(values, "motor vectors")

    def model_to_motor_vectors(self, values: np.ndarray) -> np.ndarray:
        return self._vector(values, "model vectors") / self.directions

    def _vector(self, values: np.ndarray, label: str) -> np.ndarray:
        vector = np.asarray(values, dtype=np.float64).reshape(-1)
        if vector.size != len(self.names) or not np.all(np.isfinite(vector)):
            raise ValueError(
                f"{label} must contain {len(self.names)} finite values"
            )
        return vector


def install_arm_joint_mapping(
    arm: Any,
    arm_group: Any,
    config: Optional[dict],
) -> ArmJointMapping:
    """Map SDK motor coordinates to the URDF/model joint convention."""
    joint_names = [joint.name for joint in arm_group._jcfgs]
    mapping = ArmJointMapping.from_config(config, joint_names)
    if getattr(arm, "_rebot_joint_mapping_installed", False):
        raise RuntimeError("arm joint mapping is already installed")

    raw_arm_get_state = arm.get_state
    raw_group_get_positions = arm_group.get_positions
    raw_group_get_velocities = arm_group.get_velocities
    raw_group_send_pos_vel = arm_group.send_pos_vel
    raw_group_send_mit = arm_group.send_mit
    raw_group_send_vel = arm_group.send_vel
    count = len(joint_names)

    def mapped_arm_get_state(_arm, request_feedback: bool = True):
        pos, vel, effort = raw_arm_get_state(request_feedback=request_feedback)
        pos = np.asarray(pos, dtype=np.float64).copy()
        vel = np.asarray(vel, dtype=np.float64).copy()
        effort = np.asarray(effort, dtype=np.float64).copy()
        pos[:count] = mapping.motor_to_model_positions(pos[:count])
        vel[:count] = mapping.motor_to_model_vectors(vel[:count])
        effort[:count] = mapping.motor_to_model_vectors(effort[:count])
        return pos, vel, effort

    def mapped_group_get_positions(_group, request_feedback: bool = True):
        return mapping.motor_to_model_positions(
            raw_group_get_positions(request_feedback=request_feedback)
        )

    def mapped_group_get_velocities(_group, request_feedback: bool = True):
        return mapping.motor_to_model_vectors(
            raw_group_get_velocities(request_feedback=request_feedback)
        )

    def mapped_group_send_pos_vel(_group, pos, vlim=None):
        motor_pos = mapping.model_to_motor_positions(pos)
        motor_vlim = None if vlim is None else np.abs(
            mapping.model_to_motor_vectors(vlim)
        )
        return raw_group_send_pos_vel(motor_pos, vlim=motor_vlim)

    def mapped_group_send_mit(
        _group,
        pos,
        vel=None,
        kp=None,
        kd=None,
        tau=None,
    ):
        return raw_group_send_mit(
            mapping.model_to_motor_positions(pos),
            vel=None if vel is None else mapping.model_to_motor_vectors(vel),
            kp=kp,
            kd=kd,
            tau=None if tau is None else mapping.model_to_motor_vectors(tau),
        )

    def mapped_group_send_vel(_group, vel):
        return raw_group_send_vel(mapping.model_to_motor_vectors(vel))

    arm.get_state = MethodType(mapped_arm_get_state, arm)
    arm_group.get_positions = MethodType(
        mapped_group_get_positions,
        arm_group,
    )
    arm_group.get_velocities = MethodType(
        mapped_group_get_velocities,
        arm_group,
    )
    arm_group.send_pos_vel = MethodType(mapped_group_send_pos_vel, arm_group)
    arm_group.send_mit = MethodType(mapped_group_send_mit, arm_group)
    arm_group.send_vel = MethodType(mapped_group_send_vel, arm_group)
    arm._rebot_joint_mapping_installed = True
    return mapping


def _is_rebot_repo_root(path: Path) -> bool:
    return (
        (path / "config" / "rebotarm_dm.yaml").is_file()
        and (path / _REBOT_REPO_NAME / "actuator" / "rebotarm.py").is_file()
    )


def find_rebot_repo_root(hint: Optional[str] = None) -> Path:
    repo = Path(hint).expanduser() if hint else _DEFAULT_REBOT_REPO
    if not repo.is_absolute():
        repo = (_CAMERAWS_ROOT / repo).resolve()
    else:
        repo = repo.resolve()
    for candidate in (repo, repo.parent):
        if _is_rebot_repo_root(candidate):
            return candidate
    raise FileNotFoundError(f"zekeeparm_SDK repo not found: {repo}")


def ensure_rebot_sdk_in_syspath(hint: Optional[str] = None) -> Path:
    repo = find_rebot_repo_root(hint)
    repo_str = str(repo)
    if repo_str not in sys.path:
        sys.path.insert(0, repo_str)
    return repo


def selected_hardware_yaml(repo_root: Optional[str] = None) -> Path:
    repo = find_rebot_repo_root(repo_root)
    hw_path = (repo / "config" / "rebotarm_dm.yaml").resolve()
    if not hw_path.is_file():
        raise FileNotFoundError(f"Hardware config not found: {hw_path}")
    return hw_path


def selected_arm_config(
    repo_root: Optional[str] = None,
    *,
    controller_mode: Optional[str] = None,
) -> SelectedArmConfig:
    """Return the selected arm type and SDK controller mode.

    The DM profile supplies the safe default, while callers such as the
    direct SDK route may explicitly select ``mit`` or ``posvel``.
    """
    selected_hardware_yaml(repo_root)
    mode = "posvel" if controller_mode is None else str(controller_mode).lower()
    if mode not in ("mit", "posvel"):
        raise ValueError("controller_mode must be 'mit' or 'posvel'")
    return SelectedArmConfig(arm_type="dm", controller_mode=mode)


class GraspDriver:
    MAX_DISTANCE_M = GRIPPER_MAX_DISTANCE_M
    _STATE_IDLE = "idle"
    _STATE_POSITION = "position"
    _STATE_CLOSING = "closing"
    _STATE_HOLDING = "holding"

    def __init__(
        self,
        arm: Any,
        controller: Any,
        gripper_config: Optional[dict] = None,
        joint_mapping_config: Optional[dict] = None,
        repo_root: Optional[str] = None,
        arm_control_mode: Optional[str] = None,
    ) -> None:
        self._arm = arm
        self._controller = controller
        self._arm_group = arm.groups.get("arm")
        self._gripper_group = arm.groups.get("gripper")
        if self._arm_group is None:
            raise ValueError("Hardware config missing groups.arm")
        if self._gripper_group is None or not arm.has_gripper:
            raise ValueError("Hardware config missing groups.gripper")
        gripper_jcfgs = getattr(self._gripper_group, "_jcfgs", [])
        if not gripper_jcfgs:
            raise ValueError("groups.gripper has no joints")
        self._gripper_name = gripper_jcfgs[0].name
        self._gripper_motor: Any = None

        from zekeeparm_SDK.kinematics import compute_fk, load_robot_model, pad_q_for_model

        self._compute_fk = compute_fk
        self._pad_q_for_model = pad_q_for_model
        self._model = load_robot_model()
        self._n = self._arm_group.num_joints
        self._joint_mapping = install_arm_joint_mapping(
            arm,
            self._arm_group,
            joint_mapping_config,
        )

        selected = selected_arm_config(
            repo_root,
            controller_mode=arm_control_mode,
        )
        defaults = {
            "angle_open": abs(GRIPPER_OPEN_POSITION_RAD),
            "counterclockwise": False,
            "tau_max": 1.5,
            "close_torque": 1.0,
            "default_force": 0.30,
            "contact_torque": 0.10,
        }
        gcfg = {**defaults, **((gripper_config or {}).get(selected.arm_type) or {})}
        motion_sign = 1.0 if bool(gcfg.get("counterclockwise")) else -1.0
        self._angle_open = -motion_sign * abs(float(gcfg["angle_open"]))
        self._tau_max = abs(float(gcfg["tau_max"]))
        self._open_sign = 1.0 if self._angle_open >= 0.0 else -1.0
        self._close_sign = motion_sign
        self._close_torque = self._close_sign * abs(float(gcfg["close_torque"]))
        self._default_force = self._close_sign * abs(float(gcfg["default_force"]))
        self._contact_torque = abs(float(gcfg.get("contact_torque", 0.10)))
        if not np.isfinite(self._contact_torque) or self._contact_torque < 0.10:
            raise ValueError("gripper contact_torque must be finite and at least 0.10")
        self._max_distance_m = float(
            (gripper_config or {}).get("max_distance_m", self.MAX_DISTANCE_M)
        )
        if not np.isfinite(self._max_distance_m) or self._max_distance_m <= 0.0:
            raise ValueError("gripper max_distance_m must be finite and positive")
        # angle_open is already the measured safe limit below the mechanical
        # stop, so a full-width command should reach that configured limit.
        self._open_soft_limit = self._angle_open
        self._open_lo = min(self._open_soft_limit, 0.0)
        self._open_hi = max(self._open_soft_limit, 0.0)
        self._hard_stop_angle = self._open_sign * 0.05
        # The measured open stop settles slightly below the commanded 1.30 rad
        # target; 0.15 rad accepts that physical stop without masking a real
        # opening failure.
        self._arrive_tol = 0.15
        self._kp_move = 5.0
        self._kd_move = 1.0
        self._kd_close = 0.5
        self._stall_vel = 0.05
        self._startup_dist = 0.30
        self._state_lock = threading.Lock()
        self._state = self._STATE_IDLE
        self._target_pos = 0.0
        self._start_pos = 0.0
        self._contact_pos = 0.0
        self._hold_torque = self._default_force
        self._position_reached = True
        self._grasp_result: Optional[bool] = None
        self._last_gripper_state: Optional[tuple[float, float, float]] = None

    def start(self) -> None:
        """Start the SDK arm controller and let this driver own the gripper."""
        if getattr(self._controller, "_running", False):
            return

        self._controller._has_gripper = False
        self._arm.connect()
        q_now = self.read_stable_arm_positions()
        self._gripper_motor = self._gripper_group._mm[self._gripper_name]
        if self._arm_group:
            if self._controller._arm_control_mode == "mit":
                self._arm_group.mode_mit(
                    kp=self._arm_group._mit_kp,
                    kd=self._arm_group._mit_kd,
                )
            else:
                self._arm_group.mode_pos_vel()
            self._arm_group.enable()

        self._gripper_group.mode_mit()
        self._gripper_group.enable()
        self._prime_arm_target(q_now)
        self._prime_gripper_state()
        self._arm.start_control_loop(self._loop_cb)
        self._controller._running = True

    def _loop_cb(self, r: Any, dt: float) -> None:
        self._controller._loop_cb(r, dt)
        self.gripper_tick(dt)

    def _ensure_running(self) -> None:
        if not getattr(self._controller, "_running", False):
            raise RuntimeError("GraspDriver is not started; call grasp_driver.start() first")

    def _send_gripper_mit(
        self,
        pos: float,
        vel: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        tau: float = 0.0,
    ) -> None:
        pos_cmd = float(np.clip(pos, self._open_lo, self._open_hi))
        tau_cmd = float(np.clip(tau, -self._tau_max, self._tau_max))
        self._gripper_group.send_mit(
            np.array([pos_cmd], dtype=np.float64),
            vel=np.array([vel], dtype=np.float64),
            kp=np.array([kp], dtype=np.float64),
            kd=np.array([kd], dtype=np.float64),
            tau=np.array([tau_cmd], dtype=np.float64),
        )

    def read_stable_arm_positions(
        self,
        timeout: float = 3.0,
        min_samples: int = 5,
        max_span_rad: float = 0.01,
        sample_interval: float = 0.02,
    ) -> np.ndarray:
        """Read all arm joints and reject missing or unstable SDK feedback."""
        deadline = time.monotonic() + timeout
        samples: list[np.ndarray] = []
        missing = list(self._joint_mapping.names)
        max_span = float("inf")

        while time.monotonic() < deadline:
            q_now = self._arm.get_state(request_feedback=True)[0][: self._n]
            missing = []
            for name in self._joint_mapping.names:
                motor = self._arm._motor_map.get(name)
                state = None if motor is None else motor.get_state()
                # Damiao reports 0 while disabled and 1 while enabled.
                if state is None or int(state.status_code) not in (0, 1):
                    missing.append(name)
            if missing:
                samples.clear()
            else:
                samples.append(np.asarray(q_now, dtype=np.float64).copy())
                samples = samples[-max(1, int(min_samples)):]
                if len(samples) >= min_samples:
                    stacked = np.stack(samples)
                    max_span = float(np.max(np.ptp(stacked, axis=0)))
                    if max_span <= max_span_rad:
                        return np.median(stacked, axis=0)
            time.sleep(max(0.0, sample_interval))

        if missing:
            raise RuntimeError(
                "Arm feedback incomplete; missing: " + ", ".join(missing)
            )
        raise RuntimeError(
            f"Arm feedback unstable; max span={max_span:.4f}rad"
        )

    def _prime_arm_target(self, q_now: Optional[np.ndarray] = None) -> None:
        if q_now is None:
            q_now = self.read_stable_arm_positions()
        self._controller._q_target[:] = q_now
        self._controller._qd_target[:] = 0.0

    def _prime_gripper_state(self, timeout: float = 1.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._gripper_group._request_feedback()
            state = self._read_gripper_state_cached()
            if state is not None:
                with self._state_lock:
                    self._target_pos = state[0]
                    self._contact_pos = state[0]
                    self._state = self._STATE_IDLE
                    self._position_reached = True
                    self._grasp_result = None
                return
            time.sleep(0.02)

    def _read_gripper_state_cached(self) -> Optional[tuple[float, float, float]]:
        if self._gripper_motor is None:
            return self._last_gripper_state
        st = self._gripper_motor.get_state()
        if st is None:
            return self._last_gripper_state
        self._last_gripper_state = (float(st.pos), float(st.vel), float(st.torq))
        return self._last_gripper_state

    def _wait_gripper_state(
        self,
        timeout: float = 1.0,
        cancel_event: Optional[threading.Event] = None,
    ) -> tuple[float, float, float]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cancel_event is not None and cancel_event.is_set():
                raise RuntimeError("gripper command cancelled: control client disconnected")
            state = self._read_gripper_state_cached()
            if state is not None:
                return state
            time.sleep(0.02)
        raise RuntimeError("Gripper feedback is not ready")

    def get_gripper_state(self) -> tuple[float, float, float]:
        state = self._read_gripper_state_cached()
        if state is None:
            raise RuntimeError("Gripper feedback is not ready")
        return state

    def _set_position_target(self, target: float) -> None:
        with self._state_lock:
            self._target_pos = float(target)
            self._state = self._STATE_POSITION
            self._position_reached = False
            self._grasp_result = None

    def _wait_until(
        self,
        predicate,
        timeout: float,
        cancel_event: Optional[threading.Event] = None,
    ) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cancel_event is not None and cancel_event.is_set():
                raise RuntimeError("gripper command cancelled: control client disconnected")
            if predicate():
                return True
            time.sleep(0.01)
        if cancel_event is not None and cancel_event.is_set():
            raise RuntimeError("gripper command cancelled: control client disconnected")
        return predicate()

    def _position_done(self) -> bool:
        with self._state_lock:
            return self._position_reached

    def _grasp_done(self) -> bool:
        with self._state_lock:
            return self._grasp_result is not None

    def gripper_tick(self, dt: float = 0.0) -> None:
        del dt
        pos_vel_torq = self._read_gripper_state_cached()
        with self._state_lock:
            state = self._state
            target = self._target_pos
            command: Optional[tuple[float, float, float, float, float]] = None

            if state == self._STATE_POSITION:
                command = (target, 0.0, self._kp_move, self._kd_move, 0.0)
                if pos_vel_torq is not None and abs(pos_vel_torq[0] - target) < self._arrive_tol:
                    self._position_reached = True

            elif state == self._STATE_CLOSING:
                command = (0.0, 0.0, 0.0, self._kd_close, self._close_torque)
                if pos_vel_torq is not None:
                    pos, vel, torque = pos_vel_torq
                    self._contact_pos = pos
                    moved = abs(pos - self._start_pos) >= self._startup_dist
                    at_hard_stop = self._open_sign * pos <= self._open_sign * self._hard_stop_angle
                    if moved and at_hard_stop:
                        self._target_pos = 0.0
                        self._state = self._STATE_POSITION
                        self._position_reached = False
                        self._grasp_result = False
                        command = (0.0, 0.0, self._kp_move, self._kd_move, 0.0)
                    elif (
                        moved
                        and abs(vel) < self._stall_vel
                        and abs(torque) >= self._contact_torque
                    ):
                        self._target_pos = pos
                        self._state = self._STATE_HOLDING
                        self._grasp_result = True
                        command = (pos, 0.0, self._kp_move, self._kd_move, self._hold_torque)

            elif state == self._STATE_HOLDING:
                command = (self._target_pos, 0.0, self._kp_move, self._kd_move, self._hold_torque)

        if command is not None:
            pos, vel, kp, kd, tau = command
            self._send_gripper_mit(pos, vel=vel, kp=kp, kd=kd, tau=tau)

    def open_gripper(
        self,
        distance_m: float = GRIPPER_MAX_DISTANCE_M,
        timeout: float = 3.0,
        cancel_event: Optional[threading.Event] = None,
    ) -> None:
        self._ensure_running()
        raw_target = gripper_distance_to_motor_position(
            distance_m,
            max_distance_m=self._max_distance_m,
            open_position_rad=self._angle_open,
        )
        target = float(np.clip(raw_target, self._open_lo, self._open_hi))

        self._set_position_target(target)
        if not self._wait_until(
            self._position_done,
            timeout,
            cancel_event=cancel_event,
        ):
            state = self._read_gripper_state_cached()
            feedback = "unavailable" if state is None else f"pos={state[0]:.4f} vel={state[1]:.4f} tau={state[2]:.4f}"
            raise RuntimeError(
                f"Gripper did not reach open target within {timeout:.2f}s "
                f"target={target:.4f} feedback={feedback}"
            )

    def grasp(
        self,
        force: Optional[float] = None,
        timeout: float = 5.0,
        cancel_event: Optional[threading.Event] = None,
    ) -> bool:
        self._ensure_running()
        start_pos, _, _ = self._wait_gripper_state(cancel_event=cancel_event)
        hold_torque = self._close_sign * float(
            np.clip(abs(force if force is not None else self._default_force), 0.05, self._tau_max)
        )
        with self._state_lock:
            self._start_pos = start_pos
            self._contact_pos = start_pos
            self._target_pos = 0.0
            self._hold_torque = hold_torque
            self._state = self._STATE_CLOSING
            self._position_reached = False
            self._grasp_result = None

        if not self._wait_until(
            self._grasp_done,
            timeout,
            cancel_event=cancel_event,
        ):
            with self._state_lock:
                if self._grasp_result is None:
                    self._target_pos = self._contact_pos
                    self._state = self._STATE_POSITION
                    self._position_reached = False
                    self._grasp_result = False

        with self._state_lock:
            return bool(self._grasp_result)

    def release_gripper(
        self,
        timeout: float = 4.0,
        cancel_event: Optional[threading.Event] = None,
    ) -> None:
        self._ensure_running()
        self.open_gripper(
            timeout=min(2.0, timeout),
            cancel_event=cancel_event,
        )
        self._set_position_target(0.0)
        if not self._wait_until(
            self._position_done,
            timeout,
            cancel_event=cancel_event,
        ):
            raise RuntimeError(
                f"Gripper did not return to rest within {timeout:.2f}s"
            )

    def get_tcp_pose(self, q_arm: Optional[np.ndarray] = None) -> np.ndarray:
        if q_arm is None:
            refresh = not getattr(self._controller, "_running", False)
            q_arm = self._arm.get_state(request_feedback=refresh)[0][: self._n]
        else:
            q_arm = np.asarray(q_arm, dtype=np.float64)[: self._n]
        q = self._pad_q_for_model(self._model, q_arm, self._n)
        pos, rot, _ = self._compute_fk(self._model, q)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = rot
        T[:3, 3] = pos
        return T
