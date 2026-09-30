"""Stateful ROS-only facade for hand-guided teaching and replay."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import threading
import time
from typing import Any, Callable

import numpy as np

from .session import (
    TeachingSession,
    load_teaching_session,
    retime_path,
    save_teaching_session,
    simplify_path,
)


class TeachState(Enum):
    """Lifecycle states used to prevent unsafe teaching transitions."""
    DISCONNECTED = "DISCONNECTED"
    HOLD = "HOLD"
    GUIDING = "GUIDING"
    POINT_REPLAY = "POINT_REPLAY"
    PATH_REPLAY = "PATH_REPLAY"
    SAFE_EXIT = "SAFE_EXIT"
    DISABLED = "DISABLED"


@dataclass(frozen=True)
class TeachConfig:
    """Validated runtime limits for capture, replay, and settling."""
    stationary_velocity_rad_s: float = 0.02
    duplicate_point_distance_rad: float = 0.01
    path_minimum_distance_rad: float = 0.002
    replay_max_velocity_rad_s: float = 0.10
    minimum_segment_s: float = 0.05
    settle_position_tolerance_rad: float = 0.005
    waypoint_position_tolerance_rad: float = 0.005
    settle_velocity_tolerance_rad_s: float = 0.02
    settle_required_samples: int = 3
    settle_sample_interval_s: float = 0.05
    settle_timeout_s: float = 8.0


class RosTeachRobot:
    """Coordinate teaching state with a ROS controller client."""
    def __init__(
        self,
        client: Any,
        config: TeachConfig,
        *,
        clock: Callable[[], float] = time.monotonic,
        session: TeachingSession | None = None,
    ) -> None:
        self._client = client
        self.config = config
        self._clock = clock
        self.session = TeachingSession() if session is None else session
        self.state = TeachState.DISCONNECTED
        self._session_started_monotonic = 0.0
        self._continuous_recording = False
        self._gravity_compensation_active = False
        self._last_path_sequence: int | None = None
        self._lock = threading.RLock()
        self._cancel_replay = threading.Event()

    @property
    def continuous_recording(self) -> bool:
        """Whether continuous path capture is currently enabled."""
        return self._continuous_recording

    @property
    def has_recorded_path(self) -> bool:
        """Whether the session contains a continuous path."""
        with self._lock:
            return bool(self.session.path)

    def start(self) -> None:
        """Connect to the controller and enter the stationary hold state."""
        with self._lock:
            self._require_state(TeachState.DISCONNECTED)
            self._client.start()
            self._session_started_monotonic = float(self._clock())
            self.state = TeachState.HOLD

    def enter_guiding(self) -> None:
        """Enable gravity compensation and enter hand-guiding mode."""
        with self._lock:
            self._require_state(TeachState.HOLD)
            self._client.start_gravity_compensation()
            self._gravity_compensation_active = True
            self.state = TeachState.GUIDING

    def capture_point(self) -> bool:
        """Capture the current stationary joint sample as a discrete point."""
        with self._lock:
            self._require_state(TeachState.GUIDING)
            sample = self._client.latest_sample()
            peak_velocity = float(np.max(np.abs(sample.velocities)))
            if peak_velocity > self.config.stationary_velocity_rad_s:
                raise RuntimeError(
                    "robot is moving; hold it stationary before capturing a point"
                )
            return self.session.add_point(
                sample.positions,
                captured_s=self._elapsed_s(),
                minimum_distance_rad=self.config.duplicate_point_distance_rad,
            )

    def undo_point(self) -> bool:
        """Remove the most recent discrete point while guiding."""
        with self._lock:
            self._require_state(TeachState.GUIDING)
            if not self.session.points:
                return False
            self.session.points.pop()
            return True

    def toggle_continuous_recording(self, *, overwrite: bool = False) -> bool:
        """Toggle continuous capture, requiring confirmation before replacement."""
        with self._lock:
            self._require_state(TeachState.GUIDING)
            if self._continuous_recording:
                self._continuous_recording = False
                return False
            if self.session.path and not overwrite:
                raise RuntimeError(
                    "existing continuous path requires overwrite confirmation"
                )
            if self.session.path:
                self.session.path.clear()
            self._last_path_sequence = None
            self._continuous_recording = True
            return True

    def sample_continuous(self) -> bool:
        """Capture one new feedback sequence when continuous capture is active."""
        with self._lock:
            if self.state is not TeachState.GUIDING or not self._continuous_recording:
                return False
            sample = self._client.latest_sample()
            sequence = int(sample.sequence)
            if sequence == self._last_path_sequence:
                return False
            self.session.add_path_sample(
                sample.positions,
                captured_s=self._elapsed_s(),
            )
            self._last_path_sequence = sequence
            return True

    def save(self, path: str | Path) -> Path:
        """Persist the current teaching session to JSON."""
        with self._lock:
            return save_teaching_session(self.session, path)

    def load(self, path: str | Path) -> None:
        """Load a session while disconnected or holding the robot."""
        with self._lock:
            if self.state not in (TeachState.DISCONNECTED, TeachState.HOLD):
                raise RuntimeError("teaching sessions can only be loaded while holding")
            self.session = load_teaching_session(path)
            self._last_path_sequence = None

    def replay_points(self) -> None:
        """Replay discrete points sequentially with settle checks."""
        with self._lock:
            if not self.session.points:
                raise RuntimeError("no discrete teaching points have been recorded")
        self._prepare_replay(TeachState.POINT_REPLAY)
        try:
            for point_index, sample in enumerate(tuple(self.session.points)):
                if self._cancel_replay.is_set():
                    raise RuntimeError("point replay cancelled")
                current = np.asarray(
                    self._client.latest_sample().positions,
                    dtype=np.float64,
                )
                duration = self._segment_duration_s(current, sample.positions)
                self._client.execute_direct_joint_target(
                    sample.positions,
                    duration_s=duration,
                    cancel_event=self._cancel_replay,
                )
                position_tolerance = (
                    self.config.settle_position_tolerance_rad
                    if point_index == 0
                    else self.config.waypoint_position_tolerance_rad
                )
                self._wait_for_settle(
                    sample.positions,
                    position_tolerance_rad=position_tolerance,
                )
        finally:
            self._finish_replay(TeachState.POINT_REPLAY)

    def replay_path(self) -> None:
        """Simplify, retime, and replay the recorded continuous path."""
        with self._lock:
            simplified = simplify_path(
                tuple(self.session.path),
                minimum_distance_rad=self.config.path_minimum_distance_rad,
            )
        if len(simplified) < 2:
            raise RuntimeError("continuous replay requires at least two path samples")
        retimed = retime_path(
            simplified,
            max_velocity_rad_s=self.config.replay_max_velocity_rad_s,
            minimum_segment_s=self.config.minimum_segment_s,
        )
        self._prepare_replay(TeachState.PATH_REPLAY)
        try:
            current = np.asarray(
                self._client.latest_sample().positions,
                dtype=np.float64,
            )
            first = simplified[0].positions
            self._client.execute_direct_joint_target(
                first,
                duration_s=self._segment_duration_s(current, first),
                cancel_event=self._cancel_replay,
            )
            self._wait_for_settle(first)
            if self._cancel_replay.is_set():
                raise RuntimeError("continuous replay cancelled")
            self._client.execute_joint_trajectory(
                retimed,
                cancel_event=self._cancel_replay,
            )
            self._wait_for_settle(
                simplified[-1].positions,
                position_tolerance_rad=(
                    self.config.waypoint_position_tolerance_rad
                ),
            )
        finally:
            self._finish_replay(TeachState.PATH_REPLAY)

    def cancel_replay(self) -> None:
        """Request cancellation of the active replay action."""
        with self._lock:
            self._cancel_replay.set()
            action_active = self.state in (
                TeachState.POINT_REPLAY,
                TeachState.PATH_REPLAY,
            )
        if action_active:
            self._client.cancel_active()

    def safe_exit(self) -> None:
        """Stop guiding, home the arm, disable motors, and close the client."""
        with self._lock:
            if self.state in (TeachState.POINT_REPLAY, TeachState.PATH_REPLAY):
                raise RuntimeError("replay is still active; wait for cancellation")
            if self.state is TeachState.DISABLED:
                return
            if self.state not in (TeachState.HOLD, TeachState.GUIDING):
                raise RuntimeError(
                    f"safe exit is not available from {self.state.value}"
                )
            gravity_compensation_active = self._gravity_compensation_active
            self._continuous_recording = False
            self.state = TeachState.SAFE_EXIT
        try:
            if gravity_compensation_active:
                self._client.stop_gravity_compensation()
                with self._lock:
                    self._gravity_compensation_active = False
            self._settle_at_current(cancelable=False)
            self._client.safe_home()
            self._settle_at_current(cancelable=False)
            self._client.disable()
        except BaseException:
            with self._lock:
                self.state = TeachState.HOLD
            raise
        with self._lock:
            self.state = TeachState.DISABLED
        self._client.close()

    def _prepare_replay(self, replay_state: TeachState) -> None:
        with self._lock:
            if self.state not in (TeachState.HOLD, TeachState.GUIDING):
                raise RuntimeError(
                    f"replay requires HOLD or GUIDING; current state is {self.state.value}"
                )
            if self._continuous_recording:
                raise RuntimeError(
                    "stop continuous recording before starting replay"
                )
            gravity_compensation_active = self._gravity_compensation_active
            self._continuous_recording = False
            self.state = replay_state
        try:
            if self._cancel_replay.is_set():
                raise RuntimeError("replay cancelled before preparation")
            if gravity_compensation_active:
                self._client.stop_gravity_compensation()
                with self._lock:
                    self._gravity_compensation_active = False
            if self._cancel_replay.is_set():
                raise RuntimeError("replay cancelled during preparation")
            self._wait_until_stationary(cancelable=True)
        except BaseException:
            with self._lock:
                self.state = TeachState.HOLD
                self._cancel_replay.clear()
            raise

    def _finish_replay(self, replay_state: TeachState) -> None:
        with self._lock:
            if self.state is replay_state:
                self.state = TeachState.HOLD
            self._cancel_replay.clear()

    def _settle_at_current(self, *, cancelable: bool) -> None:
        sample = self._client.latest_sample()
        self._wait_for_settle(sample.positions, cancelable=cancelable)

    def _wait_until_stationary(self, *, cancelable: bool) -> None:
        self._client.wait_until_stationary(
            velocity_tolerance_rad_s=self.config.settle_velocity_tolerance_rad_s,
            required_samples=self.config.settle_required_samples,
            sample_interval_s=self.config.settle_sample_interval_s,
            timeout_s=self.config.settle_timeout_s,
            cancel_event=self._cancel_replay if cancelable else None,
        )

    def _wait_for_settle(
        self,
        target_positions: np.ndarray,
        *,
        position_tolerance_rad: float | None = None,
        cancelable: bool = True,
    ) -> None:
        tolerance = (
            self.config.settle_position_tolerance_rad
            if position_tolerance_rad is None
            else float(position_tolerance_rad)
        )
        self._client.wait_for_settle(
            target_positions,
            position_tolerance_rad=tolerance,
            velocity_tolerance_rad_s=self.config.settle_velocity_tolerance_rad_s,
            required_samples=self.config.settle_required_samples,
            sample_interval_s=self.config.settle_sample_interval_s,
            timeout_s=self.config.settle_timeout_s,
            cancel_event=self._cancel_replay if cancelable else None,
        )

    def _segment_duration_s(
        self,
        start: np.ndarray,
        target: np.ndarray,
    ) -> float:
        displacement = float(
            np.max(
                np.abs(
                    np.asarray(target, dtype=np.float64)
                    - np.asarray(start, dtype=np.float64)
                )
            )
        )
        return max(
            self.config.minimum_segment_s,
            1.5 * displacement / self.config.replay_max_velocity_rad_s,
        )

    def _elapsed_s(self) -> float:
        elapsed = float(self._clock()) - self._session_started_monotonic
        if elapsed < 0.0:
            raise RuntimeError("monotonic clock moved backwards")
        return elapsed

    def _require_state(self, expected: TeachState) -> None:
        if self.state is not expected:
            raise RuntimeError(
                f"command requires {expected.value}; current state is {self.state.value}"
            )
