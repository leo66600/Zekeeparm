"""Minimal ROS client required by hand-guided teaching and replay."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Sequence

import numpy as np

from .constants import ARM_JOINT_NAMES
from .session import RetimedWaypoint


def _joint_state_qos() -> Any:
    from rclpy.qos import qos_profile_sensor_data

    return qos_profile_sensor_data


@dataclass(frozen=True)
class JointSample:
    """Ordered joint feedback snapshot with freshness metadata."""
    positions: np.ndarray
    velocities: np.ndarray
    received_monotonic: float
    sequence: int


class JointStateCache:
    """Thread-safe cache that normalizes incoming joint-state ordering."""
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._sample: JointSample | None = None
        self._sequence = 0

    def update(
        self,
        names: Sequence[str],
        positions: Sequence[float],
        velocities: Sequence[float],
        *,
        received_monotonic: float,
    ) -> None:
        position_by_name = dict(zip(names, positions))
        velocity_by_name = dict(zip(names, velocities))
        if any(name not in position_by_name for name in ARM_JOINT_NAMES):
            return
        if any(name not in velocity_by_name for name in ARM_JOINT_NAMES):
            return
        q = np.asarray(
            [position_by_name[name] for name in ARM_JOINT_NAMES],
            dtype=np.float64,
        )
        dq = np.asarray(
            [velocity_by_name[name] for name in ARM_JOINT_NAMES],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)):
            return
        with self._condition:
            self._sequence += 1
            self._sample = JointSample(
                positions=q.copy(),
                velocities=dq.copy(),
                received_monotonic=float(received_monotonic),
                sequence=self._sequence,
            )
            self._condition.notify_all()

    def wait(self, timeout_s: float) -> JointSample:
        """Wait until the first complete joint-state sample is available."""
        deadline = time.monotonic() + float(timeout_s)
        with self._condition:
            while self._sample is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("joint_states not received")
                self._condition.wait(timeout=remaining)
            return self._sample

    def latest(self, *, max_age_s: float, now: float | None = None) -> JointSample:
        """Return fresh feedback or raise when feedback is missing/stale."""
        current = time.monotonic() if now is None else float(now)
        with self._condition:
            sample = self._sample
        if sample is None:
            raise RuntimeError("joint_states not received")
        age = current - sample.received_monotonic
        if age < 0.0 or age > float(max_age_s):
            raise RuntimeError(
                f"joint_states stale: age={age:.3f}s "
                f"limit={float(max_age_s):.3f}s"
            )
        return sample


class TeachRosClient:
    """ROS-only facade; the controller remains the sole motor-port owner."""

    def __init__(
        self,
        *,
        namespace: str,
        joint_state_topic: str,
        timeout_s: float,
        joint_state_max_age_s: float,
        safe_home_timeout_s: float = 35.0,
    ) -> None:
        import rclpy
        from action_msgs.msg import GoalStatus
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_srvs.srv import Trigger
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        self._rclpy = rclpy
        self._GoalStatus = GoalStatus
        self._Duration = Duration
        self._FollowJointTrajectory = FollowJointTrajectory
        self._JointTrajectory = JointTrajectory
        self._JointTrajectoryPoint = JointTrajectoryPoint
        self._Trigger = Trigger
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()

        self._timeout_s = self._positive(timeout_s, "ROS timeout")
        self._safe_home_timeout_s = self._positive(
            safe_home_timeout_s,
            "safe-home timeout",
        )
        self._joint_state_max_age_s = self._positive(
            joint_state_max_age_s,
            "joint-state maximum age",
        )
        self._cache = JointStateCache()
        self._active_goal = None
        self._cancel_requested_goal = None
        self._action_starting = False
        self._active_goal_lock = threading.Lock()

        ns = str(namespace).strip("/")
        self._node = Node(f"zekeep_teach_client_{int(time.time() * 1000) % 100000}")
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name="zekeep-teach-ros",
            daemon=True,
        )
        self._node.create_subscription(
            JointState,
            str(joint_state_topic),
            self._joint_state_cb,
            _joint_state_qos(),
        )
        self._follow_joint_trajectory_client = ActionClient(
            self._node,
            FollowJointTrajectory,
            f"/{ns}/follow_joint_trajectory",
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

    @staticmethod
    def _positive(value: float, label: str) -> float:
        number = float(value)
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f"{label} must be finite and positive")
        return number

    def start(self) -> None:
        """Start ROS spinning and wait for required actions and services."""
        self._spin_thread.start()
        self._cache.wait(self._timeout_s)
        if not self._follow_joint_trajectory_client.wait_for_server(
            timeout_sec=self._timeout_s
        ):
            raise TimeoutError("follow_joint_trajectory action unavailable")
        services = (
            (self._gc_start_client, "start gravity compensation"),
            (self._gc_stop_client, "stop gravity compensation"),
            (self._safe_home_client, "safe home"),
            (self._disable_client, "disable"),
        )
        for client, label in services:
            if not client.wait_for_service(timeout_sec=self._timeout_s):
                raise TimeoutError(f"{label} service not available")

    def _joint_state_cb(self, message: Any) -> None:
        self._cache.update(
            message.name,
            message.position,
            message.velocity,
            received_monotonic=time.monotonic(),
        )

    def latest_sample(self) -> JointSample:
        """Return the latest fresh normalized joint feedback."""
        return self._cache.latest(max_age_s=self._joint_state_max_age_s)

    def wait_until_stationary(
        self,
        *,
        velocity_tolerance_rad_s: float,
        required_samples: int,
        sample_interval_s: float,
        timeout_s: float,
        cancel_event: threading.Event | None = None,
    ) -> None:
        """Wait for consecutive low-velocity feedback samples."""
        self._wait_for_feedback(
            target_positions=None,
            position_tolerance_rad=0.0,
            velocity_tolerance_rad_s=velocity_tolerance_rad_s,
            required_samples=required_samples,
            sample_interval_s=sample_interval_s,
            timeout_s=timeout_s,
            cancel_event=cancel_event,
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
        """Wait for consecutive samples within position and velocity limits."""
        target = np.asarray(target_positions, dtype=np.float64).reshape(-1)
        if target.size != len(ARM_JOINT_NAMES) or not np.all(np.isfinite(target)):
            raise ValueError("settle target must contain six finite joint positions")
        self._wait_for_feedback(
            target_positions=target,
            position_tolerance_rad=position_tolerance_rad,
            velocity_tolerance_rad_s=velocity_tolerance_rad_s,
            required_samples=required_samples,
            sample_interval_s=sample_interval_s,
            timeout_s=timeout_s,
            cancel_event=cancel_event,
        )

    def _wait_for_feedback(
        self,
        *,
        target_positions: np.ndarray | None,
        position_tolerance_rad: float,
        velocity_tolerance_rad_s: float,
        required_samples: int,
        sample_interval_s: float,
        timeout_s: float,
        cancel_event: threading.Event | None,
    ) -> None:
        if int(required_samples) <= 0:
            raise ValueError("required settle samples must be positive")
        deadline = time.monotonic() + float(timeout_s)
        consecutive = 0
        last_sequence = -1
        last_sample = None
        while time.monotonic() < deadline:
            if cancel_event is not None and cancel_event.is_set():
                raise RuntimeError("feedback wait cancelled")
            sample = self.latest_sample()
            if sample.sequence == last_sequence:
                time.sleep(max(float(sample_interval_s), 0.001))
                continue
            last_sequence = sample.sequence
            last_sample = sample
            velocity_ok = bool(
                np.max(np.abs(sample.velocities))
                <= float(velocity_tolerance_rad_s)
            )
            position_ok = target_positions is None or bool(
                np.max(np.abs(sample.positions - target_positions))
                <= float(position_tolerance_rad)
            )
            consecutive = consecutive + 1 if velocity_ok and position_ok else 0
            if consecutive >= int(required_samples):
                return
            time.sleep(max(float(sample_interval_s), 0.0))
        if last_sample is None:
            raise TimeoutError("robot joint feedback did not settle: no fresh feedback")
        peak_velocity = float(np.max(np.abs(last_sample.velocities)))
        if target_positions is None:
            worst_index = int(np.argmax(np.abs(last_sample.velocities)))
            raise TimeoutError(
                "robot joint feedback did not settle: "
                f"worst_joint={ARM_JOINT_NAMES[worst_index]} "
                "position_error=n/a "
                f"peak_velocity={peak_velocity:.6f}rad/s "
                f"velocity_tolerance={float(velocity_tolerance_rad_s):.6f}rad/s"
            )
        position_errors = np.abs(last_sample.positions - target_positions)
        worst_index = int(np.argmax(position_errors))
        raise TimeoutError(
            "robot joint feedback did not settle: "
            f"worst_joint={ARM_JOINT_NAMES[worst_index]} "
            f"position_error={float(position_errors[worst_index]):.6f}rad "
            f"position_tolerance={float(position_tolerance_rad):.6f}rad "
            f"peak_velocity={peak_velocity:.6f}rad/s "
            f"velocity_tolerance={float(velocity_tolerance_rad_s):.6f}rad/s"
        )

    def execute_direct_joint_target(
        self,
        target_positions: np.ndarray,
        *,
        duration_s: float,
        cancel_event: threading.Event | None = None,
    ) -> None:
        """Execute a two-point trajectory from measured state to target."""
        target = np.asarray(target_positions, dtype=np.float64).reshape(-1)
        duration = self._positive(duration_s, "direct joint duration")
        if target.size != len(ARM_JOINT_NAMES) or not np.all(np.isfinite(target)):
            raise ValueError("direct joint target must contain six finite positions")
        start = np.asarray(self.latest_sample().positions, dtype=np.float64)
        waypoints = ((0.0, start, np.zeros(6)), (duration, target, np.zeros(6)))
        trajectory = self._trajectory_message(waypoints)
        self._execute_trajectory_message(
            trajectory,
            timeout_s=duration + 5.0,
            cancel_event=cancel_event,
            label="direct joint trajectory",
        )

    def execute_joint_trajectory(
        self,
        waypoints: Sequence[RetimedWaypoint],
        *,
        cancel_event: threading.Event | None = None,
    ) -> None:
        """Execute a validated sequence of retimed joint waypoints."""
        if len(waypoints) < 2:
            raise ValueError("joint trajectory requires at least two waypoints")
        values = [
            (
                waypoint.time_from_start_s,
                waypoint.positions,
                waypoint.velocities,
            )
            for waypoint in waypoints
        ]
        trajectory = self._trajectory_message(values)
        self._execute_trajectory_message(
            trajectory,
            timeout_s=float(waypoints[-1].time_from_start_s) + 5.0,
            cancel_event=cancel_event,
            label="joint trajectory",
        )

    def _trajectory_message(
        self,
        waypoints: Sequence[tuple[float, Any, Any]],
    ) -> Any:
        trajectory = self._JointTrajectory()
        trajectory.joint_names = list(ARM_JOINT_NAMES)
        previous_time = -1.0
        for index, (timestamp_value, positions_value, velocities_value) in enumerate(waypoints):
            timestamp = float(timestamp_value)
            positions = np.asarray(positions_value, dtype=np.float64).reshape(-1)
            velocities = np.asarray(velocities_value, dtype=np.float64).reshape(-1)
            valid = (
                positions.size == len(ARM_JOINT_NAMES)
                and velocities.size == len(ARM_JOINT_NAMES)
                and np.all(np.isfinite(positions))
                and np.all(np.isfinite(velocities))
                and math.isfinite(timestamp)
                and timestamp >= 0.0
                and (index == 0 or timestamp > previous_time)
            )
            if not valid:
                raise ValueError("joint trajectory contains an invalid waypoint")
            point = self._JointTrajectoryPoint()
            point.positions = [float(value) for value in positions]
            point.velocities = [float(value) for value in velocities]
            point.time_from_start = self._duration(timestamp)
            trajectory.points.append(point)
            previous_time = timestamp
        return trajectory

    def _duration(self, seconds_value: float) -> Any:
        seconds = int(seconds_value)
        nanoseconds = int(round((seconds_value - seconds) * 1_000_000_000))
        if nanoseconds >= 1_000_000_000:
            seconds += 1
            nanoseconds -= 1_000_000_000
        duration = self._Duration()
        duration.sec = seconds
        duration.nanosec = nanoseconds
        return duration

    def _execute_trajectory_message(
        self,
        trajectory: Any,
        *,
        timeout_s: float,
        cancel_event: threading.Event | None,
        label: str,
    ) -> None:
        goal = self._FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        result = self._run_action(
            self._follow_joint_trajectory_client,
            goal,
            timeout_s,
            cancel_event=cancel_event,
        )
        if int(result.error_code) != int(
            self._FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                f"{label} failed: code={int(result.error_code)} "
                f"{getattr(result, 'error_string', '')}"
            )

    def start_gravity_compensation(self) -> None:
        self._call_trigger_service(self._gc_start_client, "start gravity compensation")

    def stop_gravity_compensation(self) -> None:
        self._call_trigger_service(self._gc_stop_client, "stop gravity compensation")

    def safe_home(self) -> None:
        self._call_trigger_service(
            self._safe_home_client,
            "safe home",
            timeout_s=self._safe_home_timeout_s,
        )

    def disable(self) -> None:
        self._call_trigger_service(self._disable_client, "disable")

    def stop_and_hold(self) -> None:
        self._call_trigger_service(self._stop_client, "stop and hold")

    def _run_action(
        self,
        client: Any,
        goal: Any,
        timeout_s: float,
        *,
        cancel_event: threading.Event | None = None,
    ) -> Any:
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
        finally:
            with self._active_goal_lock:
                self._action_starting = False
        with self._active_goal_lock:
            self._active_goal = goal_handle
            self._cancel_requested_goal = None
        try:
            if cancel_event is not None and cancel_event.is_set():
                self.cancel_active()
            wrapped = self._wait_future(
                goal_handle.get_result_async(),
                max(float(timeout_s), self._timeout_s),
            )
            if wrapped is None:
                raise TimeoutError("ROS action returned no result")
            if wrapped.status != self._GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(getattr(wrapped.result, "message", "action failed"))
            return wrapped.result
        except BaseException:
            try:
                self.cancel_active()
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
        """Cancel the active trajectory goal, if one exists."""
        with self._active_goal_lock:
            goal = self._active_goal
            if goal is None or self._cancel_requested_goal is goal:
                return
            self._cancel_requested_goal = goal
        self._wait_future(goal.cancel_goal_async(), timeout_s)

    def _call_trigger_service(
        self,
        client: Any,
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

    @staticmethod
    def _wait_future(future: Any, timeout_s: float) -> Any:
        deadline = time.monotonic() + float(timeout_s)
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError("ROS request timed out")
            time.sleep(0.01)
        return future.result()

    def close(self) -> None:
        """Stop ROS execution, destroy the node, and release owned rclpy."""
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
