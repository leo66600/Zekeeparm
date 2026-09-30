#!/usr/bin/env python3
"""Supervised ROS hand-guided teaching and low-speed replay tool."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path
import select
import sys
import termios
import threading
import time
import tty
from typing import Any, Callable, Iterator

import numpy as np
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]

from .robot import (
    RosTeachRobot,
    TeachConfig,
    TeachState,
)
from .ros_client import TeachRosClient


def load_config(path: str | Path) -> dict[str, Any]:
    """Load the YAML teaching configuration and require a mapping root."""
    source = Path(path)
    try:
        data = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"failed to load teaching config: {source}") from exc
    if not isinstance(data, dict):
        raise ValueError("teaching config root must be a mapping")
    return data


def default_config_path() -> Path:
    """Return the source-tree or installed default configuration path."""
    source_config = PACKAGE_ROOT / "config" / "default.yaml"
    if source_config.exists():
        return source_config
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("zekeep_teach")) / "config" / "default.yaml"


class MotionWorker:
    """Allow one blocking replay while the terminal thread remains responsive."""

    def __init__(self, robot: RosTeachRobot) -> None:
        self._robot = robot
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._error: BaseException | None = None
        self._label = ""

    @property
    def active(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def start(self, action: Callable[[], None], *, label: str) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("another replay is already active")
            self._error = None
            self._label = str(label)

            def run() -> None:
                try:
                    action()
                except BaseException as exc:
                    with self._lock:
                        self._error = exc

            self._thread = threading.Thread(
                target=run,
                name=f"teach-{self._label}",
                daemon=True,
            )
            self._thread.start()

    def cancel(self) -> None:
        if self.active:
            self._robot.cancel_replay()

    def join(self, *, timeout_s: float, raise_error: bool = True) -> None:
        with self._lock:
            thread = self._thread
        if thread is None:
            return
        thread.join(timeout=max(float(timeout_s), 0.0))
        if thread.is_alive():
            raise TimeoutError("replay worker did not stop; safe exit is blocked")
        with self._lock:
            error = self._error
            self._thread = None
            self._error = None
        if error is not None and raise_error:
            raise error

    def completed_error(self) -> BaseException | None:
        with self._lock:
            thread = self._thread
            error = self._error
        if thread is None or thread.is_alive():
            return None
        self.join(timeout_s=0.0, raise_error=False)
        return error


def stop_worker_for_safe_exit(
    worker: MotionWorker,
    *,
    timeout_s: float,
    report: Callable[[str], None] = print,
) -> None:
    """Keep the ROS process alive until replay cancellation is confirmed."""
    while True:
        try:
            worker.cancel()
        except KeyboardInterrupt:
            report("[Exit] cancel confirmation is still required; retrying")
        except Exception as exc:
            report(f"[Exit] cancel request failed; retrying: {exc}")
        try:
            worker.join(timeout_s=timeout_s, raise_error=False)
            return
        except TimeoutError:
            report("[Exit] replay is still stopping; home and disable remain blocked")
        except KeyboardInterrupt:
            report("[Exit] waiting for replay hold confirmation; exit remains blocked")


def build_teach_config(
    values: dict[str, Any],
) -> tuple[TeachConfig, dict[str, float | str]]:
    """Convert YAML values into validated teaching and runtime settings."""
    values = values or {}
    config = TeachConfig(
        stationary_velocity_rad_s=float(
            values.get("stationary_velocity_rad_s", 0.02)
        ),
        duplicate_point_distance_rad=float(
            values.get("duplicate_point_distance_rad", 0.01)
        ),
        path_minimum_distance_rad=float(
            values.get("path_minimum_distance_rad", 0.002)
        ),
        replay_max_velocity_rad_s=float(
            values.get("replay_max_velocity_rad_s", 0.10)
        ),
        minimum_segment_s=float(values.get("minimum_segment_s", 0.05)),
        settle_position_tolerance_rad=float(
            values.get("settle_position_tolerance_rad", 0.005)
        ),
        waypoint_position_tolerance_rad=float(
            values.get(
                "waypoint_position_tolerance_rad",
                values.get("settle_position_tolerance_rad", 0.005),
            )
        ),
        settle_velocity_tolerance_rad_s=float(
            values.get("settle_velocity_tolerance_rad_s", 0.02)
        ),
        settle_required_samples=int(values.get("settle_required_samples", 3)),
        settle_sample_interval_s=float(
            values.get("settle_sample_interval_s", 0.05)
        ),
        settle_timeout_s=float(values.get("settle_timeout_s", 8.0)),
    )
    runtime = {
        "sample_rate_hz": float(values.get("sample_rate_hz", 20.0)),
        "safe_home_timeout_s": float(values.get("safe_home_timeout_s", 35.0)),
        "worker_stop_timeout_s": float(values.get("worker_stop_timeout_s", 5.0)),
        "output_dir": str(values.get("output_dir", "logs/teach_sessions")),
    }
    numeric = [
        config.stationary_velocity_rad_s,
        config.duplicate_point_distance_rad,
        config.path_minimum_distance_rad,
        config.replay_max_velocity_rad_s,
        config.minimum_segment_s,
        config.settle_position_tolerance_rad,
        config.waypoint_position_tolerance_rad,
        config.settle_velocity_tolerance_rad_s,
        config.settle_sample_interval_s,
        config.settle_timeout_s,
        float(runtime["sample_rate_hz"]),
        float(runtime["safe_home_timeout_s"]),
        float(runtime["worker_stop_timeout_s"]),
    ]
    if not np.all(np.isfinite(numeric)) or any(value <= 0.0 for value in numeric):
        raise ValueError("teaching timing, tolerance, and speed values must be positive")
    if config.settle_required_samples <= 0:
        raise ValueError("settle_required_samples must be positive")
    return config, runtime


def decode_key(data: bytes) -> str:
    if data in (b"", b"\x03", b"\x04"):
        return "Q"
    key = data.decode("utf-8", errors="ignore")
    return key if key == " " else key.upper()


def dispatch_key(
    key: str,
    robot: RosTeachRobot,
    worker: MotionWorker,
    output_path: Path,
    *,
    report: Callable[[str], None] = print,
    confirm_path_overwrite: Callable[[], bool] | None = None,
) -> bool:
    """Execute one terminal command and return whether the loop continues."""
    if key == "Q":
        report("[Key Q] safe exit requested")
        return False
    if worker.active:
        raise RuntimeError("replay is active; only Q is accepted")
    if key == "T":
        robot.enter_guiding()
        report("[Key T] guiding enabled")
    elif key == " ":
        if not robot.capture_point():
            raise RuntimeError("teaching point is too close to the previous point")
        report("[Key Space] teaching point captured")
    elif key == "C":
        overwrite = False
        if not robot.continuous_recording and robot.has_recorded_path:
            if confirm_path_overwrite is None or not confirm_path_overwrite():
                report("[Key C] overwrite cancelled; existing path preserved")
                return True
            overwrite = True
        recording = robot.toggle_continuous_recording(overwrite=overwrite)
        if recording and overwrite:
            report("[Key C] existing path overwritten; recording started")
        else:
            state = "started" if recording else "stopped"
            report(f"[Key C] path recording {state}")
    elif key == "U":
        if not robot.undo_point():
            raise RuntimeError("there is no discrete point to remove")
        report("[Key U] last teaching point removed")
    elif key == "1":
        worker.start(robot.replay_points, label="point-replay")
        report("[Key 1] point replay started")
    elif key == "2":
        worker.start(robot.replay_path, label="path-replay")
        report("[Key 2] path replay started")
    elif key == "S":
        saved_path = robot.save(output_path)
        report(f"[Key S] saved: {saved_path}")
    else:
        raise ValueError(f"unknown key: {key!r}")
    return True


@contextmanager
def raw_terminal(stream: Any) -> Iterator[int]:
    """Temporarily configure an interactive stream for single-key input."""
    if not stream.isatty():
        raise RuntimeError("immediate-key teaching requires an interactive TTY")
    descriptor = stream.fileno()
    previous = termios.tcgetattr(descriptor)
    tty.setraw(descriptor)
    try:
        yield descriptor
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, previous)


def default_output_path(output_dir: str) -> Path:
    directory = Path(output_dir)
    if not directory.is_absolute():
        directory = PACKAGE_ROOT / directory
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    return directory / f"teach_{timestamp}.json"


def run_terminal(
    robot: RosTeachRobot,
    *,
    output_path: Path,
    sample_rate_hz: float,
    worker_stop_timeout_s: float,
) -> int:
    worker = MotionWorker(robot)
    period_s = 1.0 / float(sample_rate_hz)
    next_sample_monotonic = time.monotonic()
    print(
        "T guide | Space point | C path record | U undo | "
        "1 points | 2 path | S save | Q exit"
    )
    keep_running = True
    try:
        with raw_terminal(sys.stdin) as descriptor:
            def confirm_path_overwrite() -> bool:
                print(
                    "\r\n[Confirm] 已有连续路径，覆盖后旧路径将被删除。"
                    "是否覆盖？[y/N] ",
                    end="",
                    flush=True,
                )
                approved = decode_key(sys.stdin.buffer.read(1)) == "Y"
                print("Y" if approved else "N")
                return approved

            while keep_running:
                completed_error = worker.completed_error()
                if completed_error is not None:
                    print(f"\r\n[Replay] {completed_error}")
                now = time.monotonic()
                if robot.continuous_recording and now >= next_sample_monotonic:
                    try:
                        robot.sample_continuous()
                    except Exception as exc:
                        print(f"\r\n[Record] {exc}")
                    next_sample_monotonic = now + period_s
                readable, _, _ = select.select([descriptor], [], [], period_s)
                if not readable:
                    continue
                key = decode_key(sys.stdin.buffer.read(1))
                try:
                    keep_running = dispatch_key(
                        key,
                        robot,
                        worker,
                        output_path,
                        report=lambda message: print(f"\r\n{message}"),
                        confirm_path_overwrite=confirm_path_overwrite,
                    )
                except Exception as exc:
                    print(f"\r\n[Command] {exc}")
    finally:
        stop_worker_for_safe_exit(
            worker,
            timeout_s=worker_stop_timeout_s,
        )
        robot.safe_exit()
    print("[Exit] safe-home complete, stationary, motors disabled")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="ROS gravity-compensation teaching and low-speed replay"
    )
    parser.add_argument("--input", type=Path, help="load an existing teaching JSON")
    parser.add_argument("--output", type=Path, help="teaching JSON output path")
    parser.add_argument(
        "--config",
        type=Path,
        default=default_config_path(),
        help="teaching configuration YAML",
    )
    args = parser.parse_args()

    config_data = load_config(args.config)
    robot_values = config_data.get("robot", {})
    teach_config, runtime = build_teach_config(config_data.get("teaching", {}))
    output_path = args.output or default_output_path(str(runtime["output_dir"]))
    if not sys.stdin.isatty():
        print("[Fatal] teaching requires an interactive TTY", file=sys.stderr)
        return 2
    client = TeachRosClient(
        namespace=str(robot_values.get("namespace", "zekeep")),
        joint_state_topic=str(
            robot_values.get("joint_state_topic", "/zekeep/joint_states")
        ),
        timeout_s=float(robot_values.get("ros_timeout_s", 8.0)),
        joint_state_max_age_s=float(robot_values.get("joint_state_max_age_s", 0.5)),
        safe_home_timeout_s=float(runtime["safe_home_timeout_s"]),
    )
    robot = RosTeachRobot(client, teach_config)
    print(
        "[Safety] Supervised development tool; keyboard input is not a "
        "safety-rated enable device."
    )
    try:
        if args.input is not None:
            robot.load(args.input)
        robot.start()
        return run_terminal(
            robot,
            output_path=output_path,
            sample_rate_hz=float(runtime["sample_rate_hz"]),
            worker_stop_timeout_s=float(runtime["worker_stop_timeout_s"]),
        )
    except KeyboardInterrupt:
        if robot.state in (TeachState.HOLD, TeachState.GUIDING):
            robot.safe_exit()
        return 130
    except Exception as exc:
        if robot.state is TeachState.DISCONNECTED:
            try:
                client.close()
            except Exception:
                pass
        print(f"[Fatal] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
