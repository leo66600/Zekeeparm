"""Exclusive ownership checks for official direct-SDK route."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os
import subprocess
from typing import Callable, Iterable


class SdkExclusiveError(RuntimeError):
    """SDK cannot safely own robot hardware."""


@dataclass(frozen=True)
class OwnershipReport:
    ros_controller_pids: tuple[int, ...]
    serial_holders: tuple[str, ...]

    @property
    def available(self) -> bool:
        return not self.ros_controller_pids and not self.serial_holders


def find_ros_controller_pids(
    process_lines: Iterable[str] | None = None,
) -> tuple[int, ...]:
    if process_lines is None:
        result = subprocess.run(
            ["ps", "-eo", "pid=,args="],
            check=True,
            capture_output=True,
            text=True,
        )
        process_lines = result.stdout.splitlines()
    found = []
    for line in process_lines:
        fields = line.strip().split(maxsplit=1)
        if len(fields) != 2 or "ZekeepController" not in fields[1]:
            continue
        try:
            pid = int(fields[0])
        except ValueError:
            continue
        if pid != os.getpid():
            found.append(pid)
    return tuple(sorted(set(found)))


def find_serial_holders(
    serial_path: str | Path,
    *,
    lsof_runner: Callable[..., subprocess.CompletedProcess] | None = None,
) -> tuple[str, ...]:
    path = str(serial_path)
    runner = lsof_runner or subprocess.run
    try:
        result = runner(
            ["lsof", "-nP", "--", path],
            check=False,
            capture_output=True,
            text=True,
        )
        # A stale desktop FUSE mount must not block inspection of a /dev device.
        # Retry with that mount exempted; keep all other inspection errors fatal.
        if (result.returncode in (0, 1)
                and result.stderr.strip() == (
                    "lsof: WARNING: can't stat() fuse file system /tmp/fuse\n"
                    "      Output information may be incomplete."
                )
                and Path(path).resolve().is_relative_to("/dev")
                and Path(path).is_char_device()):
            result = runner(
                ["lsof", "-nP", "-e", "/tmp/fuse", "--", path],
                check=False, capture_output=True, text=True,
            )
    except OSError as exc:
        raise SdkExclusiveError(f"serial ownership check unavailable; install lsof: {exc}") from exc
    if result.returncode not in (0, 1) or result.stderr.strip():
        raise SdkExclusiveError(
            f"serial ownership check failed ({result.returncode}): {result.stderr.strip()}"
        )
    if result.returncode == 0 and not result.stdout.strip():
        raise SdkExclusiveError("serial ownership check returned no parseable result")
    lines = [line.strip() for line in result.stdout.splitlines()[1:] if line.strip()]
    return tuple(lines)


def assert_sdk_exclusive(
    *,
    serial_path: str | Path | None,
    process_lines: Iterable[str] | None = None,
    lsof_runner: Callable[..., subprocess.CompletedProcess] | None = None,
) -> OwnershipReport:
    report = OwnershipReport(
        ros_controller_pids=find_ros_controller_pids(process_lines),
        serial_holders=find_serial_holders(serial_path, lsof_runner=lsof_runner)
        if serial_path
        else (),
    )
    if report.ros_controller_pids:
        raise SdkExclusiveError(
            "SDK start refused: ROS controller running "
            f"pids={report.ros_controller_pids}"
        )
    if report.serial_holders:
        raise SdkExclusiveError(
            "SDK start refused: serial/CAN occupied "
            f"path={serial_path}"
        )
    return report
