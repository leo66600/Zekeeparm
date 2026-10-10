"""Print the connected camera serial number."""

from contextlib import contextmanager, redirect_stderr, redirect_stdout
from io import StringIO
import os
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
CONFIG_PATH = PROJECT_ROOT / "config/default.yaml"


@contextmanager
def quiet_camera():
    quiet = StringIO()
    stdout_fd, stderr_fd = os.dup(1), os.dup(2)
    null_fd = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(null_fd, 1)
        os.dup2(null_fd, 2)
        with redirect_stdout(quiet), redirect_stderr(quiet):
            yield
    finally:
        os.dup2(stdout_fd, 1)
        os.dup2(stderr_fd, 2)
        os.close(null_fd)
        os.close(stdout_fd)
        os.close(stderr_fd)


def write_camera_serial(serial: str) -> None:
    lines = CONFIG_PATH.read_text().splitlines(keepends=True)
    in_camera = False
    for index, line in enumerate(lines):
        if line.startswith("camera:"):
            in_camera = True
        elif in_camera and line and not line.startswith((" ", "\t")):
            in_camera = False
        elif in_camera and line.startswith("  serial:"):
            ending = "\n" if line.endswith("\n") else ""
            escaped = serial.replace("\\", "\\\\").replace('"', '\\"')
            lines[index] = f'  serial: "{escaped}"{ending}'
            CONFIG_PATH.write_text("".join(lines))
            return
    raise RuntimeError(f"camera.serial not found in {CONFIG_PATH}")


def main() -> int:
    with quiet_camera():
        from drivers.camera import make_camera
        from utils.camera_utils import load_config

        camera = make_camera(load_config(PROJECT_ROOT / "config/default.yaml"))
        try:
            camera.open()
            serial = camera.serial
        finally:
            camera.close()
    write_camera_serial(serial)
    print(serial)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
