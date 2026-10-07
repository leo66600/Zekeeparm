"""Check GraspNet setup decisions without network, installs, or hardware."""
from pathlib import Path
import os
import subprocess
import tempfile

script = (Path(__file__).resolve().parents[1] / "setup.sh").read_text()
block = script.split("graspnet_accepted=${ACCEPT_GRASPNET_LICENSE:-}", 1)[1]
block = "graspnet_accepted=${ACCEPT_GRASPNET_LICENSE:-}" + block.split('step "创建 LeRobot 遥操作环境"', 1)[0]
stubs = r"""
set -euo pipefail
conda_exe=conda_stub
git() { printf 'CLONE %s\n' "$*" >> "$CHECK_LOG"; mkdir -p "${@: -1}"; }
conda_stub() { printf 'INSTALL %s\n' "$*" >> "$CHECK_LOG"; }
download_checked() { printf 'DOWNLOAD %s\n' "$2" >> "$CHECK_LOG"; }
"""

with tempfile.TemporaryDirectory() as temporary:
    root = Path(temporary)
    for name, accepted, existing, clone_count in (
        ("declined", "NO", False, 0),
        ("fresh", "YES", False, 2),
        ("existing", "YES", True, 0),
    ):
        workspace = root / name
        workspace.mkdir()
        if existing:
            (workspace / "third_party/graspnet-baseline/graspnetAPI").mkdir(parents=True)
        log = workspace / "commands"
        log.touch()
        environment = dict(os.environ, workspace_dir=str(workspace), CHECK_LOG=str(log),
                           ACCEPT_GRASPNET_LICENSE=accepted)
        subprocess.run(["bash", "-c", stubs + block], env=environment,
                       input="NO\n", text=True, check=True, capture_output=True)
        commands = log.read_text().splitlines()
        assert sum(line.startswith("CLONE ") for line in commands) == clone_count, commands
        if accepted == "YES":
            assert sum(line.startswith("INSTALL ") for line in commands) == 1, commands
            assert commands[-1].endswith("checkpoints/checkpoint-rs.tar"), commands
            if not existing:
                assert "graspnet/graspnet-baseline.git" in commands[0], commands
                assert "graspnet/graspnetAPI.git" in commands[1], commands
        else:
            assert not commands, commands
print("GraspNet declined, fresh checkout, and existing checkout: OK")
