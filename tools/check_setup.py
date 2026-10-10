"""Check environment selection and GraspNet setup without installs or hardware."""
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
vision_env_dir="$workspace_dir/envs/rebotarm"
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

environment_block = script.split('step "准备 Miniforge"', 1)[1].split('step "下载视觉模型"', 1)[0]
teleop_block = script.split('step "创建 LeRobot 遥操作环境"', 1)[1].split('if [[ "$graspnet_accepted" == YES ]]', 1)[0]
with tempfile.TemporaryDirectory() as temporary:
    root = Path(temporary)
    other_bin = root / 'other-conda/bin'
    other_bin.mkdir(parents=True)
    other = other_bin / 'conda'
    other.write_text('#!/bin/bash\nexit 99\n')
    other.chmod(0o700)
    for existing in (False, True):
        miniforge = root / ('existing forge' if existing else 'fresh forge')
        conda = miniforge / 'bin/conda'
        conda.parent.mkdir(parents=True)
        conda.write_text('#!/bin/bash\nprintf "%s\\n" "$*" >> "$CHECK_LOG"\n')
        conda.chmod(0o700)
        vision, teleop = miniforge / 'envs/rebotarm', miniforge / 'envs/lerobot'
        if existing:
            for prefix in (vision, teleop):
                (prefix / 'conda-meta').mkdir(parents=True)
                (prefix / 'conda-meta/history').touch()
        log = miniforge / 'commands'
        env = dict(os.environ, workspace_dir=str(root), miniforge_dir=str(miniforge),
                   vision_env_dir=str(vision), teleop_env_dir=str(teleop), CHECK_LOG=str(log),
                   PATH=str(other_bin) + os.pathsep + os.environ['PATH'])
        subprocess.run(['bash', '-c', 'set -euo pipefail\nstep() { :; }\n'
                        + environment_block + teleop_block], env=env, check=True, capture_output=True)
        commands = log.read_text().splitlines()
        assert commands[0].startswith(f'env {"update" if existing else "create"} -p {vision} -f '), commands
        assert sum(line.startswith(f'create -p {teleop} ') for line in commands) == int(not existing), commands
        assert all('-n ' not in line for line in commands), commands
        assert all(str(vision) in line or str(teleop) in line for line in commands), commands
print('Miniforge prefixes, fresh/repeated setup, and competing PATH Conda: OK')
