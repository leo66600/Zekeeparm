#!/usr/bin/env bash
# Build on the target GPU. Does not open cameras or robot hardware.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
: "${CUDA_HOME:?Set CUDA_HOME to the CUDA 12.8 Toolkit directory}"
: "${TORCH_CUDA_ARCH_LIST:?Set target GPU architecture, e.g. 8.6 or 12.0}"
export CC="${CC:-gcc-11}"
export CXX="${CXX:-g++-11}"
export PATH="$CUDA_HOME/bin:$PATH"
python - <<'CHECK'
import sys
import subprocess
import torch
assert sys.version_info[:2] == (3, 10), "Python 3.10 required"
assert torch.__version__.split("+")[0] == "2.7.1", torch.__version__
assert torch.version.cuda == "12.8", torch.version.cuda
assert torch.cuda.is_available(), "A visible target NVIDIA GPU is required"
assert "release 12.8" in subprocess.check_output(["nvcc", "--version"], text=True)
print(torch.cuda.get_device_name(), torch.cuda.get_device_capability())
CHECK
# Build in a fresh temporary copy: never reuse old ABI/GPU objects from checkout.
build_dir="$(mktemp -d)"
trap 'rm -rf -- "$build_dir"' EXIT
python - "$workspace_dir" "$build_dir" <<'COPY'
import shutil, sys
from pathlib import Path
source = Path(sys.argv[1]) / "third_party/graspnet-baseline"
for name in ("pointnet2", "knn"):
    shutil.copytree(source / name, Path(sys.argv[2]) / name,
                    ignore=shutil.ignore_patterns("build", "dist", "*.so", "*.o", "*.egg-info", "__pycache__"))
COPY
python -m pip install --no-build-isolation --no-deps "$build_dir/pointnet2" "$build_dir/knn"
printf 'GraspNet CUDA extensions installed.\n'
