#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
miniforge_dir="${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}"
ros_packages=(
  zekeep_msgs
  zekeepcontroller
  zekeep_bringup
  zekeep_moveit_config
  zekeep_teach
  zekeep_joystick
)

die() {
  printf '错误：%s\n' "$*" >&2
  exit 1
}

step() {
  printf '\n== %s ==\n' "$*"
}

[[ "$(uname -m)" == "x86_64" ]] || die "只支持 x86_64"
source /etc/os-release
[[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]] || die "只支持 Ubuntu 22.04"
[[ ${EUID} -ne 0 ]] || die "请用普通用户运行，脚本会在需要时调用 sudo"
setup_temp_dir="$(mktemp -d)"
trap 'rm -rf -- "$setup_temp_dir"' EXIT

step "获取官方 reBotArm SDK"
sdk_dir="$workspace_dir/third_party/reBotArm_control_py"
if [[ ! -d "$sdk_dir/.git" ]]; then
  mkdir -p "$workspace_dir/third_party"
  git clone --depth 1 https://github.com/Seeed-Projects/reBotArm_control_py.git "$sdk_dir"
fi

step "安装 ROS 2 Humble 和系统依赖"
sudo apt-get update
sudo apt-get install -y ca-certificates curl wget locales software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo add-apt-repository universe -y

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  ros_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | sed -n 's/.*"tag_name":[[:space:]]*"\([^"]*\)".*/\1/p' | head -n1)"
  [[ -n "$ros_apt_version" ]] || die "无法取得 ros-apt-source 版本"
  ros_apt_deb="$setup_temp_dir/ros2-apt-source.deb"
  curl -fL -o "$ros_apt_deb" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.jammy_all.deb"
  sudo dpkg -i "$ros_apt_deb"
fi

sudo apt-get update
sudo apt-get install -y \
  ros-humble-desktop ros-humble-moveit ros-humble-rosbridge-server \
  ros-humble-joy ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-tf-transformations python3-colcon-common-extensions \
  python3-rosdep python3-venv python3-pip build-essential gcc-11 g++-11 \
  cmake git git-lfs lsof nodejs ffmpeg

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths "$workspace_dir/src" --ignore-src --rosdistro humble -r -y
sudo usermod -aG dialout "$(id -un)"

step "创建 ROS Python 环境并构建工作区"
if [[ ! -x "$workspace_dir/.venv-ros/bin/python" ]]; then
  /usr/bin/python3 -m venv --system-site-packages "$workspace_dir/.venv-ros"
fi
source "$workspace_dir/.venv-ros/bin/activate"
export PYTHONNOUSERSITE=1
python -m pip install --upgrade pip
python -m pip install -r "$workspace_dir/tools/requirements-ros-sdk.txt"
python -m pip install --no-deps -e "$workspace_dir/third_party/reBotArm_control_py"
cd "$workspace_dir"
ros_build_dir="$setup_temp_dir/ros-build"
python -m colcon --log-base "$ros_build_dir/log" build \
  --build-base "$ros_build_dir/build" --install-base "$workspace_dir/install" \
  --base-paths src --packages-select "${ros_packages[@]}" --symlink-install
deactivate

step "准备 Conda"
if command -v conda >/dev/null 2>&1; then
  conda_exe="$(command -v conda)"
elif [[ -x "$HOME/miniconda3/bin/conda" ]]; then
  conda_exe="$HOME/miniconda3/bin/conda"
elif [[ -x "$miniforge_dir/bin/conda" ]]; then
  conda_exe="$miniforge_dir/bin/conda"
else
  miniforge_installer="$setup_temp_dir/Miniforge3-Linux-x86_64.sh"
  curl -fL --retry 3 -o "$miniforge_installer" https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
  curl -fL --retry 3 -o "${miniforge_installer}.sha256" https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh.sha256
  miniforge_sha="$(awk 'NR == 1 {print $1}' "${miniforge_installer}.sha256")"
  printf '%s  %s\n' "$miniforge_sha" "$miniforge_installer" | sha256sum -c -
  bash "$miniforge_installer" -b -p "$miniforge_dir"
  conda_exe="$miniforge_dir/bin/conda"
fi

step "创建视觉环境"
if "$conda_exe" env list | awk '{print $1}' | grep -qx rebotarm; then
  "$conda_exe" env update -n rebotarm -f "$workspace_dir/src/rebot_grasp/environment.yml"
else
  "$conda_exe" env create -f "$workspace_dir/src/rebot_grasp/environment.yml"
fi
"$conda_exe" run -n rebotarm python -m pip install --force-reinstall \
  -c "$workspace_dir/tools/constraints-vision.txt" \
  torch==2.7.1 torchvision==0.22.1 torchaudio==2.7.1 \
  --index-url https://download.pytorch.org/whl/cu128 \
  --extra-index-url https://pypi.org/simple
"$conda_exe" run -n rebotarm python -m pip install --no-deps -e "$workspace_dir/third_party/reBotArm_control_py"
"$conda_exe" run -n rebotarm python -m pip install --force-reinstall --no-deps opencv-contrib-python==4.7.0.72

graspnet_accepted=${ACCEPT_GRASPNET_LICENSE:-}
if [[ "$graspnet_accepted" != YES ]]; then
  printf '%s\n' 'GraspNet 源码和权重仅限本人或同一机构单站点的非商业内部研究，禁止转让或向第三方分发。'
  if ! read -r -p '接受 GraspNet 许可并从官方仓库获取源码？输入 YES：' graspnet_accepted; then
    graspnet_accepted=NO
  fi
fi
if [[ "$graspnet_accepted" == YES ]]; then
  if [[ ! -d "$workspace_dir/third_party/graspnet-baseline/.git" ]]; then
    git clone --depth 1 https://github.com/graspnet/graspnet-baseline.git \
      "$workspace_dir/third_party/graspnet-baseline"
  fi
  "$conda_exe" run -n rebotarm python -m pip install \
    -c "$workspace_dir/tools/constraints-vision.txt" --no-build-isolation \
    "$workspace_dir/third_party/graspnet-baseline/graspnetAPI"
else
  printf '%s\n' '已跳过 GraspNet 源码和原生扩展。'
fi

step "创建 LeRobot 遥操作环境"
if ! "$conda_exe" env list | awk '{print $1}' | grep -qx lerobot; then
  "$conda_exe" create -n lerobot python=3.12 -y
fi
"$conda_exe" run -n lerobot python -m pip install --upgrade pip
"$conda_exe" run -n lerobot python -m pip install -e "$workspace_dir/src/zekeep_teleop"

if [[ "$graspnet_accepted" == YES ]]; then
  cuda_home=${CUDA_HOME:-/usr/local/cuda-12.8}
  if [[ -x "$cuda_home/bin/nvcc" ]] && nvidia-smi >/dev/null 2>&1; then
    step "构建 GraspNet CUDA 扩展"
    gpu_arch="$("$conda_exe" run -n rebotarm python -c 'import torch; print(".".join(map(str, torch.cuda.get_device_capability())))')"
    CUDA_HOME="$cuda_home" TORCH_CUDA_ARCH_LIST="$gpu_arch" \
      "$conda_exe" run -n rebotarm bash "$workspace_dir/tools/build_vision_extensions.sh"
  else
    printf '%s\n' '未找到 CUDA 12.8 Toolkit 或可见 NVIDIA GPU；已跳过 GraspNet 原生扩展。'
    printf '%s\n' '按 README 安装驱动和 CUDA 12.8 后重跑本脚本。'
  fi
fi

printf '\n安装流程完成。重新登录后 dialout 组权限生效。\n'
printf 'ROS：cd %q && source tools/activate_ros.sh\n' "$workspace_dir"
printf '视觉：%s run -n rebotarm bash\n' "$conda_exe"
printf '遥操作：%s run -n lerobot zhongling-teleoperate --config_path=%q\n' "$conda_exe" "$workspace_dir/src/zekeep_teleop/configs/zhongling_b601_teleop.yaml"
