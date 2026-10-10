# Zekeeparm

六轴机械臂独立运行包，覆盖真机控制、MoveIt、视觉抓取、拖动示教和遥操作。

**目标平台：** Ubuntu 22.04 x86_64 · ROS 2 Humble · Python 3.10

[项目组成](#项目组成) · [一键安装](#一键安装) · [NVIDIA 驱动与 CUDA](#nvidia-驱动和-cuda-128) · [运行入口](#运行入口) · [标定](#标定边界)

## 项目组成

| 模块 | 功能 |
| --- | --- |
| `zekeep_bringup` | 模型、硬件参数、真机、RViz 和网页启动 |
| `zekeepcontroller` | 硬件抽象、状态机、ROS 服务和轨迹 Action |
| `zekeep_msgs` | 自定义消息、服务和 Action |
| `zekeep_moveit_config` | MoveIt 配置 |
| `zekeep_teach` | 重力补偿拖动示教、网页任务和轨迹复现 |
| `zekeep_shadow` | 网页 RGB-D 检测及 YOLO/GraspNet 抓取后端 |
| `zekeep_llm` | 网页 LLM/MCP 计划、人工确认执行和取消 |
| `zekeep_joystick` | 六轴关节及末端手柄控制 |
| `zekeep_grasp` | 相机、当前标定、检测和抓取 |
| `zekeep_teleop` | 独立 LeRobot 主从遥操作 |
| `zekeeparm_SDK` | 机械臂 SDK，仅保留 DM 硬件配置 |

GraspNet 源码由安装者接受许可限制后从官方仓库获取，不随本仓库分发。

发布源码包含已有检查脚本，不包含 MuJoCo、具身学习和数据采集模块。
模型权重、下载的 GraspNet、日志、缓存及构建产物由本地安装或运行生成，
当前已安装工作区可包含这些文件；它们不属于发布源码清单。

## 一键安装

> [!IMPORTANT]
> 安装需联网，脚本会使用 `sudo apt`。NVIDIA 驱动和 CUDA Toolkit 需按[下方步骤](#nvidia-驱动和-cuda-128)单独安装。脚本不会连接相机或机械臂，也不会使能电机。

```bash
git clone --branch 1010 https://github.com/Leocia/Zekeeparm.git "$HOME/Desktop/Zekeeparm"
cd "$HOME/Desktop/Zekeeparm"
bash setup.sh
```

脚本安装 ROS、SDK、视觉环境和 LeRobot 遥操作环境，并下载视觉模型权重。中断后可直接重跑。

视觉与遥操作统一使用 Miniforge，默认安装到 `~/miniforge3`，环境分别为
`~/miniforge3/envs/rebotarm`（Python 3.10）和 `~/miniforge3/envs/lerobot`（Python 3.12）。
自定义安装位置时，在安装和运行前设置 `ZKEEP_MINIFORGE_DIR` 为实际绝对路径。
脚本使用该目录的 Conda 和明确的环境路径，不复用其他 Conda 的同名环境。
ROS 仍使用独立 `.venv-ros`；三个环境的 Python、NumPy 和 MotorBridge 版本约束保持分离。

已有 Miniconda 环境的电脑需先在 Miniforge 中安装项目环境，不能直接改目录名。
过渡期间可用 `ZKEEP_VISION_PYTHON` 显式指定现有视觉 Python；该变量优先于 Miniforge 默认路径。

脚本未找到 CUDA 12.8 或可见 NVIDIA GPU 时，会跳过 GraspNet 原生扩展。完成下方 GPU 安装后重跑 `bash setup.sh`。

### GraspNet 下载和编译

运行 `bash setup.sh` 时，脚本会询问是否获取 GraspNet。确认仅用于本人或同机构内部非商业研究并输入 `YES` 后，脚本自动：

- 克隆 `graspnet-baseline` 和 `graspnetAPI` 源码；
- 下载 `checkpoint-rs.tar` 模型权重；
- 在当前 GPU 上编译 `pointnet2` 和 `knn` CUDA 扩展。

不输入 `YES` 时，脚本跳过 GraspNet 源码、权重和原生扩展。基础 ROS、仿真、固定几何后端及其他功能仍可安装；YOLO/GraspNet 抓取后端无法运行。已存在且校验通过的文件不会重复下载。

## NVIDIA 驱动和 CUDA 12.8

先查看 Ubuntu 推荐驱动，再由用户确认安装：

```bash
sudo apt update
sudo apt install -y ubuntu-drivers-common
ubuntu-drivers devices
sudo ubuntu-drivers install
sudo reboot
```

重启后确认驱动：

```bash
nvidia-smi
```

安装 CUDA Toolkit 12.8，不使用 `cuda` 元包，避免由该命令主动安装驱动：

```bash
cd /tmp
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-12-8
```

设置当前终端并重跑安装脚本：

```bash
export CUDA_HOME=/usr/local/cuda-12.8
export PATH="$CUDA_HOME/bin:$PATH"
cd "$HOME/Desktop/Zekeeparm"
bash setup.sh
```

## 运行入口

### ROS、RViz、网页、手柄和示教

```bash
cd "$HOME/Desktop/Zekeeparm"
source install/setup.bash
```

如需每次打开 Bash 终端时自动加载 ROS 2 和本工作区，在 `~/.bashrc` 末尾添加一次：

```bash
if [ -f "$HOME/Desktop/Zekeeparm/install/setup.bash" ]; then
  source "$HOME/Desktop/Zekeeparm/install/setup.bash"
fi
```

保存后，在当前终端执行：

```bash
source ~/.bashrc
ros2 pkg prefix zekeep_bringup
```

输出应为工作区的 `install/zekeep_bringup` 目录。
以后新终端无需重复加载；此配置只加载环境，不启动驱动或使能电机。
迁移工作区后需修改上述绝对路径。使用其他 ROS 工作区时，需注意自动加载的包路径。

真机关节小幅运动测试见[关节测试教程](docs/JOINT_SMALL_MOTION_TEST.md)。

无硬件 RViz：

```bash
ros2 launch zekeep_moveit_config demo.launch.py
```

网页：

```bash
ros2 launch zekeep_bringup web.launch.py
```

每个新终端加载一次环境即可。需要加载 SDK/Python 依赖时，执行
`source tools/activate_ros.sh`。迁移到其他电脑时，运行 `setup.sh` 创建本机环境。

网页地址 `http://127.0.0.1:3001`。支持 TCP 只读预览、网页与实机示教、LLM/MCP 文本控制。
`web.launch.py` 默认启动只读 AI 后端，不启动实机驱动或相机。
网页视觉抓取的统一入口：

```bash
"$HOME/Desktop/Zekeeparm/tools/start_web_sim.sh"
```

启动不自动使能或抓取；AI 运动还需 `--ai-motion` 和网页人工确认。
只读入口加 `--preview`。视觉 Python 默认使用 Miniforge 的 `rebotarm` 环境；
可用 `ZKEEP_MINIFORGE_DIR` 指定根目录，或用 `ZKEEP_VISION_PYTHON` 指定解释器。SDK 环境用
`ZKEEP_ROS_VENV` 指定。完整说明见 [网页 README](src/zekeep_bringup/web/README.md)。

### 视觉抓取

```bash
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/rebotarm"
cd "$HOME/Desktop/Zekeeparm/src/zekeep_grasp"
```

### 主从遥操作

```bash
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/lerobot"
cd "$HOME/Desktop/Zekeeparm/src/zekeep_teleop"
```

夹爪统一采用 `1.45 rad = 70 mm` 的线性映射，电机目标限制为 `0–1.35 rad`，
最大允许开口约 `65.17 mm`。ROS 与 SDK 的 URDF 内容一致；SDK 保留 `official_tcp`
兼容坐标系。修改模型后须核对现场并重新确认标定身份。

## 标定边界

发布源码包含当前 `intrinsics.npz` 和 `hand_eye.npz`，不分发本地设备身份记录或标定历史备份；安装后的工作区可包含这些本地文件。这些标定不能证明目标电脑或现场已经验证。设备、安装位置或场景变化后必须重新标定；完全未变化时也应先执行相机身份确认：

```bash
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/rebotarm"
cd "$HOME/Desktop/Zekeeparm"
python src/zekeep_grasp/scripts/confirm_calibration.py \
  --config src/zekeep_grasp/config/default.yaml --confirm-same-installation
```

公开仓库的 `config/default.yaml` 中设备身份字段留空。确认前请在 `src/zekeep_grasp/config/default.yaml` 填写实际 `camera.serial`、`robot.id` 和 `calibration.installation_id`。

该命令只打开相机，不连接或使能机械臂。完整硬件安全说明见[视觉抓取说明](src/zekeep_grasp/README.md)等各包 README。

## 许可

本工作区仅供本人或同一机构内部非商业研究，不得用于商业用途或向第三方转让、分发。GraspNet 源码与权重另受其使用许可限制；定制 SDK 按适用许可条款使用。

## 安全提示

> [!WARNING]
> 安装脚本不会安装或更换 NVIDIA 驱动，不会自动连接相机或机械臂，也不会使能电机。真机启动前先检查急停、工作空间、速度与碰撞边界。操作者需为设备运行及由此产生的损害负责。
