# 六轴全栈具身机器人学习平台 --ZK-EM600擎栈

面向 Ubuntu 22.04 x86_64、ROS 2 Humble、Python 3.10 的zekeeparm运行与集成工作区。覆盖 ROS 控制、MoveIt、示教、手柄、视觉抓取和独立主从遥操作。

## 能力

| 模块 | 能力 | 路径 |
|---|---|---|
| Bringup 与硬件控制 | 真机、RViz、网页、硬件抽象、状态机、ROS 服务和轨迹 Action | [`src/zekeep_bringup`](src/zekeep_bringup)、[`src/zekeepcontroller`](src/zekeepcontroller) |
| ROS 接口与规划 | 自定义消息/服务/Action、MoveIt 配置 | [`src/zekeep_msgs`](src/zekeep_msgs)、[`src/zekeep_moveit_config`](src/zekeep_moveit_config) |
| 示教与手柄 | 重力补偿拖动示教、轨迹复现、六轴与末端遥控 | [`src/zekeep_teach`](src/zekeep_teach)、[`src/zekeep_joystick`](src/zekeep_joystick) |
| 视觉抓取 | 相机、当前内参与手眼标定、检测、GraspNet 抓取 | [`src/rebot_grasp`](src/rebot_grasp) |
| LeRobot 遥操作 | 独立 Python/Conda 主从遥操作环境 | [`src/zekeep_teleop`](src/zekeep_teleop) |
| 定制 SDK | 独立仓库维护，安装固定到 `v0.1.0` | [Zekeep_control_py](https://github.com/leo66600/Zekeep_control_py) |

集成布局：ROS 包置于 `src/`；安装与构建脚本置于 `tools/` 和根目录；定制 SDK 独立仓库维护；各模块说明置于对应包内。

## 一键安装

联网执行。脚本会使用 `sudo apt`，但不会安装 NVIDIA 驱动或 CUDA Toolkit，不会连接相机或机械臂，也不会使能电机。

```bash
git clone https://github.com/leo66600/Zekeeparm.git "$HOME/Desktop/Zekeeparm"
cd "$HOME/Desktop/Zekeeparm"
bash setup.sh
```

脚本安装 ROS、SDK、视觉环境和 LeRobot 遥操作环境。不下载模型权重，也不安装 VLM。GraspNet 源码仅在用户明确接受其非商业内部研究许可后获取。中断后可直接重跑。

视觉检测需用户自行准备兼容 Ultralytics 的封闭类别分割权重，并放到 `src/rebot_grasp/models/custom-segmentation.pt`。仓库及安装脚本不提供或下载任何模型、VLA/VLM 或开放词汇检测组件。

脚本未找到 CUDA 12.8 或可见 NVIDIA GPU 时，会跳过 GraspNet 原生扩展。完成下方 GPU 安装后重跑 `bash setup.sh`。

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

## 环境入口

ROS、RViz、网页、手柄和示教：

```bash
cd "$HOME/Desktop/Zekeeparm"
source tools/activate_ros.sh
```

无硬件 RViz：

```bash
ros2 launch zekeep_moveit_config demo.launch.py
```

网页：

```bash
ros2 launch zekeep_bringup web.launch.py
```

视觉：

```bash
source "$HOME/miniforge3/etc/profile.d/conda.sh" 2>/dev/null || \
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate rebotarm
cd "$HOME/Desktop/Zekeeparm/src/rebot_grasp"
```

主从遥操作：

```bash
source "$HOME/miniforge3/etc/profile.d/conda.sh" 2>/dev/null || \
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate lerobot
cd "$HOME/Desktop/Zekeeparm/src/zekeep_teleop"
```

## 标定边界

仓库包含当前相机内参与手眼标定数据，不包含本地身份标识。先在本机配置 `src/rebot_grasp/config/default.yaml` 中的相机序列号、机械臂 ID 和安装 ID，再确认并生成本地身份文件：

```bash
cd "$HOME/Desktop/Zekeeparm"
conda run -n rebotarm python src/rebot_grasp/scripts/confirm_calibration.py \
  --config src/rebot_grasp/config/default.yaml --confirm-same-installation
```

该命令只打开相机，不连接或使能机械臂。设备、安装位置或场景变化后必须重新标定。`identity.local.json` 会被 Git 忽略。完整硬件安全说明见各包 README。

## 许可

本工作区仅供本人或同一机构内部非商业研究，不得用于商业用途或向第三方转让、分发。GraspNet 源码与权重另受其使用许可限制；定制 SDK 按适用许可条款使用。

## 安全提示

安装脚本不会安装或更换 NVIDIA 驱动，不会自动连接相机/机械臂，也不会使能电机。真机启动前先检查急停、工作空间、速度与碰撞边界。操作者需为设备运行及由此产生的损害负责。
