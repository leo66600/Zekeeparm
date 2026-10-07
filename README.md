# Zekeeparm

六轴机械臂独立运行包。目标平台固定为 Ubuntu 22.04 x86_64、ROS 2 Humble、Python 3.10。

包含：

- `zekeep_bringup`：模型、硬件参数、真机、RViz 和网页启动文件。
- `zekeepcontroller`：硬件抽象、状态机、ROS 服务和轨迹 Action。
- `zekeep_msgs`：自定义消息、服务和 Action。
- `zekeep_moveit_config`：MoveIt 配置。
- `zekeep_teach`：重力补偿拖动示教和轨迹复现。
- `zekeep_joystick`：六轴关节及末端手柄控制。
- `rebot_grasp`：相机、当前标定、检测和抓取。
- `zekeep_teleop`：独立 LeRobot 主从遥操作。
- `zekeeparm_SDK`：机械臂 SDK，仅保留 DM 硬件配置。
- GraspNet：安装者接受许可限制后从官方仓库获取源码，不随本仓库分发。

包含已有检查脚本。不包含语言控制、具身学习、数据采集模块、模型权重、日志、缓存和构建产物。

## 一键安装

联网执行。脚本会使用 `sudo apt`，但不会安装 NVIDIA 驱动或 CUDA Toolkit，不会连接相机或机械臂，也不会使能电机。

```bash
cd "$HOME/Desktop/Zekeeparm"
bash setup.sh
```

脚本安装 ROS、SDK、视觉环境、LeRobot 遥操作环境，并下载模型权重。GraspNet 源码和权重下载前必须确认仅用于本人或同机构内部非商业研究。中断后可直接重跑。

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

包含当前 `intrinsics.npz` 和 `hand_eye.npz`，不包含本地设备身份记录或标定历史备份。这些标定不能证明目标电脑或现场已经验证。设备、安装位置或场景变化后必须重新标定；完全未变化时也应先执行相机身份确认：

```bash
conda activate rebotarm
cd "$HOME/Desktop/Zekeeparm"
python src/rebot_grasp/scripts/confirm_calibration.py \
  --config src/rebot_grasp/config/default.yaml --confirm-same-installation
```

公开仓库的 `config/default.yaml` 中设备身份字段留空。确认前请在 `src/rebot_grasp/config/default.yaml` 填写实际 `camera.serial`、`robot.id` 和 `calibration.installation_id`。

该命令只打开相机，不连接或使能机械臂。完整硬件安全说明见各包 README。

## 许可

`third_party/graspnet-baseline` 仅限本人或同一机构单站点的非商业内部研究，不得转让或向第三方分发。其他组件按各自许可证使用。
