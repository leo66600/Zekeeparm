# Zekeeparm Python SDK 安装

工作区随附 SDK 源码：`zekeeparm_SDK`。控制接口由 Python 提供；Pinocchio 等依赖包含 C++ 原生库。

SDK 仅保留 `config/rebotarm_dm.yaml`，默认直接读取 DM 配置。
模型与 ROS 主 URDF 内容一致，保留 `official_tcp`；直接 SDK 六轴 IK 固定夹指关节。
夹爪映射基准 `1.45 rad = 70 mm`，允许目标 `0–1.35 rad`，最大开口约 `65.17 mm`。
Python 导入统一为 `zekeeparm_SDK`；`RebotArm`、`RebotArmEndPose` 等类名保持不变。
旧环境迁移时，先执行 `python -m pip uninstall rebotarm-control-py`，再按下方步骤安装新包。

## 功能特性

- 电机连接、状态读取和控制循环。
- 关节正逆运动学。
- 基于 Pinocchio 的动力学计算与重力补偿。
- 末端控制和轨迹规划工具。
- YAML 硬件配置。

## 环境安装

ROS 驱动和视觉抓取使用独立 Python 环境，SDK 需分别安装到两者中。先设置工作区路径：

```bash
export ZKEEP_WS="/实际工作区路径"
cd "$ZKEEP_WS"
```

SDK 的 `pyproject.toml` 声明较宽的依赖范围。以下步骤由各环境的依赖清单管理版本，再以 `--no-deps` 安装本地 SDK，避免 pip 改动 NumPy、Pinocchio 或 MotorBridge 组合。

### ROS 驱动环境

目标为 Ubuntu 22.04、ROS 2 Humble 和 Python 3.10。完成 ROS 系统依赖安装后执行：

```bash
cd "$ZKEEP_WS"
/usr/bin/python3 -m venv --system-site-packages .venv-ros
source .venv-ros/bin/activate
export PYTHONNOUSERSITE=1
python -m pip install -r tools/requirements-ros-sdk.txt
python -m pip install --no-deps -e zekeeparm_SDK
source tools/activate_ros.sh
```

后续 ROS 构建和运行都使用 `source tools/activate_ros.sh`。脚本加载 ROS、`.venv-ros` 和对应的 Pinocchio 原生库。

### 视觉抓取环境

先按[工作区说明](../README.md)创建 `rebotarm` 环境并完成视觉依赖安装，再执行：

```bash
cd "$ZKEEP_WS"
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/rebotarm"
python -m pip install --no-deps -e zekeeparm_SDK
```

视觉环境使用 NumPy 1.26.4；ROS venv 使用 NumPy 2.2.6。不要 source ROS venv，也不要把视觉依赖装进 ROS venv。

### 独立遥操作环境

`zekeep_teleop` 不安装 reBotArm SDK。它使用独立的 `motorbridge==0.4.9`；ROS 与视觉环境使用 `motorbridge==0.5.1`。

## 安装验证

在目标环境激活后运行对应导入检查。命令只导入 SDK，不连接硬件：

ROS 环境：

```bash
source "$ZKEEP_WS/tools/activate_ros.sh"
python -c '
import rclpy, pinocchio, motorbridge, numpy
from zekeeparm_SDK.actuator import RebotArm
from zekeeparm_SDK.controllers import RebotArmEndPose
from zekeeparm_SDK.dynamics import compute_generalized_gravity
from zekeeparm_SDK.kinematics import compute_fk
'
```

视觉环境：

```bash
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/rebotarm"
python -c '
import pinocchio, motorbridge, numpy
from zekeeparm_SDK.actuator import RebotArm
from zekeeparm_SDK.controllers import RebotArmEndPose
from zekeeparm_SDK.dynamics import compute_generalized_gravity
from zekeeparm_SDK.kinematics import compute_fk
'
```

确认 Python 路径和安装版本：

```bash
python -c 'import sys; print(sys.executable)'
python -m pip show zekeeparm-sdk motorbridge pin
```

检查所有 SDK 模块、唯一 DM 配置和模型路径（不连接硬件）：

```bash
python -B tools/check_sdk.py
```

ROS/SDK 模型替换后的 FK 与六轴 IK 检查需在视觉环境执行（不连接硬件）：

```bash
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/rebotarm"
python -B tools/check_vision_model.py
```

## 项目使用

- ROS 真机驱动由 `zekeepcontroller` 调用 SDK；安装和启动步骤见[控制器说明](../src/zekeepcontroller/README.md)。
- 视觉抓取直接调用 SDK 并独占机械臂串口；用法见[视觉抓取说明](../src/zekeep_grasp/README.md)。
- ROS 驱动和直接 SDK 抓取不可同时连接同一机械臂。
- 串口权限使用 `dialout` 组，不要用 `chmod 777` 放宽设备权限。

## SDK 目录

```text
zekeeparm_SDK/
├── config/                  # SDK 硬件配置
└── zekeeparm_SDK/
    ├── actuator/            # 电机连接与控制循环
    ├── controllers/         # 末端控制器
    ├── dynamics/            # 动力学计算
    ├── kinematics/          # 正逆运动学
    └── trajectory/          # 轨迹工具
```
