# Zekeep reBotArm Python SDK 安装

安装脚本从 [Zekeep_control_py](https://github.com/leo66600/Zekeep_control_py) 的 `v0.1.0` 标签获取 SDK 到 `third_party/reBotArm_control_py`。该版本适配本工作区；Python 导入名仍为 `reBotArm_control_py`。SDK 仓库保留 Seeed 上游 fork 关系和历史；使用时须遵守上游条款。控制接口由 Python 提供；Pinocchio 等依赖包含 C++ 原生库。

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
python -m pip install --no-deps -e third_party/reBotArm_control_py
source tools/activate_ros.sh
```

后续 ROS 构建和运行都使用 `source tools/activate_ros.sh`。脚本加载 ROS、`.venv-ros` 和对应的 Pinocchio 原生库。

### 视觉抓取环境

先按[工作区说明](../README.md)创建 `rebotarm` 环境并完成视觉依赖安装，再执行：

```bash
cd "$ZKEEP_WS"
conda activate rebotarm
python -m pip install --no-deps -e third_party/reBotArm_control_py
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
from reBotArm_control_py.actuator import RebotArm
from reBotArm_control_py.controllers import RebotArmEndPose
from reBotArm_control_py.dynamics import compute_generalized_gravity
from reBotArm_control_py.kinematics import compute_fk
'
```

视觉环境：

```bash
conda activate rebotarm
python -c '
import pinocchio, motorbridge, numpy
from reBotArm_control_py.actuator import RebotArm
from reBotArm_control_py.controllers import RebotArmEndPose
from reBotArm_control_py.dynamics import compute_generalized_gravity
from reBotArm_control_py.kinematics import compute_fk
'
```

确认 Python 路径和安装版本：

```bash
python -c 'import sys; print(sys.executable)'
python -m pip show rebotarm-control-py motorbridge pin
```

## 项目使用

- ROS 真机驱动由 `zekeepcontroller` 调用 SDK；安装和启动步骤见[控制器说明](../src/zekeepcontroller/README.md)。
- 视觉抓取直接调用 SDK 并独占机械臂串口；用法见[视觉抓取说明](../src/rebot_grasp/README.md)。
- ROS 驱动和直接 SDK 抓取不可同时连接同一机械臂。
- 串口权限使用 `dialout` 组，不要用 `chmod 777` 放宽设备权限。

## SDK 目录

```text
third_party/reBotArm_control_py/
├── config/                  # SDK 硬件配置
└── reBotArm_control_py/
    ├── actuator/            # 电机连接与控制循环
    ├── controllers/         # 末端控制器
    ├── dynamics/            # 动力学计算
    ├── kinematics/          # 正逆运动学
    └── trajectory/          # 轨迹工具
```
