# zekeep_moveit_config

六轴机械臂 MoveIt 2 配置：运动学、规划器、碰撞矩阵、控制器映射和命名姿态。
环境与构建见[工作区 README](../../README.md)。每个终端设置实际 `ZKEEP_WS` 后加载：

```bash
cd "$ZKEEP_WS"
source install/setup.bash
```

## 无硬件演示

```bash
ros2 launch zekeep_moveit_config demo.launch.py
```

使用仿真控制，不连接电机串口。

## 真机

先在独立终端启动 `zekeep_bringup driver.launch.py`，再启动：

```bash
ros2 launch zekeep_moveit_config hardware.launch.py arm_namespace:=zekeep
```

此入口连接已有驱动，不额外启动真机驱动。默认通过
`/zekeep/follow_joint_trajectory` 执行轨迹；硬件模式不依赖根命名空间的
`/controller_manager`。驱动退出回零依赖 `/check_state_validity`，应最后关闭 MoveIt。

`arm_namespace` 同时绑定关节反馈与轨迹控制器。例如驱动与本入口均设置
`arm_namespace:=other_arm` 时，使用 `/other_arm/joint_states` 和
`/other_arm/follow_joint_trajectory`。配置 YAML 保留默认 `zekeep` 模板，启动时按参数替换；
不需要为每个命名空间复制配置。该参数只控制真机 MoveIt 的驱动接口，网页入口仍使用 `/zekeep`。

## 配置契约

- `config/zekeep.srdf`：`arm` 链从 `base_link` 到 `link6`；`gripper` 包含 `gripper_joint`。
- 命名姿态：`planning_initial`、`ready_pose`、`open`、`closed`。
- `ready_pose` 与视觉配置的 `official_sdk.ready_joints` 对齐，以关节值为准。
- J1 `[-2.58, 2.58] rad`，J2 `[0, 3.7] rad`、J3 `[-0.01, 3.7] rad`，J4–J6 `[-1.57, 1.57] rad`。
- `gripper_joint` 是平移关节，`open≈0.0325862 m`、`closed=0 m`，不是电机弧度。
- `kinematics.yaml`、`ompl_planning.yaml`、`joint_limits.yaml` 管理 IK、规划和限位。
- `moveit_controllers.yaml` 管理仿真执行接口，`moveit_hardware_controllers.yaml` 管理真机执行接口；命名空间须与驱动、客户端一致。
- URDF 使用 bringup 主模型；旧 rod 连杆已移除，碰撞豁免以当前 SRDF 为准。

## 固定点任务

主要搬运入口为 `zekeep_bringup carry.launch.py`，见[搬运说明](../zekeep_bringup/README.md)。
本包另有 `pick_and_place.launch.py`，使用独立的 `config/pick_place_params.yaml`：

```bash
ros2 launch zekeep_moveit_config pick_and_place.launch.py sim:=true auto_start:=false
```

该入口默认 `sim=true`、`auto_start=true`，与 carry 的默认值不同。
真机使用前先核对其独立点位参数，不要同时运行两个搬运任务。

## 排错

```bash
ros2 action info /zekeep/follow_joint_trajectory
ros2 topic echo /zekeep/joint_states --once
ros2 service list
```

轨迹服务端应唯一，反馈应持续更新。各终端使用相同工作区、`ROS_DOMAIN_ID` 和命名空间。
