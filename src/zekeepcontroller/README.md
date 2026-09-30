# zekeepcontroller

六轴机械臂真机驱动，负责电机连接、坐标映射、反馈、轨迹执行和 ROS 服务。
同一设备只允许一个驱动进程；直接 SDK 抓取与 LeRobot 遥操作不能同时占用该设备。

## 环境与启动

先按[安装说明](../../docs/INSTALL_PORTABLE.md)准备 ROS，再按[SDK 安装说明](../../docs/SDK_INSTALL.md)
安装 SDK 并构建。
设置 `ZKEEP_WS` 为实际工作区绝对路径：

```bash
source "$ZKEEP_WS/tools/activate_ros.sh"
ros2 launch zekeep_bringup driver.launch.py channel:=/dev/ttyACM0
```

默认不自动使能。另开终端加载同一环境并运行
`ros2 launch zekeep_moveit_config hardware.launch.py arm_namespace:=zekeep`。

硬件参数来自 `../zekeep_bringup/config/zekeep_hardware.yaml`，节点默认参数来自
同目录 `driver_params.yaml`。六轴方向均为 `-1`，J1 `±2.58 rad`、J2/J3 `0–3.7 rad`、
J4–J6 `±1.57 rad`；标准手臂控制模式为 `posvel`。修改后需重新构建并重启。

## 常用接口

默认命名空间 `/zekeep`；消息定义见 [zekeep_msgs](../zekeep_msgs/README.md)。

| 接口 | 用途 |
| --- | --- |
| `joint_states`、`arm_status` | 反馈与驱动状态 Topic |
| `follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |
| `enable`、`disable` | 显式使能、失能；`std_srvs/srv/Trigger` |
| `stop` | 中断运动并尝试保持；不是失能 |
| `safe_home` | 校验路径后回零 |
| `gravity_compensation/start`、`stop`、`status` | 拖动补偿服务 |
| `gripper/set`、`gripper/open`、`gripper/close` | 夹爪服务 |
| `move_to_pose_ik`、`set_zero` | IK 查询、硬件置零 |

```bash
ros2 topic echo /zekeep/arm_status --once
ros2 service call /zekeep/enable std_srvs/srv/Trigger '{}'
```

置零会改变硬件标定，不是普通回零。独立遥操作沿用自己的主从映射。

## 退出与异常

已使能时正常退出先回零，成功后失能断开。回零需 MoveIt 场景校验，
先退出驱动，再关闭 MoveIt。失败时中止退出并尝试保持，修复原因后重试。
`disable_after_safe_home=false` 不允许已使能手臂直接断开。
显式 `disable` 会释放力矩，调用前需支撑机械臂。

标准配置已授权普通运动与人工监督重力补偿；`dynamics_verified=false`，
普通轨迹重力前馈关闭。授权标记不代表现场验证结果。
