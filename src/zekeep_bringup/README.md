# zekeep_bringup

六轴机械臂的启动文件、URDF、网格、硬件配置、RViz 和网页资源包。
先按[安装说明](../../docs/INSTALL_PORTABLE.md)安装依赖并构建。
每个终端设置 `ZKEEP_WS` 为实际工作区绝对路径，再加载：

```bash
source "$ZKEEP_WS/tools/activate_ros.sh"
```

## 当前模型与标定

以当前参数及现有设备标定为准。主模型为 `description/urdf/sixaxis.urdf`，
来源于 0918 CAD，已删除 `rod_left_link`、`rod_right_link` 及对应网格。
夹爪使用 `gripper_base`、`left_link`、`right_link`，右指 mimic 左指。

ROS、视觉 SDK 和网页范围：J1 `[-2.58, 2.58] rad`，J2/J3 `[0, 3.7] rad`，
J4–J6 `[-1.57, 1.57] rad`。ROS/视觉电机到模型的六轴方向均为 `-1`。
独立 LeRobot 遥操作保留原有角度制标定，不套用这组方向。
当前 joint6 零位法兰相对旧坐标系绕 joint6 自身轴增加 `+90°`，两份 URDF 已同步。
因此准备位 TCP 姿态的 roll 已重新计算；准备位关节值和限位不变。
当前手眼标定已完成；仅换电脑且设备、安装、场景未变时可沿用，见[视觉说明](../rebot_grasp/README.md)。

标准配置 `config/zekeep_hardware.yaml` 使用 `posvel` 控制，
`motion_authorized=true`、`supervised_trial_authorized=true`，启动默认不使能。
`dynamics_verified=false`，普通轨迹重力前馈关闭。手眼标定完成不等于动力学已验证。
配置在启动时读取；修改源码配置后需重新构建并重启驱动。

夹爪电机硬限位 `[0, 1.57] rad`，标准 ROS 开爪目标 `1.15 rad`，最大开口配置
`0.070 m`。电机角度与 URDF 单指平移量不是同一单位。

## 启动入口

| 入口 | 行为 |
| --- | --- |
| `driver.launch.py` | 真机驱动，不启动 MoveIt |
| `bringup.launch.py` | 真机驱动、robot_state_publisher 和可选 RViz，不启动 MoveIt |
| `carry.launch.py` | MoveIt 和固定点搬运节点，真机驱动需独立启动 |
| `web.launch.py` | 本机网页及 rosbridge，真机驱动需独立启动 |
| `carry_student.launch.py` | 含填空占位符的教学模板，不能直接运行 |

真机驱动与 MoveIt 分别在两个已加载环境的终端启动：

```bash
ros2 launch zekeep_bringup driver.launch.py channel:=/dev/ttyACM0
```

```bash
ros2 launch zekeep_moveit_config hardware.launch.py arm_namespace:=zekeep
```

确认现场条件后显式使能：

```bash
ros2 service call /zekeep/enable std_srvs/srv/Trigger '{}'
```

驱动正常退出需要 MoveIt 校验回零路径。先退出驱动，确认成功后再关闭 MoveIt。
回零失败时控制器保留连接并尝试保持，处理故障后重试。

## 固定点搬运

```bash
ros2 launch zekeep_bringup carry.launch.py sim:=true auto_start:=false
# 在另一个已加载环境的终端，待 MoveIt 就绪后触发
ros2 service call /carry/start std_srvs/srv/Trigger '{}'
```

真机使用 `sim:=false`，并先启动唯一的驱动；不要另开重复的 MoveIt 栈。
`sim` 默认 `false`，`auto_start` 默认 `false`，`use_rviz` 默认 `true`。
当前代码仍提供 `preview_only:=true`，仅规划而不执行机械臂和夹爪动作。

参数从当前安装包的 `config/carry_params.yaml` 读取，不绑定用户名或工作区路径。
`pick_pose.position.z` 是最终 `grasp_tcp` 绝对高度，`hover_height` 是实际下降量。
笛卡尔路径不完整时任务失败。

## 网页

```bash
ros2 launch zekeep_bringup web.launch.py
```

访问 `http://127.0.0.1:3001`，连接 `ws://127.0.0.1:9090`。
两项服务默认仅监听本机，详见[网页说明](web/README.md)。
