# zekeep_joystick

六轴关节和末端笛卡尔遥操作包，默认使用实测的 Zikway HID gamepad 映射。节点不会调用 `/zekeep/enable`；电机使能必须手动完成。

## 控制映射

- 按住 `LB`：允许运动；松开后保持当前位置。
- 左摇杆横/纵：J1/J2。
- 右摇杆纵/横：J3/J4。
- 十字键横/纵：J5/J6。
- 按住 `LB` 时，`A` 力矩夹持（接触后转保持力矩），`B` 打开夹爪。

按 A/B 后，节点先停止关节伺服并确认保持成功，再执行夹爪动作。夹爪动作期间暂停关节遥操作，完成后松开并重新按住 LB 恢复。停止失败时不会发送夹爪命令；动作失败原因会显示在终端。

A 调用驱动的 `/zekeep/gripper/grasp` Action。检测到物体后，驱动持续提供保持力矩，恢复关节控制后仍保持；B 打开并退出夹持。动作执行期间 A/B 不重复触发，等待结果后再按。
`grasp_closing_torque`、`grasp_hold_torque`（单位 N·m）和 `grasp_timeout`（秒）为 0 时继承驱动配置。当前标准配置为闭合 1.0 N·m、保持 0.30 N·m、超时 5 秒；驱动限制力矩上限为 1.5 N·m。
接触判据为速度绝对值不超过 0.05 rad/s、反馈力矩绝对值至少 0.10 N·m，并且已闭合运动至少 0.30 rad；闭合硬限位附近判为空抓。先用 B 打开再夹持，较宽物体若无法达到最小运动量可能超时。这些阈值来自驱动硬件配置，需按物体和反馈调试；电机力矩不等于指尖夹持力。

## 末端笛卡尔模式

关节控制入口保持不变。末端控制使用独立入口，两者不能同时运行：

```bash
ros2 launch zekeep_joystick cartesian_joystick.launch.py
```

手动使能后按住 LB：左摇杆上下/左右控制基坐标系 X/Y，右摇杆上下控制 Z，十字键左右控制 pitch、上下控制 roll，右摇杆左右控制 yaw；LB+A 力矩夹持，LB+B 打开夹爪。单按 `X` 先停止伺服，再调用 `/zekeep/safe_home` 自动回安全零位；该驱动动作也会关闭夹爪，且必须同时运行 MoveIt 做全路径碰撞校验。`LB+Y` 从末端模式切换到关节位置模式；松开 LB 后重新按住，映射变为左摇杆 J1/J2、右摇杆 J3/J4、十字键 J5/J6。此切换单向生效，恢复末端模式需重启手柄节点。

夹爪、回零或模式切换前都会先停止末端伺服。动作完成后松开再按 LB。松开 LB 保持。目标是 `base_link` 下的 `link6`；速度、方向、按钮和 IK 连续性阈值见 `config/zikway_cartesian.yaml`。录制示范时只在一段 episode 结束后使用 X，避免把未记录的回零动作混入状态序列。

关节入口参数在 `config/zikway.yaml` 修改，末端入口参数在 `config/zikway_cartesian.yaml` 修改。
`LB` 只是按住运行开关，不会使能或失能电机。

## 构建与启动

设置 `ZKEEP_WS` 为工作区实际路径。每个 ROS 终端加载同一工作区：

首次使用或修改代码后构建：

```bash
cd "$ZKEEP_WS"
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
colcon build --packages-select zekeep_joystick --symlink-install
source install/setup.bash
```

终端 1 启动唯一的真机驱动：

```bash
cd "$ZKEEP_WS"
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
ros2 launch zekeep_bringup driver.launch.py channel:=/dev/ttyACM0
```

关节控制不需要 MoveIt。终端 2 手动使能并启动：

```bash
cd "$ZKEEP_WS"
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
ros2 service call /zekeep/enable std_srvs/srv/Trigger '{}'
ros2 launch zekeep_joystick joystick.launch.py
```

末端笛卡尔控制及 X 回安全零位需要 MoveIt。终端 2 启动 MoveIt：

```bash
cd "$ZKEEP_WS"
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
ros2 launch zekeep_moveit_config hardware.launch.py arm_namespace:=zekeep use_rviz:=false
```

终端 3 手动使能并启动末端手柄：

```bash
cd "$ZKEEP_WS"
source tools/activate_ros.sh
export ROS_LOCALHOST_ONLY=1
ros2 service call /zekeep/enable std_srvs/srv/Trigger '{}'
ros2 launch zekeep_joystick cartesian_joystick.launch.py
```

关节控制与末端控制只能二选一，不能同时启动。两个启动文件都会启动 `joy_node`。末端控制不要另启 `zekeep_bringup bringup.launch.py`，`hardware.launch.py` 已提供 TF。

## 手柄配置与排错

Zikway 的实体 X/Y 分别为按钮 3/2，LB 为按钮 6；右摇杆横/纵为轴 2/3，轴 4/5 为扳机。
其他手柄先核对 `/joy`，可通过 `config:=/绝对路径/映射.yaml` 选择配置；`config/xbox.yaml` 保留给旧布局。

以上命令假设所有 ROS 节点运行在本机，因此每个终端都设置 `ROS_LOCALHOST_ONLY=1`。
如果需要连接其他电脑上的 ROS 节点，所有相关终端应使用一致的网络设置。

如果终端反复显示 `cartesian joystick control active` 和 `deadman released`，检查 `/joy` 是否有重复发布者：

```bash
ros2 topic info /joy --verbose
```

正常只应有一个 `/joy` 发布者。多个发布者可能交替发送不同的 LB 状态。先停止多余的手柄或仿真节点；如果其他电脑的 ROS 节点混入本机图，停止现有 ROS 进程，在每个终端按上述方式设置 `ROS_LOCALHOST_ONLY=1` 后重启。必要时再运行 `ros2 daemon stop` 和 `ros2 daemon start`，并复查发布者数量。

首次连接不同型号手柄时先检查轴号：

```bash
ros2 topic echo /joy
```

真机前先悬空或降低 `max_speeds`，逐轴确认方向。手柄断连或 `/joy` 超过 0.3 秒无更新时，节点请求保持；驱动自身还有 0.2 秒伺服看门狗。
