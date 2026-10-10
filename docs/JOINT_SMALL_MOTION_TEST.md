# 真机关节小幅运动测试

本文使用 `zekeepcontroller` 自带的 `MoveTo` 命令，每次只测试一个关节。
命令发送的是**绝对关节角度**，单位为弧度（rad），不是相对增量。

## 1. 测试前检查

- 确认急停可用，机械臂周围无人和障碍物。
- 固定底座，并准备在失去力矩时支撑机械臂。
- 关闭其他会占用机械臂的程序，例如网页任务、抓取、手柄或 LeRobot 遥操作。
- 第一次测试建议步长不超过 `0.05 rad`（约 `2.9°`），运动时间使用 `3 s`。

当前关节范围：

| 关节 | 允许范围（rad） |
| --- | --- |
| `joint1` | `-2.58` 至 `2.58` |
| `joint2` | `0.00` 至 `3.70` |
| `joint3` | `-0.01` 至 `3.70` |
| `joint4` | `-1.57` 至 `1.57` |
| `joint5` | `-1.57` 至 `1.57` |
| `joint6` | `-1.57` 至 `1.57` |

不要执行本文中的示例角度，除非示例角度确实接近机械臂当前角度。

## 2. 启动真机驱动

打开终端 1：

```bash
cd "$HOME/Desktop/Zekeeparm"
source install/setup.bash
ros2 launch zekeep_bringup driver.launch.py channel:=/dev/ttyACM0
```

驱动启动后保持此终端运行。默认不会自动使能电机。

## 3. 启动 MoveIt

打开终端 2：

```bash
cd "$HOME/Desktop/Zekeeparm"
source install/setup.bash
ros2 launch zekeep_moveit_config hardware.launch.py arm_namespace:=zekeep
```

`MoveTo` 本身直接调用驱动的轨迹 Action。MoveIt 用于驱动退出时校验回零路径，
因此测试结束前不要先关闭 MoveIt。

## 4. 检查连接和反馈

打开终端 3：

```bash
cd "$HOME/Desktop/Zekeeparm"
source install/setup.bash
ros2 action info /zekeep/follow_joint_trajectory
ros2 topic echo /zekeep/arm_status --once
ros2 topic echo /zekeep/joint_states --once
```

确认：

- Action 信息中存在服务端；
- `arm_status` 没有错误；
- `joint_states` 同时显示 `name` 和 `position`，并包含 `joint1` 至 `joint6`。

`position` 数组和 `name` 数组按相同下标对应。记录准备测试关节的当前角度，记为
`q0`。

## 5. 使能机械臂

确认现场安全后执行：

```bash
ros2 service call /zekeep/enable std_srvs/srv/Trigger '{}'
```

再次检查状态：

```bash
ros2 topic echo /zekeep/arm_status --once
```

确认返回内容中的 `enabled: true`，且 `error_codes` 为空。

## 6. 单关节小幅运动

选择目标值：

```text
q_target = q0 + 0.05 rad
```

如果加 `0.05 rad` 会接近关节上限，则改为减 `0.05 rad`。`joint2` 和
`joint3` 不要使用负目标值。

以下示例假设 `joint1` 当前角度 `q0 = 0.32 rad`，因此目标设为
`0.37 rad`：

```bash
ros2 run zekeepcontroller MoveTo \
  --joint joint1 \
  --position 0.37 \
  --duration 3
```

成功时末尾应显示：

```text
error_code=0
```

检查实际位置：

```bash
ros2 topic echo /zekeep/joint_states --once
```

然后返回测试前角度 `q0`：

```bash
ros2 run zekeepcontroller MoveTo \
  --joint joint1 \
  --position 0.32 \
  --duration 3
```

测试其他关节时，只替换 `--joint` 和根据该关节当前反馈计算出的
`--position`。一次只测试一个关节。

## 7. 异常停止

运动异常时，另开一个已进入工作区并执行 `source install/setup.bash` 的终端：

```bash
ros2 service call /zekeep/stop std_srvs/srv/Trigger '{}'
```

该命令停止运动并保持当前位置，不会释放电机力矩。若软件停止无效，使用现场急停。

不要把 `disable` 当作普通停止命令。`disable` 会释放力矩，机械臂可能下落。

## 8. 正常结束

1. 确认机械臂、夹爪和回零路径无障碍物。
2. 在终端 1 按 `Ctrl+C` 关闭驱动。
3. 等待驱动完成 MoveIt 碰撞校验、回到 `safe_home`、失能并断开。
4. 驱动成功退出后，再在终端 2 按 `Ctrl+C` 关闭 MoveIt。

正常退出会回到配置的 `safe_home`，可能比本教程的测试步长大。回零失败时驱动会保留连接
和保持力矩；不要强制结束进程。排除报错后，再次在终端 1 按 `Ctrl+C` 重试退出。

## 常见问题

### `Package 'zekeepcontroller' not found`

当前终端没有加载工作区：

```bash
cd "$HOME/Desktop/Zekeeparm"
source install/setup.bash
```

### `joint_states not available`

检查驱动是否仍在运行、串口是否正确，以及 `/zekeep/joint_states` 是否存在：

```bash
ros2 topic list | grep /zekeep
```

### `follow_joint_trajectory action not available`

检查驱动是否启动成功：

```bash
ros2 action info /zekeep/follow_joint_trajectory
```

### `trajectory exceeds configured joint velocity limits`

增加运动时间，例如：

```bash
ros2 run zekeepcontroller MoveTo \
  --joint joint1 \
  --position 0.37 \
  --duration 5
```

仍需确认目标值接近当前值并位于关节范围内。
