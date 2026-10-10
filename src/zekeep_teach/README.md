# zekeep_teach

通过 ROS 进行人工监督拖动示教、离散点记录和低速轨迹复现。
电机连接由 `zekeepcontroller` 独占，示教程序不另开串口。

## 启动

先完成[工作区构建](../../README.md)。每个终端设置实际工作区路径，
例如 `export ZKEEP_WS="$HOME/Desktop/Zekeeparm"`。按顺序使用三个独立终端。

终端 1，启动唯一的真机驱动，默认不使能：

```bash
cd "$ZKEEP_WS"
source tools/activate_ros.sh
ros2 launch zekeep_bringup driver.launch.py
```

终端 2，启动 MoveIt，供安全回零与退出时校验路径：

```bash
cd "$ZKEEP_WS"
source install/setup.bash
ros2 launch zekeep_moveit_config hardware.launch.py arm_namespace:=zekeep
```

终端 3，检查状态并启动示教：

```bash
cd "$ZKEEP_WS"
source install/setup.bash
ros2 topic echo /zekeep/arm_status --once
ros2 topic echo /zekeep/joint_states --once
ros2 run zekeep_teach teach_replay
```

默认参数位于 `config/default.yaml`。用 `ros2 run zekeep_teach teach_replay --help`
查看命令行选项。命名空间须与驱动一致，默认为 `zekeep`。

## 按键

| 按键 | 行为 |
| --- | --- |
| `T` | 进入重力补偿拖动 |
| 空格 | 记录离散点 |
| `U` | 撤销最后一个离散点 |
| `C` | 开始/停止连续记录；覆盖已有轨迹前询问 |
| `1` | 复现离散点 |
| `2` | 复现连续轨迹 |
| `S` | 保存 |
| `Q` | 请求安全退出、回零和失能 |

复现进行时只接受 `Q`。进入重力补偿可能使能电机，应先支撑机械臂。
标准配置允许人工监督补偿，但动力学尚未标记为验证完成。
退出期间保持驱动和 MoveIt 运行；回零失败应处理故障。
示教退出成功后再退出驱动，待驱动退出成功后最后关闭 MoveIt。

位置采用 ROS 弧度制，遵循当前驱动限位和方向，与独立 LeRobot 角度制配置不同。
详细流程见[拖动示教说明](docs/ros-hand-guided-teaching.md)。
