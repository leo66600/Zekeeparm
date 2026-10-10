# zekeep_msgs

驱动、示教客户端和网页桥接共用的 ROS 2 接口定义。

| 目录 | 接口 |
| --- | --- |
| `msg/` | `ArmStatus`、`JointMitCmd`、`JointMotorState`、`JointPosVelCmd` |
| `srv/` | `GripperCommand`、`MoveToPoseIK`、`SetGripper`、`SetZero`、`WebTeachCommand`、`WebTaskLease` |
| `action/` | `GripperGrasp`、`MoveToPose`、`WebGrasp` |

字段和单位以 `.msg`、`.srv`、`.action` 文件为准。
轨迹执行使用标准 `control_msgs/action/FollowJointTrajectory`；使能、失能、停止、回零
使用 `std_srvs/srv/Trigger`，这些接口不在本包重复定义。

## 构建与查看

先完成[环境安装](../../README.md#一键安装)，设置实际 `ZKEEP_WS`：

```bash
cd "$ZKEEP_WS"
source "$ZKEEP_WS/tools/activate_ros.sh"
python -m colcon build --base-paths src --packages-select zekeep_msgs
source install/setup.bash
ros2 interface show zekeep_msgs/msg/ArmStatus
ros2 interface show zekeep_msgs/srv/SetGripper
ros2 interface show zekeep_msgs/action/MoveToPose
```

默认接口实例位于 `/zekeep`，服务表见[控制器说明](../zekeepcontroller/README.md)。
修改定义后需同步修改生产者与消费者，重新构建依赖包并重启相关节点。
