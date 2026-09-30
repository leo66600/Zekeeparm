# Six-axis 网页仿真器

网页加载当前 bringup 包的 URDF 和网格，不依赖作者用户名或固定工作区目录。

## 启动

完成[工作区安装](../../../README.md)，设置实际 `ZKEEP_WS` 后执行：

```bash
source "$ZKEEP_WS/tools/activate_ros.sh"
ros2 launch zekeep_bringup web.launch.py
```

网页地址 `http://127.0.0.1:3001`，ROS 连接 `ws://127.0.0.1:9090`。
Node 服务与 rosbridge 默认只监听本机；不能直接用另一台电脑访问这些端口。
该启动文件不启动真机驱动，实机控制需先启动驱动与所需的 MoveIt。

服务器使用 Node 内置模块，无需 `npm install`。仅查看模型时可独立运行：

```bash
node "$ZKEEP_WS/src/zekeep_bringup/web/server.js"
```

默认从服务器相邻的 bringup 目录读取模型；`ZKEEP_BRINGUP_SHARE` 可显式覆盖，
`ZKEEP_WS` 则指定源码工作区。若 source 安装包但仍设置 `ZKEEP_WS`，模型会读取该源码目录。
## 网页功能

- 精确显示 Six-axis URDF 外形和材质。
- 主模型跟随 `/zekeep/joint_states` 实际反馈。
- 半透明目标模型显示待发送姿态。
- 支持零位、Ready、关节滑块、夹爪宽度控制。
- 姿态预设先选择并预览，再通过“执行到所选姿态”按钮执行；控制锁打开时发送实机。
- Pose 输入支持 X/Y/Z（米）和时长。
- 未连接 ROS 或控制锁关闭时，Pose 在网页本地仿真。
- Pose 的 IK 姿态默认保持当前 TCP 姿态，避免单位四元数导致不可达。
- 连接 ROS 并打开控制锁后，Pose、规划轨迹和示教轨迹按对应按钮直接发送实机。
- 开始新示教录制时，如已有路径会明确询问是否覆盖；取消后旧路径保持不变。
- 关节和夹爪指令仍受控制锁保护。
- 网页不再显示桌面、仿真物料或视觉抓取面板。
- 碰撞几何会加载，但不在网页中显示。

## 坐标和夹爪

Pose 输入使用 ROS `base_link` 坐标系，单位米。网页模型转换为 Three.js 坐标：

```text
Three.js = (ROS X, ROS Z, -ROS Y)
```

J1 `±2.58 rad`、J2/J3 `0–3.7 rad`、J4–J6 `±1.57 rad`，与 ROS 模型一致。
夹爪界面开口 0–70 mm，当前网页按电机 0–1.30 rad 映射；标准 ROS 开爪目标为
1.15 rad，两者仍有差异。URDF `gripper_joint` 表示单指平移 0–0.035 m。

## 实机安全

默认只读，前端控制锁是操作保护，不是服务端认证。控制实机前确认：

1. ROS 控制器、关节反馈和 rosbridge 正常；
2. 急停可用、机械臂周围无人；
3. 网页反馈姿态与实机一致；
4. 勾选“允许网页向真实机械臂发控制”。打开后运动按钮会直接下发实机。
