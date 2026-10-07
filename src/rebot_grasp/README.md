# rebot_grasp

Zekeep 直接 SDK 视觉抓取、RGB-D 相机与标定工具。

## 环境与启动

先按[安装说明](../../docs/INSTALL_PORTABLE.md)准备视觉环境、Orbbec SDK、模型权重和 CUDA 扩展，
再按[SDK 安装说明](../../docs/SDK_INSTALL.md)安装 reBotArm SDK。设置 `ZKEEP_WS` 为实际工作区绝对路径，
使用独立 `rebotarm` 环境，不把视觉 NumPy 约束安装进 ROS venv。

主抓取入口：

```bash
conda activate rebotarm
cd "$ZKEEP_WS/src/rebot_grasp"
python scripts/main.py --help
python scripts/main.py --config config/default.yaml \
  --target-class "red block"
```

`main.py` 直接占用官方 SDK 串口，不得与 `zekeepcontroller`、Gateway 或其他
SDK 进程同时运行。启动后会连接机械臂并执行准备位运动；先完成相机内参、手眼标定和现场检查，再运行上述命令。

## 当前配置与标定

主模型由 `robot.urdf_path` 指向 bringup URDF，SDK 路径由 `robot.repo_root` 解析。
这些相对路径从视觉项目目录解析，不要求固定用户名或工作区目录。
J1 `±2.58 rad`、J2/J3 `0–3.7 rad`、J4–J6 `±1.57 rad`，六轴方向均为 `-1`，与 ROS 一致。
准备位使用 `official_sdk.ready_joints`，应与 SRDF 的 `ready_pose` 一致。
joint6 零位法兰已按当前安装绕自身轴增加 `+90°`；准备位 TCP 姿态已按新 URDF FK 重算，
关节值和方向保持不变。

现有设备手眼标定已完成，以当前参数为准。仅换电脑且相机、机械臂、安装和场景不变时，
沿用当前内参与手眼文件，并在新机执行身份确认：

```bash
python scripts/confirm_calibration.py --config config/default.yaml --confirm-same-installation
```

该命令只打开相机，不连接机械臂，生成被 Git 忽略的 `identity.local.json`。
默认相机序列号以 YAML 为准，不能用于不同相机。身份记录缺失或不匹配时抓取会拒绝启动；
更换设备或安装后需要重新标定，不能跳过校验。

## 抓取参数

下降高度由 `config/default.yaml` 中的参数直接控制：

- `grasp_pipeline.grasp.pregrasp_offset_m` 是预抓点到抓取点的下降量。
- `grasp_pipeline.grasp.grasp_center_z_offset_m` 是最终 TCP 相对物体中心的 Z 修正。
- `grasp_pipeline.grasp.allow_shorter_descent_fallback` 默认为 `false`，因此不会
  在 IK 失败时静默换成更短的下降量。

低于安全线或发生碰撞的请求会明确失败，不会被静默改成更高的目标。

ROS 标准开爪目标为 `1.15 rad`，本包 `robot.gripper` 当前 `angle_open=1.30 rad`；
这是仍存在的配置差异，不代表两条执行路径已统一。

主抓取入口没有 `--dry-run` 参数，不能用它做无运动检查。
相机预览使用 `python scripts/camera_preview.py`，仅用于查看图像。

## 目录与诊断

标定工具位于 `calibration/`，相机驱动位于 `drivers/camera/`，坐标变换、抓取
候选筛选和执行安全检查位于 `utils/`。
位姿误差诊断入口为 `scripts/diagnose_pose_error.py`（直接 SDK）和
`scripts/diagnose_ros_pose_error.py`（ROS/MoveIt）；默认只规划，`--execute` 才执行运动。
