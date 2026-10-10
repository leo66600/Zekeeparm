# zekeep_grasp

Zekeep 直接 SDK 视觉抓取、RGB-D 相机与标定工具。

## 环境与启动

先按[安装说明](../../README.md#一键安装)准备视觉环境、Orbbec SDK、模型权重和 CUDA 扩展，
再按[SDK 安装说明](../../docs/SDK_INSTALL.md)安装 reBotArm SDK。设置 `ZKEEP_WS` 为实际工作区绝对路径，
使用独立 `rebotarm` 环境，不把视觉 NumPy 约束安装进 ROS venv。

主抓取入口：

```bash
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/rebotarm"
cd "$ZKEEP_WS/src/zekeep_grasp"
python scripts/main.py --help
python scripts/main.py --config config/default.yaml \
  --target-class "red block"
```

`main.py` 直接占用官方 SDK 串口，不得与 `zekeepcontroller`、Gateway 或其他
SDK 进程同时运行。启动后会连接机械臂并执行准备位运动；先完成相机内参、手眼标定和现场检查，再运行上述命令。

## 当前配置与标定

主模型由 `robot.urdf_path` 指向 bringup URDF，SDK 路径由 `robot.repo_root` 解析。
这些相对路径从视觉项目目录解析，不要求固定用户名或工作区目录。
J1 `±2.58 rad`、J2 `0–3.7 rad`、J3 `-0.01–3.7 rad`、J4–J6 `±1.57 rad`，六轴方向均为 `-1`，与 ROS 一致。
准备位使用 `official_sdk.ready_joints`，应与 SRDF 的 `ready_pose` 一致。
joint6 零位法兰已按当前安装绕自身轴增加 `+90°`；准备位 TCP 姿态已按新 URDF FK 重算，
关节值和方向保持不变。

## 手眼标定文件与参数修改位置

**随包提供的标定文件和参数来自原设备安装，不能直接用于另一套设备。**
更换相机、机械臂、相机安装位置或末端参考坐标系后，必须按实际设备修改配置，
重新采集手眼标定结果，并更新本地身份记录。修改 YAML 参数不能代替重新标定。
仅更换电脑或目录名称，且设备、安装与场景完全不变时，可以沿用有效标定。

### 需要更新的文件

以下路径相对于 `src/zekeep_grasp/`。当前相机类型为 `orbbec_gemini2`；
读取目录由 `config/default.yaml` 中的 `camera.type` 决定，即 `config/calibration/<camera.type>/`。

| 修改位置 | 内容与更新方式 |
| --- | --- |
| [config/calibration/orbbec_gemini2/hand_eye.npz](config/calibration/orbbec_gemini2/hand_eye.npz) | 手眼外参。重新运行 `scripts/collect_handeye_eih.py` 生成，不要只改文件名或手工填写猜测矩阵。 |
| [config/calibration/orbbec_gemini2/intrinsics.npz](config/calibration/orbbec_gemini2/intrinsics.npz) | 当前相机彩色流的内参、畸变和分辨率。更换相机或改变成像配置后，通过 `scripts/save_orbbec_intrinsics.py` 更新。 |
| [config/calibration/orbbec_gemini2/identity.local.json](config/calibration/orbbec_gemini2/identity.local.json) | 绑定相机、机械臂、安装、标定文件和配置的本地记录。完成标定和现场核对后，用 `scripts/confirm_calibration.py` 重新生成，不要手改记录来绕过校验。 |
| [config/calibration/orbbec_gemini2/auto_poses.yaml](config/calibration/orbbec_gemini2/auto_poses.yaml) | 自动采集使用的姿态、参考坐标系和起始关节姿态。自动模式前需按当前模型和现场核对轨迹、标定板可见性及碰撞；不要直接照搬原安装的姿态。手动模式不读取此文件。 |

`hand_eye.npz` 中的 `T_result` 是 4×4 相机到末端参考坐标系的变换，平移单位为米。
当前为眼在手上模式 `mode=eye_in_hand`，`reference_frame` 必须与
`robot.end_effector_frame` 一致，当前值为 `gripper_base`。
不能把相机到 `base_link`、`link6` 或其他 TCP 的矩阵直接作为当前结果使用。

### 需要修改的参数

主要修改文件：[config/default.yaml](config/default.yaml)。按下列字段定位：

| 参数位置 | 应填写或核对的内容 |
| --- | --- |
| `camera.type`、`camera.serial` | 实际相机驱动类型和序列号；不能沿用另一台相机的序列号。 |
| `camera.color_width`、`camera.color_height`、`camera.fps`、`camera.color_distortion_mode` | 实际彩色流设置，必须与保存的内参匹配。畸变模式支持 `factory` / `zero`，按当前图像是否已校正选择，不能为消除报错随意切换。 |
| `camera.preset_json` | 相机预设文件路径，当前为 [config/camera/orbbec_gemini2_viewer.json](config/camera/orbbec_gemini2_viewer.json)。曝光等控制由此 JSON 提供；需要调整时修改或重新导出该文件。 |
| `robot.id`、`calibration.installation_id` | 实际机械臂身份和本次安装标识。重新安装相机或改变现场后更新安装标识，不使用电脑用户名代替设备身份。 |
| `calibration.target_type`、`calibration.charuco.*`、`calibration.aruco.*` | 与实际标定板匹配的类型、行列数、字典和尺寸。ChArUco 修改 `squares_x`、`squares_y`、`square_length_m`、`marker_length_m`、`dict_id`；ArUco 修改 `marker_length_m`、`dict_id`、`target_marker_id`。长度单位为米，按打印后的实测尺寸填写。 |
| `robot.urdf_path`、`robot.end_effector_frame`、`robot.joint_mapping`、`official_sdk.urdf` | 实际模型、手眼参考坐标系和关节标定。方向、零点及限位需与 [ROS 硬件配置](../zekeep_bringup/config/zekeep_hardware.yaml) 一致；两份 URDF 的几何和坐标系需一致。模型或零点改变后，应重新验证手眼标定。 |
| `calibration.hand_eye_compensation_m.x/y/z` | 已量测确认的残差补偿，单位为米，按 `base_link` 轴向平移；初始建议全部为 `0`。不能用全局补偿掩盖错误外参、坐标系或抓取姿态。 |
| `calibration.hand_eye_method`、`calibration.charuco.min_corners`、`calibration.charuco.max_reprojection_rmse_px`、`calibration.quality.*` | 求解方法与质量门槛。当前使用 `PARK`，重投影 RMSE 上限 `1 px`、平移闭合 RMSE 上限 `0.010 m`、旋转闭合 RMSE 上限 `3°`。结果超限时重新采样排查，不通过放宽门槛接受错误结果。 |

工作台高度和运动范围也需匹配现场：在同一 YAML 中核对 `safety.table_z_m`、
`official_sdk.workspace`、`official_sdk.min_tcp_z_m` 和 `place.joint_target_rad`。
这些是安全和放置参数，不能用来修正错误手眼外参。

### 按颜色放置

[config/default.yaml](config/default.yaml) 的 `place.class_targets` 将检测类别映射到
`place.named_joint_targets_rad` 中的已示教点位，六个值依次为 ROS `joint1`–`joint6`，单位为弧度：

| 检测类别 | 放置点 |
| --- | --- |
| `red block` | `red_place`，红色区 |
| `blue block` | `blue_place`，蓝色区 |
| `green block` | `green_place`，绿色区 |
| 其他未配置类别 | `place.joint_target_rad`，原普通放置区 |

三个分区点位来自 [ready_3poses_20261009_095828.json](../../ready_3poses_20261009_095828.json)，
并同步记录在 [MoveIt 命名姿态](../zekeep_moveit_config/config/zekeep.srdf) 中。
修改分区位置时更新 `place.named_joint_targets_rad` 和对应 SRDF `group_state`；
修改类别归属时更新 `place.class_targets`。放置配置也参与 `identity.local.json` 身份校验，
修改后须核对现场并按下文重新确认身份，不能删除校验或手改哈希。

网页 RGB-D 取帧沿用 `safety.capture_frame_retries`（当前重试 3 次）和
`safety.capture_frame_retry_interval_s`（当前 `0.05 s`）。偶发缺帧可恢复；持续失败会报告相机底层原因。
重试计入采集时长，超过 `0.25 s` 或机械臂移动时，图像仍只供预览，不生成可抓取目标。

### 物块颜色确认

当前 YOLOE 使用开放词表类别，模型标签可能把红块判成紫块。
SDK 和网页共用的 `utils/yolo_utils.py:detect_objects` 在分割掩膜内部复核 HSV 像素颜色：
占比达到 `yolo.block_color_min_fraction`（当前 `0.60`）且颜色明确时，修正物块类别；
原本带颜色标签、但混色或无法确认颜色的候选被剔除，不据此执行抓取。
校正后同色且高度重叠的重复框合并，避免把同一块物体当成多个目标。

灰色块还需确认低饱和度和有效亮度；一般物品类别保持原识别方式。
置信度阈值仍为 `detection.conf_threshold=0.30`，不会为补漏检而自动降低。
颜色占比参数须大于 `0.5` 且不超过 `1.0`；逆光、彩色照明、遮挡或掩膜错误仍可能造成漏检，
应优先改善光照、相机预设和物体可见范围，不能把此复核视为对所有现场的颜色识别保证。

### 修改后重新生成标定

先备份现有 `hand_eye.npz`、`intrinsics.npz`、`identity.local.json` 和 `config/default.yaml`，
再修改上述参数。设置 `ZKEEP_WS` 为实际工作区路径，进入视觉环境：

```bash
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/rebotarm"
cd "$ZKEEP_WS/src/zekeep_grasp"
python scripts/save_orbbec_intrinsics.py --config config/default.yaml
```

内参脚本只打开相机，不连接机械臂，并自动备份已有 `intrinsics.npz`。
以下手眼采集需要已启动 ROS 驱动和现场监督，`--manual` 会请求重力补偿：

```bash
source "$ZKEEP_WS/install/setup.bash"
python scripts/collect_handeye_eih.py --manual
```

保持标定板固定，手动改变末端位置和方向，静止后按 Enter 采样，输入 `c` 或 `q` 求解。
建议采集至少 15 个覆盖不同位置和旋转的有效样本。合格结果写入 `hand_eye.npz`；
质量不合格的结果写入 `hand_eye_rejected.npz`，不会替换已有手眼文件。
不加 `--manual` 会使用 `auto_poses.yaml` 自动运动，必须先完成轨迹和现场检查。
采集脚本固定读取 `config/default.yaml`，不支持 `--config` 参数。

确认新标定结果确实属于当前设备、安装和场景后，更新身份记录：

```bash
python scripts/confirm_calibration.py --config config/default.yaml --confirm-same-installation
```

该命令只打开相机，不连接机械臂，生成被 Git 忽略的 `identity.local.json`；
它只记录确认结果，不重新计算外参，也不证明标定精度。
身份记录缺失或不匹配时抓取会拒绝启动，不能跳过校验。

## 抓取参数

下降高度由 `config/default.yaml` 中的参数直接控制：

- `grasp_pipeline.grasp.pregrasp_offset_m` 是预抓点到抓取点的下降量。
- `grasp_pipeline.grasp.grasp_center_z_offset_m` 是最终 TCP 相对物体中心的 Z 修正。
- `grasp_pipeline.grasp.allow_shorter_descent_fallback` 默认为 `false`，因此不会
  在 IK 失败时静默换成更短的下降量。

低于安全线或发生碰撞的请求会明确失败，不会被静默改成更高的目标。

ROS 与直接 SDK 的开爪目标、硬限位统一为 `0–1.35 rad`。
线性映射为 `1.45 rad = 70 mm`，最大允许开口约 `65.17 mm`，
URDF 单指行程上限约 `0.0325862 m`；超过上限的请求和抓取候选被拒绝。
ROS 主 URDF 与本包 `config/sixaxis.urdf` 内容一致，质量、惯量和关节几何一致；
保留 SDK 使用的 `official_tcp`，六轴 IK 在内存中固定夹指关节。
模型文件或现场配置更新后，原 `identity.local.json` 会失配；核对现场后按上文重新确认，
不要删除身份校验或自动改写确认记录。

主抓取入口没有 `--dry-run` 参数，不能用它做无运动检查。
相机预览使用 `python scripts/camera_preview.py`，仅用于查看图像。

## 目录与诊断

### SDK 重力补偿拖动测试

`diagnose_joint_tracking.py` 的普通关节测试保持启动时的固定目标；拖到其他角度后会产生位置回拉。
手动检查重力补偿时使用 `--control-mode mit --gravity-follow`。此模式只在内存中将六轴位置增益设为零，
保留所选配置的阻尼和重力补偿系数，不发送轨迹或自动回准备位，不修改正式配置。
应使用经过现场核对的低阻尼测试配置，例如本机的 `config/gravity_test.local.yaml`，不要直接套用高增益运动配置。

```bash
python scripts/diagnose_joint_tracking.py --config config/gravity_test.local.yaml \
  --control-mode mit --gravity-follow --execute
```

该命令会使能电机，需要全程监督、可靠支撑、空载夹爪和可用急停；不提供碰撞避免或停止辅助。
在允许限位内小范围拖动，逐步减小托扶力并保留接住能力，观察持续下沉、上漂或抖动。
终端每 0.5 秒显示 J2 角度，CSV 持续保存六轴角度、速度、计算的模型补偿力矩与电机反馈力矩；
计算力矩不是逐条发送确认，电机反馈力矩也不等于外部负载真值。
此模式持续运行到操作者托稳后按 Enter，再失能退出；不会在普通测试的 `--duration` 时间后自动停止。
异常时先用现场急停处理并保持支撑。省略 `--execute` 仅检查配置，不连接硬件。

标定工具位于 `calibration/`，相机驱动位于 `drivers/camera/`，坐标变换、抓取
候选筛选和执行安全检查位于 `utils/`。
位姿误差诊断入口为 `scripts/diagnose_pose_error.py`（直接 SDK）和
`scripts/diagnose_ros_pose_error.py`（ROS/MoveIt）；默认只规划，`--execute` 才执行运动。
