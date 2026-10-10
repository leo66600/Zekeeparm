# zekeep_shadow：网页视觉与抓取后端

提供网页 RGB-D 检测、YOLO/GraspNet 感知和抓取后端。

- `fixed`：默认使用 OpenCV 检测红色物块，抓取使用已确认的物体尺寸与姿态。
- `yolo-graspnet`：使用 YOLO 分割和 GraspNet 规划抓取；需要对应环境、模型权重与 CUDA 扩展。

抓取任务由 `zekeep_teach` 协调，使用 MoveIt 规划和控制器任务接口。
网页和 LLM 自动感知仍依赖本包。

## 工作区与构建

设置实际工作区路径；以下为本机路径：

```bash
export ZKEEP_WS="$HOME/Desktop/Zekeeparm"
cd "$ZKEEP_WS"
source /opt/ros/humble/setup.bash
colcon build --base-paths src/zekeep_shadow --packages-select zekeep_shadow
source install/setup.bash
```

本命令只构建本包；首次安装及其余依赖构建见[工作区 README](../../README.md)。

## 网页只读预览

`web_vision` 节点复用 RGB-D 驱动和所选检测后端，默认发布
`/zekeep/web/camera/image_raw` 和 `/zekeep/web/vision/detections`。
该节点打开相机，不启动机械臂驱动或发送运动命令。
无有效标定或机械臂移动时，仍可预览及提供二维识别结果；
可抓取的三维目标需要有效深度、静止反馈、TF 与标定身份校验。
该节点独占相机，运行前关闭其他占用同一相机的程序。

单独启动默认红色检测预览，先设置上述 `ZKEEP_WS`：

```bash
source "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/etc/profile.d/conda.sh"
conda activate "${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/rebotarm"
cd "$ZKEEP_WS"
source install/setup.bash
python "$(ros2 pkg prefix zekeep_shadow)/lib/zekeep_shadow/web_vision" --ros-args \
  -p perception_root:="$ZKEEP_WS/src/zekeep_grasp"
```

使用 YOLO/GraspNet 时在该命令追加 `-p grasp_backend:=yolo-graspnet`。
按 `Ctrl+C` 关闭视觉节点；该入口不启动网页服务。

需要同时启动任务服务与视觉节点时，使用：

```bash
ros2 launch zekeep_bringup web_tasks.launch.py start_vision:=true \
  perception_root:="$ZKEEP_WS/src/zekeep_grasp"
```

`web_tasks.launch.py` 与 `web_grasp.launch.py` 默认显式使用 Miniforge 的视觉 Python，
避免已安装 ROS 节点的 shebang 选择 ROS venv；支持 `ZKEEP_MINIFORGE_DIR`、
`ZKEEP_VISION_PYTHON` 和优先级最高的启动参数 `python_executable`。运动授权默认关闭。
网页、驱动、视觉与 AI 的完整启动方式见[网页 README](../zekeep_bringup/web/README.md)。
