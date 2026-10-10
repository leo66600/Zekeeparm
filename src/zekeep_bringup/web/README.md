# Six-axis 网页仿真与控制系统

基于 Web 的六轴机械臂可视化与控制平台。集成 URDF 模型渲染、MoveIt 运动规划预览、示教回放以及 YOLO + GraspNet 视觉语言（VLM/LLM）控制。

---

## 1. 快速启动

每个终端先设置实际工作区路径：

```bash
export ZKEEP_WS="$HOME/Desktop/Zekeeparm"
```

视觉环境统一使用 Miniforge 的 `~/miniforge3/envs/rebotarm`。
自定义根目录时设置 `ZKEEP_MINIFORGE_DIR`；显式 `ZKEEP_VISION_PYTHON` 优先。
首次安装见[工作区安装说明](../../../README.md#一键安装)，仅有旧 Miniconda 环境时需先准备 Miniforge 环境。

### 推荐：统一启动（网页 + 驱动 + 视觉 + AI）
单命令拉起网页服务、rosbridge、MoveIt、视觉检测后端、任务调度及真机驱动：

```bash
# 1. 启动真机驱动、视觉与示教任务；AI 运动授权关闭
"$ZKEEP_WS/tools/start_web_sim.sh"

# 2. 额外授权 AI 运动，仍需网页人工核对并确认
"$ZKEEP_WS/tools/start_web_sim.sh" --ai-motion

# 3. 只读预览模式（不启动驱动，不授权运动）
"$ZKEEP_WS/tools/start_web_sim.sh" --preview
```

默认入口不自动使能或执行任务，但视觉/示教任务的后端运动授权已打开；
`--ai-motion` 额外打开 AI 运动授权。完全只读请使用 `--preview`。
统一启动器在启动任何节点前检查网页端口、rosbridge `9090` 和 AI 后端 `8082`；
端口被占用时停止启动，先按原流程关闭旧实例，避免使用另一实例的 AI 授权状态。

> **安全退出**：结束任务并放下物体后，按一次 `Ctrl+C` 等待驱动安全回零并退出，严禁强杀驱动进程。

---

### 分模块手动启动（进阶 / 调试）

* **仅 Web 基础节点（含 rosbridge 与 AI 服务，不含真机驱动）**：
  ```bash
  cd "$ZKEEP_WS"
  source install/setup.bash
  ros2 launch zekeep_bringup web.launch.py
  ```

* **独立纯模型预览（无需 npm install，不依赖 ROS）**：
  ```bash
  node "$ZKEEP_WS/src/zekeep_bringup/web/server.js"
  ```

* **访问入口**：
  * Web 控制前端：`http://127.0.0.1:3001`
  * rosbridge WebSocket：`ws://127.0.0.1:9090`
  * AI / MCP 服务端口：`http://127.0.0.1:8082`

---

## 2. 视觉与 AI 抓取操作流程

系统采用 **“生成计划 $\to$ 人工核对 $\to$ 确认执行”** 机制。视觉抓取在机械臂静止时选取单帧标定有效的 RGB-D 快照，规划后不依赖后续画面追踪。

1. **就绪确认**：使用 `--ai-motion` 启动，打开网页实机控制锁并使能机械臂。
2. **场景查询**：保持机械臂静止，在“LLM / MCP 文本控制”输入 `当前桌上有什么`，核对物体识别结果。
3. **指令与分步控制**：
   * **完整抓放**：输入 `抓取红色物块`，核对生成计划，勾选现场监督并确认执行。
   * **仅夹取**：输入 `仅夹取红块，不放置`（抬升后保持夹持，不自动归位）。
   * **持物回预备位**：输入 `夹取红块后回我的预备位，不放置`。
   * **单独放置**：输入 `放置已夹住的物体`（使用既有配置目标释放并回位）。

> **说明**：
> * 支持颜色：`red`、`blue`、`green`、`yellow`、`purple`。若存在多个同色目标将拒绝自动选择，需人工分离后重新检测。
> * 已配置红、蓝、绿专用放置区；其他未配置类别使用原普通区。修改位置见 [视觉包按颜色放置说明](../../zekeep_grasp/README.md#按颜色放置)。完整抓放和单独放置均使用抓取时锁定的类别。
> * 物块颜色由分割掩膜内的 HSV 像素复核；明确的颜色修正 YOLO 标签，无法确认的彩色物块不用于抓取。确认占比配置为 `yolo.block_color_min_fraction=0.60`。
> * 快照锁定后请勿移动物体；若桌面发生变动，请取消任务后重新触发。
> * 场景查询统计最近 2 秒内重复出现的二维识别结果；物体没有可靠深度或有效抓取几何时，也可列出，但不能据此执行抓取。抓取仍使用当前标定有效、机械臂静止的三维目标。
> * 看得到物块但没有识别框时，先检查遮挡、光照和物体在画面中的大小；当前 YOLO 置信度阈值为 `0.30`，不应把低置信度识别当作已确认目标。

---

## 3. 网页核心功能与控制

* **实时状态与预览**：
  * 主模型实时同步 `/zekeep/joint_states` 实际姿态。
  * 半透明模型显示目标预览及 IK 规划。
* **位姿（Pose）控制**：
  * 坐标系采用 ROS `base_link`（单位：米），前端自动映射至 Three.js：$(X_{\text{Three}}, Y_{\text{Three}}, Z_{\text{Three}}) = (X_{\text{ROS}}, Z_{\text{ROS}}, -Y_{\text{ROS}})$。
  * 支持设置 XYZ 与期望运动时长；点击“IK 运动”后经限位校验方下发执行，单纯编辑仅供预览。
  * “关闭预览 / 打开预览”按钮控制目标点、连线与半透明目标姿态。默认打开；关闭后停止预览 IK 请求，重新打开时显示当前输入，不发送运动。
* **双模轨迹示教**：
  * **网页示教**：本地航点编辑，格式为 `zekeep_ros_waypoints_v1`。
  * **实机示教**：拖动记录真实关节序列，格式为 `zekeep_teach`（支持“清除本次记录”重置数据）。
* **保护机制**：
  * “停止并保持”直通 `/zekeep/stop`；任务租约确保视觉、示教与手动控制互斥。

---

## 4. 关键技术参数

| 分类 | 项目 | 参数 / 范围 | 说明 |
| :--- | :--- | :--- | :--- |
| **关节限位** | J1 | $[-2.58, 2.58]\text{ rad}$ | 超出范围拒绝规划 |
| | J2 | $[0, 3.7]\text{ rad}$ | |
| | J3 | $[-0.01, 3.7]\text{ rad}$ | |
| | J4–J6 | $[-1.57, 1.57]\text{ rad}$ | |
| **末端夹爪** | 界面开口 | $0 \sim 65.17\text{ mm}$ | 单指平移行程 $0 \sim 0.0325862\text{ m}$ |
| | 电机行程 | $0 \sim 1.35\text{ rad}$ | 映射基准 1.45 rad = 70 mm；超限请求和候选拒绝 |
| **运动速度** | 常规关节速度 | $\le 0.30\text{ rad/s}$ | 实际时长按位移与三次插值自适应延长 |
| | 抓取搬运 / 放置 | $\le 0.30\text{ rad/s}$ | |
| | 抓取下降 / 抬升 | $\le 0.10\text{ rad/s}$ | 接触确认后才允许抬升 |
| **碰撞余量** | 夹爪连杆 | $3\text{ mm}$ | 下降路径必须全段无碰撞 |
| | 其余机械臂连杆 | $20\text{ mm}$ | |
| **视觉检测** | YOLO 输入分辨率 | $1280 \times 1280$ | 默认置信度阈值 $0.3$ |

---

## 5. 编译构建

如修改了核心包接口或配置，执行以下命令完成增量构建：

```bash
cd "$ZKEEP_WS"
source /opt/ros/humble/setup.bash
python3 -m colcon build --base-paths src \
  --packages-select zekeep_msgs zekeepcontroller zekeep_bringup zekeep_shadow zekeep_teach zekeep_moveit_config zekeep_llm
```
