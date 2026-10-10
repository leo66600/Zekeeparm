# Zekeep LLM / MCP 文本控制

使用 DeepSeek `deepseek-flash` 将终端自然语言文本任务转换为受限 ROS 工具计划。
模型输出不直接控制关节、坐标或电机；本地白名单校验和人工确认通过后才调用现有接口。

ROS 包名和 Python 模块名均为 `zekeep_llm`，终端入口为 `ros2 run zekeep_llm terminal`。
功能为纯文本 LLM，不接受图片，`--image` 已移除。优先使用 `ZKEEP_LLM_API_KEY`；旧 `ZKEEP_VLM_API_KEY` 仅作为密钥变量迁移兼容，不启用视觉功能。

## 支持工具与参数

下表为 LLM、MCP、ROS 适配器共用的后端契约。所有写操作仍需人工确认与后端授权。

| 工具 | 参数 / 行为 |
| --- | --- |
| `get_robot_status` | 无参数，真实关节/电机新鲜度、使能与控制状态 |
| `diagnose_ros` | 无参数，节点、服务、话题与反馈年龄 |
| `gravity_compensation_status` | 无参数，驱动重力补偿状态 |
| `ik_check` | `position_m: [x,y,z]`、`rpy_rad: [r,p,y]`，只求解，不运动 |
| `detect_blocks` | 无参数，只读近期二维识别与最新三维目标；抓取目标要求标定有效且机械臂静止 |
| `enable_robot` / `disable_robot` | 无参数，显式使能/失能；模型不会自行补上使能动作 |
| `gravity_compensation_start` / `gravity_compensation_stop` | 无参数，启动/结束补偿；启动前需现场监督，并具备动力学验证或明确的监督试验授权 |
| `record_start` / `record_stop` / `record_clear` | 无参数，记录真实反馈到本后端内存；录制不会开启补偿，不静默覆盖已有记录，清除前需停止 |
| `safe_home` | 无参数，内置安全回零 |
| `set_gripper_opening_mm` | `opening_mm: 0–(70×1.35/1.45)`，约 0–65.17 mm，执行时读取实际夹爪标定，不能超过实际最大开口 |
| `move_to_pose` | `position_m`、`rpy_rad`，可选 `duration_s: 0.2–60`，精确 IK 后低速关节轨迹执行 |
| `move_joints` | `positions_rad` 六个 rad；可选 `relative`、`oscillate`、`repeat: 1–20`、`duration_s: 0.2–60` |
| `pick_color` | `color: red/blue/green/yellow/purple`；可选 `place_after: false` 仅夹取、抬升并保持夹持。省略或为 `true` 沿用配置的完整流程。未检测到该颜色或同色目标不唯一时拒绝 |
| `place_object` | 无参数，将本视觉后端已夹取并成功抬升的物体放到配置的 `fixed_joint` 点，释放后按配置回 Ready。无已夹持物体或放置配置未启用时拒绝 |
| `record_replay` | 无参数，结束记录后低速回放；等待各关节动作与夹爪结果 |

兼容保留 `get_status`、`stop`、`return_ready`、`open_gripper`、`grasp`。`grasp` 仅限力闭爪；完整视觉抓取使用 `pick_color`。单个计划最多 32 步，stop/disable 必须独占计划。

拆分操作：输入“仅夹取红色物块，不放置”，确认计划中 `pick_color` 的 `place_after=false`，执行后抬升并保持夹持；随后输入“放置已夹住的物体”，单独确认 `place_object`。
“夹取红块并放置”仍使用 `place_after=true`。仅夹取不自动返回 Ready，也不打开夹爪；其附着物碰撞模型保留到释放。
输入“夹取红块后回我的预备位，不放置”，计划应依次为
`pick_color`（`color=red, place_after=false`）、`return_ready`（无参数）。
夹取成功并抬升后才回已配置预备位，继续保持夹持；夹取失败时不执行后续步骤。
其他持物目标可明确提供六个关节角（`positions_rad`，rad），或完整的位置和姿态
（`position_m`，base_link 下 m；`rpy_rad`，rad）；仍需限位与路径碰撞检查，不推测缺失坐标。
持物运动和视觉规划的关节状态请求保留 MoveIt 场景中的附着物，避免忽略物体碰撞。
抓取、放置各自等待 Action 完成，沿用监督、授权、互斥、心跳、取消与碰撞校验。持物状态在任务后端内存中，重启不会自动恢复；重新抓取前需处理当前夹持物体。

坐标使用 `base_link`，RPY 与关节为 rad。姿态目标需完整位置和方向，不允许模型猜测缺失方向。位置受限在 0.7m 半径、Z≥0 范围，实际 IK/碰撞不可达会拒绝。

“当前桌上有什么”使用 `detect_blocks`，网页显示中文物体列表、数量与置信度。
结果只覆盖当前 YOLO 配置类别的近期二维识别，不代表桌上全部物体；
有二维识别不等于存在可抓取的有效 RGB-D 目标。执行返回的 `execution_events` 只包含本次工具结果，原 `events` 仍保留历史记录；MCP 调试保留原始 JSON。

终端的桌面查询直接输出中文分类与数量，不显示计划 JSON、图像框、坐标或检测 ID。展示时将 IoU > 0.8 的近乎重合图像框合并计数，优先采用观测次数更多、置信度更高的类别；同一位置类别跳变或置信度低于 50% 时标注“识别待确认”，不直接丢弃低置信度物体。无有效结果时明确说明无法判断，不宣称桌面为空。此清洗仅用于终端展示，原始 ROS/MCP 检测结果和抓取安全校验保持不变；运动计划和 `--dry-run` 仍显示完整计划 JSON，供人工检查。

终端和网页调用 `detect_blocks` 时，若没有新鲜检测且检测话题没有发布者，会自动启动现有 `zekeep_shadow/web_vision` 相机/YOLO 后端，首次加载最多等待 45 秒，再收集 2 秒检测。已有后端直接复用；助手退出只结束自己启动的感知进程。此启动入口只打开相机，不启动驱动、使能或抓取，也不跳过标定、静止状态和检测新鲜度校验。
自动启动需已构建并 source `zekeep_shadow` 包。默认视觉 Python 使用 `~/miniforge3/envs/rebotarm/bin/python`；自定义 Miniforge 根目录通过 `ZKEEP_MINIFORGE_DIR` 指定，显式 `ZKEEP_VISION_PYTHON` 优先；工作区自动定位，必要时设置 `ZKEEP_WS`。使用现有 `src/zekeep_grasp/config/default.yaml` 和 `yolo-graspnet` 配置，不修改模型或相机标定。相机被其他程序占用、模型依赖缺失或标定无效时明确报错。

文本查询汇总最近 2 秒内至少两帧检测到的物体，以同类别框的 IoU > 0.5 区分目标；窗口过后自动移除。`observations` 只提供名称、图像框、最近置信度及观测次数，不提供历史抓取坐标；原 `detections` 仍是最新帧的有效目标。抓取只使用最新目标，近期检测到多个同色物体也会拒绝任意选择。此汇总减少单帧漏检导致的计数跳动，不改变 YOLO 权重或阈值，也不保证每个物体都被识别。

“左右摆动10次”默认解释为当前姿态附近 joint1 ±10°，其余关节不变；计划必须展示该解释，人工确认后才执行。执行前读取最新反馈，检查全部往复端点限位，并用 MoveIt `/check_state_validity` 对每段按最大 0.03rad 间隔采样检查。MoveIt 缺失或任一点碰撞时，整个任务不发送。碰撞检查是离散采样，不是连续碰撞保证。峰值关节速度限制 0.30rad/s，时间不足自动延长；单工具动作/回放预算最多 1800秒。

重力补偿使用持续 `ai` 控制器租约；开启后保持任务所有权，直到明确停止/失能，或网页心跳丢失后停止保持。普通运动和其他任务不能抢占。enable/disable、内存记录不抢占运动租约；完整颜色抓取由视觉任务后端持有自己的抓取租约，不能与补偿同时执行。

记录最多 10000 个真实样本，不写文件；后端重启会丢失，因此正式持久化示教仍使用实机示教面板。回放会按照低速和动作结果重新定时，超出时间预算拒绝。录制没有夹爪反馈时只回放关节；没有反馈不能伪造开口。

## 构建

```bash
cd "$ZKEEP_WS"
source tools/activate_ros.sh
colcon build --packages-select zekeep_llm
source install/setup.bash
```

从旧包名迁移时，同时重新构建网页启动包，并在新终端加载环境：

```bash
cd "$ZKEEP_WS"
source /opt/ros/humble/setup.bash
colcon build --packages-select zekeep_llm zekeep_bringup
source install/setup.bash
```

原终端命令改用 `ros2 run zekeep_llm terminal`，Python 导入改用 `zekeep_llm`。
已经运行的终端助手或网页 AI 服务需要重新启动，才能使用新包。

## 配置与运行

API Key 只放环境变量，不要写入配置文件或提交到 Git：

```bash
export ZKEEP_LLM_API_KEY="你的 DeepSeek API Key"
```

交互模式：

```bash
ros2 run zekeep_llm terminal
```

终端执行任务时会按需补齐真机驱动和 MoveIt，无需另开终端手动启动；已有服务直接复用。驱动沿用当前工作区硬件配置和 `ZKEEP_ROS_VENV`，始终使用 `auto_enable=false`。使能仍需明确输入任务并确认 `yes`。MoveIt 使用现有 `hardware.launch.py`，开启固定工作台碰撞场景。服务最多等待 30 秒，再检查新鲜关节/电机反馈；启动失败不会发送后续动作。

`diagnose_ros`、停止、失能、停止补偿、停止/清除记录不会拉起新驱动。只查询状态等反馈操作不依赖 MoveIt。需要使能、关节/姿态运动、回零/预备位、回放或视觉查询时才补齐 MoveIt。`--dry-run` 不启动这些 ROS 后端。

自动启动的驱动和 MoveIt 在退出助手后继续运行：驱动退出可能触发安全回零，不能因关闭聊天而未经确认地回零或切断保持力矩。运行日志保存在 `~/.local/share/zekeep/llm_driver.log`、`llm_moveit.log`。进程锁防止多个助手重复启动；发现已有控制器/MoveIt 进程但对应服务不可达时，明确拒绝重复启动，需核对 ROS domain 和命名空间。原相机/感知进程仍在其所属助手退出时清理。

只查看计划，不调用 ROS：

```bash
ros2 run zekeep_llm terminal --dry-run "打开夹爪，然后回准备位"
```

终端必须输入精确的 `yes` 才执行写操作；只有明确的 `enable_robot` 计划才调用使能接口。

## API 参考

DeepSeek API 用法依据官方文档：

- https://api-docs.deepseek.com/guides/json_mode/

## 网页文本控制

```bash
cd "$ZKEEP_WS"
source install/setup.bash
# 模型密钥仅配置在服务进程环境中，切勿写到网页或提交到仓库。
ros2 launch zekeep_bringup web_ai.launch.py
```

在后端进程环境设置 `ZKEEP_LLM_API_KEY` 后重启该启动项，网页“LLM / MCP 文本控制”才能生成自然语言计划。未配置密钥时，服务仍可提供健康状态、工具列表；生成计划明确失败。网页代码不读取或保存密钥。

`motion_authorized` 默认 `false`。需要实机执行时，由现场操作人员显式开启后端授权，确认监督和运动条件，再开启网页实机控制锁、勾选监督并确认所示计划。此启动项默认不启动控制器、不自动使能机器人。

网页默认不自动启动驱动或 MoveIt；显式设置 `motion_authorized=true` 后，确认执行时采用与终端相同的按需启动。依赖会在申请控制器任务租约前就绪。启动仍不自动使能，也不改变视觉抓取任务后端的独立运动授权。

聊天接口只返回严格解析的白名单计划，不执行工具。计划保存在后端，确认使用随机一次性 `plan_id`，有效期 120 秒；前端不能替换确认后的步骤。确认执行前后再次检查授权、工具和控制器任务租约。工具逐步等待结果，失败/取消/心跳丢失不继续下一步。后端持有 `ai` 租约，使用已有 `/zekeep/web_task/*` 控制器接口；租约续期失败由控制器看门狗停止保持并保留所有权。

白名单、参数和单位复用本页工具契约。网页不接受任意 ROS 路径或 Python 命令。旧的自动执行 text-agent `/chat` 不再作为网页代理目标；网页改为请求 `/plan`，并拒绝健康协议不是 `plan-review-v1` 的后端。

“断开助手”只关闭助手连接；它不宣称机器人停止。执行中的任务因失去网页心跳请求取消保持。“取消 AI 任务”等待任务结束及停止确认，常规机器人停止按钮也会取消 AI 任务。停止/租约释放未确认时，后端锁定后续运动，需要检查控制器后恢复。

### HTTP 与 MCP

服务只监听 `127.0.0.1:8082`，Node 网页使用固定路径代理。拒绝跨源 JSON 写请求，不开放任意后端 URL。这里是本机操作边界，不是多用户认证；不要将这些服务转发到公网。

- `POST /plan`（`/chat` 同义）：`{"text":"查询状态"}`，只生成计划。
- `POST /execute`：`{"plan_id":"...","confirmed":true}`，复检后执行。
- `POST /cancel`：`{}`，等待结束/保持确认。
- `GET /health`、`GET /status`：状态、授权和有限事件日志，不含密钥。
- `POST /mcp`：无会话 JSON 响应的 Streamable HTTP；协议 `2025-03-26`，支持 initialize、ping、notifications/initialized、tools/list、tools/call。GET 返回 405，不提供 SSE。

MCP 提供 `plan` 和本页全部工具。运动工具必须携带人工确认后由后端签发的一次性 `approval_token`。令牌绑定具体工具及完整参数、只用一次且不返回给模型或 Dashboard；确认执行器通过相同 MCP 工具调度路径完成计划。缺少、伪造或重放令牌均拒绝运动。网页“工具调试”提供工具列表、JSON 参数表单和调用结果。未实现任意 Webhook 注册。

MCP 接口依据 [Streamable HTTP 规范](https://modelcontextprotocol.io/specification/2025-03-26/basic/transports)与[工具规范](https://modelcontextprotocol.io/specification/2025-03-26/server/tools)。

### TLS EOF / 全局 TUN 出口

`SSL: UNEXPECTED_EOF_WHILE_READING` 是 TLS 连接被截断，不能据此判断密钥无效。仅移除 HTTP_PROXY 不会绕过全局 TUN。

LLM 支持独立 `ZKEEP_LLM_PROXY_URL`，不修改系统代理，不关闭证书校验。已安装 Mihomo 的机器可按下一节配置 `~/.local/share/zekeep/llm_network.json`，
然后启动 `ros2 run zekeep_llm terminal` 或 `zekeep_bringup web_ai.launch.py`；
后端会为自身启动限定 DeepSeek 的本机出口。

端口设为 17897 时，该出口仅监听 127.0.0.1:17897，只允许 api.deepseek.com，其余域名拒绝。不会改变 Clash 全局节点、TUN 或订阅配置。已有 8082/17897 服务时先结束对应旧实例，避免端口冲突。若使用其他已验证代理：

```bash
ros2 launch zekeep_bringup web_ai.launch.py llm_proxy_url:=http://127.0.0.1:17897
```

保持密钥在后端环境中。直连出口适用于上述网络条件，不把特定网卡写成所有机器的默认值。

### 网页启动时自动启动后端

`ros2 launch zekeep_bringup web.launch.py` 默认包含文本后端，关闭启动项时后端一起结束；不再依赖临时单独启动的进程。已有独立文本服务时设置 `start_ai:=false`。

可选本机配置 `~/.local/share/zekeep/llm_network.json`：

```json
{"direct_interface":"enp2s0","proxy_port":17897}
```

该文件只保存物理网卡与本机端口，不保存密钥。存在配置时，网页后端和终端都会为自身启动 DeepSeek 专用出口，并在退出时结束自己启动的出口；已存在的出口不由它关闭。显式配置不同的 `ZKEEP_LLM_PROXY_URL` 优先。没有此文件的机器继续使用原来的网络配置。网卡需填写本机实际接口，Mihomo 二进制需存在于 `/usr/bin/verge-mihomo`。

终端遇到 `Connection refused` 时，检查代理端口是否仍有服务监听；仅设置代理地址不会启动出口。配置上述文件后，重新构建 `zekeep_llm`、source 工作区并重启终端，即可自动启动专用出口。可先用 `ros2 run zekeep_llm terminal --dry-run "查询状态"` 验证计划生成，不调用机械臂。
