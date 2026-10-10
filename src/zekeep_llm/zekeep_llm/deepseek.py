"""Small DeepSeek text client using only the Python standard library."""

from __future__ import annotations

import json
import os
import ssl
from urllib.parse import urlsplit
import urllib.error
import urllib.request

from .planner import Plan, TOOL_SCHEMAS, parse_plan


API_URL = "https://api.deepseek.com/chat/completions"
MODEL = "deepseek-flash"
MAX_RESPONSE_BYTES = 1024 * 1024

SYSTEM_PROMPT = """你是 Zekeep 机械臂的纯文本任务规划器。只输出 JSON：
{"summary":"中文计划或缺少信息的原因","steps":[{"tool":"工具名","arguments":{}}]}
所有写操作由人工确认、后端授权和互斥校验后执行，不能绕过确认。
单位：关节角和 RPY 为 rad，位置为 base_link 下 m，夹爪开口为 mm。
不得编造当前关节位置、物块检测结果、ROS 接口、其他工具或任意代码。
move_joints 支持六个关节值 positions_rad，relative=true 为当前位姿偏移；
oscillate=true 时按当前位姿的负偏移、正偏移往复 repeat 次，最后回起点。
“左右摆动N次”缺少幅度时采用 joint1 ±10度（0.1745329252rad），其他偏移为0，
relative=true、oscillate=true、repeat=N、duration_s=3，并在 summary 明确幅度和关节，供用户确认。
后端执行前重新读取当前关节并检查全部往复端点限位；超限拒绝整个运动。
move_to_pose 和 ik_check 需要明确 position_m 和 rpy_rad；缺少目标或方向时返回空计划询问，不能自行猜测。
询问“桌上有什么”等场景信息时使用 detect_blocks 查询当前 YOLO 检测；不要预先声称只返回红色或能够识别所有物体。
detect_blocks 在感知未运行时会自动启动现有相机/YOLO 后端并等待检测；无需让用户手动启动，不会使能或移动机械臂。
终端执行任务会按需补齐现有驱动和 MoveIt；启动驱动不使能，必须明确调用 enable_robot 并由用户确认。已有服务直接复用；diagnose_ros、停止、失能和清除记录不拉起新驱动。
pick_color 通过独立视觉抓取任务执行，支持 red/blue/green/yellow/purple；紫色使用 purple，不得替换成 red。没有对应颜色的检测或有多个同色目标时会拒绝。
“仅夹取”“抓住不放置”“只抓取”使用 pick_color 并明确 place_after=false：夹取后抬升并保持夹持，不放置、不回预备位。
“夹取并放置”“抓取并搬运”使用 pick_color、place_after=true；未指定拆分时沿用完整流程。
“夹取后回预备位，不放置”“夹起后回我的预备位”依次使用 pick_color、place_after=false，再 return_ready，无参数；保持夹持，不调用 place_object 或 open_gripper。
“夹取后移动到指定位置，不放置”先 pick_color、place_after=false，再用用户明确给出的 positions_rad 调用 move_joints，或完整 position_m、rpy_rad 调用 move_to_pose；缺少目标参数则返回空计划询问，不猜测坐标或姿态。
“放置已夹住的物体”使用 place_object，无参数：沿已有 fixed_joint 配置按夹取时的识别类别选择 red_place、blue_place、green_place，其他物品使用默认固定点，再按配置回预备位；未夹持时拒绝。
不得用 open_gripper、return_ready 或自编关节路径代替 place_object；仅夹取后不得自行追加放置或打开夹爪。
gravity_compensation_start 需要现场监督和动力学校验；record_start 只记录反馈，不启动重力补偿。
record_replay 等待每个动作成功。grasp 仅限力闭爪，不是完整视觉抓取。
stop/disable_robot 必须独占计划。最多32步，每步参数严格遵守以下工具 JSON Schema：
""" + json.dumps(TOOL_SCHEMAS, ensure_ascii=False)


class DeepSeekPlanner:
    def __init__(self, *, api_key: str | None = None, timeout_s: float = 30.0):
        key = api_key or os.environ.get("ZKEEP_LLM_API_KEY") or os.environ.get("ZKEEP_VLM_API_KEY")
        if not key or not key.strip():
            raise ValueError("缺少环境变量 ZKEEP_LLM_API_KEY")
        if not 1.0 <= float(timeout_s) <= 120.0:
            raise ValueError("API 超时必须在 1 到 120 秒之间")
        self._api_key = key.strip()
        self._timeout_s = float(timeout_s)
        proxy = os.environ.get("ZKEEP_LLM_PROXY_URL", "").strip()
        self._opener = None
        if proxy:
            parsed = urlsplit(proxy)
            if parsed.scheme not in ("http", "https") or not parsed.hostname or parsed.username or parsed.password or parsed.path not in ("", "/") or parsed.query or parsed.fragment:
                raise ValueError("ZKEEP_LLM_PROXY_URL 必须是无凭据的 HTTP/HTTPS 代理地址")
            # Explicit per-client proxy; do not modify process/global proxy settings.
            self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({"https": proxy}))

    def plan(self, task: str) -> Plan:
        task = task.strip()
        if not task or len(task) > 1000:
            raise ValueError("任务长度必须在 1 到 1000 字符之间")
        payload = {
            "model": MODEL,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": task},
            ],
            "response_format": {"type": "json_object"},
            "max_tokens": 2500,
        }
        request = urllib.request.Request(
            API_URL,
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {self._api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            open_request = self._opener.open if self._opener else urllib.request.urlopen
            with open_request(request, timeout=self._timeout_s) as response:
                body = response.read(MAX_RESPONSE_BYTES + 1)
            if len(body) > MAX_RESPONSE_BYTES:
                raise RuntimeError("DeepSeek API 返回内容过大")
            result = json.loads(body)
            raw = result["choices"][0]["message"]["content"]
            if not isinstance(raw, str) or not raw.strip():
                raise ValueError("模型返回空内容")
            return parse_plan(raw)
        except urllib.error.HTTPError as exc:
            raise RuntimeError(f"DeepSeek API HTTP {exc.code}") from exc
        except urllib.error.URLError as exc:
            if isinstance(exc.reason, ssl.SSLEOFError):
                raise RuntimeError("DeepSeek TLS 连接被出口链路截断；检查代理/TUN，或配置 ZKEEP_LLM_PROXY_URL 的可用出口。证书校验仍开启。") from exc
            raise RuntimeError(f"DeepSeek API 连接失败: {exc.reason}") from exc
        except (KeyError, IndexError, TypeError, json.JSONDecodeError) as exc:
            raise RuntimeError("DeepSeek API 返回格式无效") from exc
