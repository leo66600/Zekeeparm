"""通过单一后台线程串行访问舵机总线。"""

from __future__ import annotations

import queue
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable

from .bus import TARGET_PWM_BY_ID, ZhonglingServoBus


DEFAULT_TIMEOUT_S = 2.0
DEFAULT_SCAN_TIMEOUT_S = 15.0
DEFAULT_WRITE_TIMEOUT_S = 30.0
READING_INTERVAL_S = 0.1
DEFAULT_TOLERANCE = 10
STABILITY_SAMPLES = 3
STABILITY_SPAN = 5
SAMPLE_INTERVAL_S = 0.03
CALIBRATE_RELEASE_WINDOW_S = 0.35
CALIBRATE_RELEASE_INTERVAL_S = 0.01
PERSISTENT_WRITE_TIMEOUT_S = 1.0
ID_SAVE_WAIT_S = 0.5
ID_SCAN_START = 0
ID_SCAN_END = 10
CALIBRATION_TARGET_PWM = 1500
CALIBRATION_PLAN = {
    servo_id: {
        "command": "PSCK" if target == CALIBRATION_TARGET_PWM else None,
        "target": target,
        "manual": target != CALIBRATION_TARGET_PWM,
    }
    for servo_id, target in TARGET_PWM_BY_ID.items()
}


def _error(code: str, message: str, **extra: Any) -> dict[str, Any]:
    return {"ok": False, "error": {"code": code, "message": message}, **extra}


class _SimulatedBus:
    """确定性假总线，用于无硬件演练。"""

    PROFILES = ("assembled", "unstable", "new", "new-no-pcsm", "numbered", "multi", "empty")

    def __init__(self, port: str, baudrate: int, timeout_s: float, response_wait_s: float,
                 use_crlf: bool, profile: str = "assembled") -> None:
        del timeout_s, response_wait_s, use_crlf
        if profile not in self.PROFILES:
            raise ValueError(f"未知模拟配置：{profile}")
        self.port = port
        self.baudrate = baudrate
        self.profile = profile
        if profile in {"assembled", "unstable"}:
            self.pwms = {sid: target - 40 for sid, target in TARGET_PWM_BY_ID.items()}
        elif profile in {"new", "new-no-pcsm"}:
            self.pwms = {0: 1460}
        elif profile == "numbered":
            self.pwms = {2: 2460}
        elif profile == "multi":
            self.pwms = {0: 1460, 3: 1460}
        else:
            self.pwms = {}
        self.silent: set[int] = set()
        self._tick = 0

    def close(self) -> None:
        pass

    def command(self, command: str) -> str:
        match = re.fullmatch(r"#(\d{3})P([A-Z]{2,4})(\d{0,4})!", command)
        if not match:
            return ""
        servo_id, verb, argument = int(match[1]), match[2], match[3]
        if servo_id not in self.pwms or servo_id in self.silent:
            return ""
        if verb == "VER":
            return f"#{servo_id:03d}PV模拟!"
        if verb == "RAD":
            return f"#{servo_id:03d}P{self._pwm_of(servo_id):04d}!"
        if verb in {"ULK", "ULR"}:
            return "#OK!"
        if verb == "ID" and argument:
            target = int(argument)
            self.pwms[target] = self.pwms.pop(servo_id)
            return "#OK!"
        if verb == "SCK":
            self.pwms[servo_id] = 1500
            return "#OK!"
        if verb == "CSM":
            if self.profile == "new-no-pcsm":
                return ""
            self.silent.add(servo_id)
            return "#OK!"
        if verb == "CSR":
            return "#OK!"
        return ""

    def _pwm_of(self, servo_id: int) -> int:
        pwm = self.pwms[servo_id]
        if self.profile == "unstable":
            self._tick += 1
            return pwm + (-9, 0, 9)[self._tick % 3]
        return pwm

    def read_pwm(self, servo_id: int, retries: int) -> int | None:
        for _ in range(retries):
            if servo_id in self.pwms and servo_id not in self.silent:
                return self._pwm_of(servo_id)
        return None

    def release_torque(self, servo_id: int) -> str:
        return self.command(f"#{servo_id:03d}PULK!")


@dataclass
class _Request:
    operation: str
    args: tuple[Any, ...]
    reply: queue.Queue[dict[str, Any]] = field(default_factory=lambda: queue.Queue(maxsize=1))


class BusWorker:
    """Own the bus and serialize every command on one daemon thread."""

    def __init__(self, simulate: bool = False, profile: str = "assembled") -> None:
        if simulate and profile not in _SimulatedBus.PROFILES:
            raise ValueError(f"未知模拟配置：{profile}")
        self._simulate = simulate
        self._profile = profile
        self._requests: queue.Queue[_Request | None] = queue.Queue()
        self._bus: ZhonglingServoBus | _SimulatedBus | None = None
        self._port: str | None = None
        self._baudrate: int | None = None
        self._found_ids: list[int] = []
        self._unknown_ids: list[int] = []
        self._active_ids: set[int] = set()
        self._readings: dict[str, dict[str, Any]] = {}
        self._pending_power_cycle: dict[int, str] = {}
        self._tolerance = DEFAULT_TOLERANCE
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name="舵机总线线程", daemon=True)
        self._thread.start()

    @staticmethod
    def _valid_id(value: Any, low: int = 0, high: int = 254) -> bool:
        return isinstance(value, int) and not isinstance(value, bool) and low <= value <= high

    def _submit(self, operation: str, *args: Any, timeout_s: float = DEFAULT_TIMEOUT_S) -> dict[str, Any]:
        if timeout_s <= 0:
            return _error("invalid_request", "超时时间必须大于零")
        if not self._thread.is_alive():
            return _error("worker_stopped", "舵机总线工作线程已停止")
        request = _Request(operation, args)
        self._requests.put(request)
        try:
            return request.reply.get(timeout=timeout_s)
        except queue.Empty:
            return _error("timeout", "总线操作超时")

    def _run(self) -> None:
        while True:
            try:
                request = self._requests.get(timeout=READING_INTERVAL_S)
            except queue.Empty:
                self._poll_readings()
                continue
            if request is None:
                self._close()
                return
            try:
                result = getattr(self, f"_do_{request.operation}")(*request.args)
            except Exception as exc:  # Serial errors must reach the UI as JSON.
                self._close()
                result = _error("operation_failed", f"总线操作失败：{exc}")
            request.reply.put(result)
            self._poll_readings()

    def _status_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "connected": self._bus is not None,
                "port": self._port,
                "baudrate": self._baudrate,
                "found_ids": list(self._found_ids),
                "unknown_ids": list(self._unknown_ids),
                "reading_ids": sorted(self._active_ids),
                "pending_power_cycle": dict(self._pending_power_cycle),
            }

    def _reject_if_reading(self) -> dict[str, Any] | None:
        with self._lock:
            active = sorted(self._active_ids)
        if not active:
            return None
        listed = "、".join(f"{sid:03d}" for sid in active)
        return _error("reading_active", f"仍有舵机在读数中（{listed}），持久写入前请先停止全部读数")

    def _close(self) -> None:
        bus = self._bus
        with self._lock:
            self._bus = None
            self._port = None
            self._baudrate = None
            self._found_ids.clear()
            self._unknown_ids.clear()
            self._active_ids.clear()
            self._readings.clear()
            self._pending_power_cycle.clear()
        if bus is not None:
            try:
                bus.close()
            except Exception:
                pass

    def _do_open_port(self, port: str, baudrate: int) -> dict[str, Any]:
        if self._bus is not None:
            return _error("already_connected", "串口已经连接，请先断开")
        bus_type: Callable[..., Any] = _SimulatedBus if self._simulate else ZhonglingServoBus
        kwargs = {"port": port, "baudrate": baudrate, "timeout_s": 0.1,
                  "response_wait_s": 0.03, "use_crlf": True}
        if self._simulate:
            kwargs["profile"] = self._profile
        bus = bus_type(**kwargs)
        with self._lock:
            self._bus = bus
            self._port, self._baudrate = port, baudrate
        return {"ok": True, "status": self._status_snapshot()}

    def _do_close_port(self) -> dict[str, Any]:
        self._close()
        return {"ok": True, "status": self._status_snapshot()}

    def _scan_range(self, start: int, end: int) -> tuple[list[int], list[int]]:
        assert self._bus is not None
        found, unknown = [], []
        for servo_id in range(start, end + 1):
            version = self._bus.command(f"#{servo_id:03d}PVER!")
            pwm = self._bus.read_pwm(servo_id, retries=1)
            (found if version or pwm is not None else unknown).append(servo_id)
        return found, unknown

    def _do_scan(self, start: int, end: int) -> dict[str, Any]:
        if self._bus is None:
            return _error("not_connected", "尚未连接串口")
        found, unknown = self._scan_range(start, end)
        with self._lock:
            self._found_ids, self._unknown_ids = found, unknown
            pending = dict(self._pending_power_cycle)
        return {"ok": True, "found_ids": found, "unknown_ids": unknown,
                "pending_power_cycle": pending}

    def _do_read_pwm(self, servo_id: int) -> dict[str, Any]:
        if self._bus is None:
            return _error("not_connected", "尚未连接串口")
        pwm = self._bus.read_pwm(servo_id, retries=1)
        if pwm is None:
            return _error("no_response", f"舵机 ID {servo_id:03d} 没有有效回包")
        return {"ok": True, "id": servo_id, "pwm": pwm}

    def _do_start_reading(self, servo_id: int, release_torque: bool) -> dict[str, Any]:
        if self._bus is None:
            return _error("not_connected", "尚未连接串口")
        released = self._bus.release_torque(servo_id) if release_torque else None
        with self._lock:
            self._active_ids.add(servo_id)
        return {"ok": True, "id": servo_id, "released": released}

    def _do_stop_reading(self, servo_id: int) -> dict[str, Any]:
        with self._lock:
            self._active_ids.discard(servo_id)
            self._readings.pop(str(servo_id), None)
        return {"ok": True, "id": servo_id}

    def _poll_readings(self) -> None:
        with self._lock:
            servo_ids = sorted(self._active_ids)
        if not servo_ids or self._bus is None:
            return
        now = time.time()
        updates = {}
        for servo_id in servo_ids:
            pwm = self._bus.read_pwm(servo_id, retries=1)
            updates[str(servo_id)] = {"id": servo_id, "pwm": pwm, "ok": pwm is not None,
                                      "message": "" if pwm is not None else "无响应", "updated_at": now}
        with self._lock:
            self._readings.update(updates)

    def _command_wait(self, message: str, followup: str = "") -> str:
        assert self._bus is not None
        if self._simulate:
            response = self._bus.command(message)
            if followup:
                self._bus.command(followup)
            return response
        serial_port = self._bus.serial
        serial_port.reset_input_buffer()
        serial_port.write(f"{message}{self._bus.terminator}".encode("ascii"))
        serial_port.flush()
        if followup:
            # A single PULK can be ignored during PSCK's save. Repeat only release,
            # never calibration; verified on unloaded 007 / V2.1.16STG.
            deadline = time.monotonic() + CALIBRATE_RELEASE_WINDOW_S
            while time.monotonic() < deadline:
                serial_port.write(f"{followup}{self._bus.terminator}".encode("ascii"))
                serial_port.flush()
                time.sleep(CALIBRATE_RELEASE_INTERVAL_S)
            return serial_port.read_all().decode("ascii", errors="ignore").strip()
        deadline, chunks = time.monotonic() + PERSISTENT_WRITE_TIMEOUT_S, []
        while time.monotonic() < deadline:
            waiting = serial_port.in_waiting
            if waiting:
                chunks.append(serial_port.read(waiting).decode("ascii", errors="ignore"))
                if "!" in chunks[-1]:
                    break
            time.sleep(0.02)
        return "".join(chunks).strip()

    def _do_set_id(self, source_id: int, target_id: int, confirm_text: str,
                   auto_release_on_boot: bool) -> dict[str, Any]:
        if self._bus is None:
            return _error("not_connected", "尚未连接串口")
        if busy := self._reject_if_reading():
            return busy
        found_before, _ = self._scan_range(ID_SCAN_START, ID_SCAN_END)
        if not found_before:
            return _error("no_response", "未检测到舵机，请检查接线", scan_before=found_before)
        if len(found_before) > 1:
            listed = "、".join(f"{sid:03d}" for sid in found_before)
            return _error("bus_not_exclusive", f"总线上检测到多个舵机（{listed}），ID 标定只能单独连接一个舵机",
                          scan_before=found_before)
        if found_before[0] != source_id:
            return _error("wrong_servo", f"总线上实际检测到的是 {found_before[0]:03d}，不是源 ID {source_id:03d}",
                          scan_before=found_before)
        expected = f"SET ID {target_id:03d}"
        if (confirm_text or "").strip() != expected:
            return _error("confirm_mismatch", f"确认文本不正确，需要输入：{expected}", scan_before=found_before)
        id_message = f"#{source_id:03d}PID{target_id:03d}!"
        boot_message = f"#{target_id:03d}PCSM!"
        id_response = self._command_wait(id_message)
        time.sleep(ID_SAVE_WAIT_S)
        found_after, unknown_after = self._scan_range(ID_SCAN_START, ID_SCAN_END)
        with self._lock:
            self._found_ids, self._unknown_ids = found_after, unknown_after
        id_ok = target_id in found_after and source_id not in found_after
        id_result = {"ok": id_ok, "command": id_message, "response": id_response,
                     "scan_before": found_before, "scan_after": found_after}
        if not id_ok:
            return {"ok": False, "error": {"code": "id_write_failed",
                    "message": f"改号未通过验证：改号后扫描到 {found_after}"},
                    "id_write": id_result, "boot_release": None}
        if not auto_release_on_boot:
            return {"ok": True, "id_write": id_result, "boot_release": None}
        boot_response = self._command_wait(boot_message)
        boot_ok = "#OK!" in boot_response
        boot_result = {"ok": boot_ok, "command": boot_message, "response": boot_response,
                       "verified": False, "note": "命令已接受，需断电重启后人工确认；手册没有读回开机释力状态的命令",
                       "goes_silent": boot_ok}
        if boot_ok:
            with self._lock:
                self._pending_power_cycle[target_id] = "PCSM"
            return {"ok": True, "id_write": id_result, "boot_release": boot_result}
        return {"ok": False, "partial": True,
                "error": {"code": "boot_release_failed",
                          "message": f"ID 已成功改为 {target_id:03d}，但开机释力未确认；请勿重发改号命令"},
                "id_write": id_result, "boot_release": boot_result}

    def _do_calibrate_zero(self, servo_id: int, confirm_text: str,
                           manual_release_confirmed: bool = False) -> dict[str, Any]:
        if self._bus is None:
            return _error("not_connected", "尚未连接串口")
        if busy := self._reject_if_reading():
            return busy
        plan = CALIBRATION_PLAN[servo_id]
        target = plan["target"]
        if plan["manual"]:
            return _error("manual_only", f"舵机 {servo_id:03d} 的零位目标是 {target}，PSCK 只能映射成 1500；请手动对位")
        expected = f"CALIBRATE {servo_id:03d} TO {target:04d}"
        if (confirm_text or "").strip() != expected:
            return _error("confirm_mismatch", f"确认文本不正确，需要输入：{expected}")
        before = self._bus.read_pwm(servo_id, retries=2)
        if before is None:
            return _error("no_response", f"舵机 ID {servo_id:03d} 没有有效回包，无法标定")
        release_message = f"#{servo_id:03d}PULK!"
        released = self._command_wait(release_message)
        if "#OK!" not in released and not (manual_release_confirmed and not released):
            return _error("torque_release_failed",
                          f"标定前释力未收到确认，未发送 PSCK；{release_message} 回包：{released!r}"
                          "（空字符串表示未收到回包）。无回包不能判定释力失败，也不能证明已释力",
                          command=release_message, response=released)
        samples = []
        for index in range(STABILITY_SAMPLES):
            if index:
                time.sleep(SAMPLE_INTERVAL_S)
            sample = self._bus.read_pwm(servo_id, retries=2)
            if sample is None:
                return _error("no_response", f"舵机 ID {servo_id:03d} 采样时无回包")
            samples.append(sample)
        span = max(samples) - min(samples)
        if span > STABILITY_SPAN:
            return _error("joint_not_stable", f"关节没有保持稳定：3 次采样 {samples}，跨度 {span} 超过 5", samples=samples)
        message = f"#{servo_id:03d}{plan['command']}!"
        response = self._command_wait(message, followup=release_message)
        # Firmware may ignore the queued PULK while saving; confirm a fresh release.
        re_release = self._command_wait(release_message)
        release_acknowledged = "#OK!" in re_release
        if not release_acknowledged and not (manual_release_confirmed and not re_release):
            return _error("torque_release_failed",
                          f"PSCK 已发送，但保存后释力未确认；{release_message} 回包：{re_release!r}；"
                          "请扶稳并检查舵机，勿直接重试标定",
                          id=servo_id, command=message, response=response, re_released=re_release,
                          torque_restored=None)
        after = self._bus.read_pwm(servo_id, retries=2)
        with self._lock:
            tolerance = self._tolerance
        verified = after is not None and abs(after - target) <= tolerance
        result = {"id": servo_id, "command": message, "response": response, "target": target,
                  "before": samples[-1], "samples": samples, "after": after, "tolerance": tolerance,
                  "verified": verified, "torque_restored": False if release_acknowledged else None,
                  "re_released": re_release, "torque_release_acknowledged": release_acknowledged,
                  "manual_release_confirmed": manual_release_confirmed}
        if verified:
            return {"ok": True, **result}
        return {"ok": False, "error": {"code": "pwm_out_of_tolerance",
                "message": f"回读 {after if after is not None else '无响应'} 与目标 {target} 相差超过 ±{tolerance}，标定未通过"}, **result}

    def _do_startup_torque(self, servo_id: int, mode: str, confirm_text: str) -> dict[str, Any]:
        if self._bus is None:
            return _error("not_connected", "尚未连接串口")
        if busy := self._reject_if_reading():
            return busy
        verb = "PCSM" if mode == "release" else "PCSR"
        expected = f"{verb} {servo_id:03d}"
        if (confirm_text or "").strip() != expected:
            return _error("confirm_mismatch", f"确认文本不正确，需要输入：{expected}")
        if self._bus.read_pwm(servo_id, retries=2) is None:
            return _error("no_response", f"舵机 ID {servo_id:03d} 不在线，拒绝写入")
        message = f"#{servo_id:03d}{verb}!"
        response = self._command_wait(message)
        accepted = "#OK!" in response
        if accepted:
            with self._lock:
                if mode == "release":
                    self._pending_power_cycle[servo_id] = verb
                else:
                    self._pending_power_cycle.pop(servo_id, None)
        payload = {"ok": accepted, "id": servo_id, "mode": mode, "command": message,
                   "response": response, "verified": False}
        return payload if accepted else {**payload, "error": {"code": "no_response", "message": "写入没有有效回包"}}

    def open_port(self, port: Any, baudrate: Any, timeout_s: float = DEFAULT_TIMEOUT_S) -> dict[str, Any]:
        if not isinstance(port, str) or not port.strip():
            return _error("invalid_request", "串口路径不能为空")
        if not isinstance(baudrate, int) or isinstance(baudrate, bool) or baudrate <= 0:
            return _error("invalid_request", "波特率必须是正整数")
        return self._submit("open_port", port.strip(), baudrate, timeout_s=timeout_s)

    def close_port(self, timeout_s: float = DEFAULT_TIMEOUT_S) -> dict[str, Any]:
        return self._submit("close_port", timeout_s=timeout_s)

    def scan(self, start: Any, end: Any, timeout_s: float = DEFAULT_SCAN_TIMEOUT_S) -> dict[str, Any]:
        if not self._valid_id(start) or not self._valid_id(end):
            return _error("invalid_request", "扫描 ID 必须是 0 到 254 的整数")
        if start > end:
            return _error("invalid_request", "扫描起始 ID 不能大于结束 ID")
        return self._submit("scan", start, end, timeout_s=timeout_s)

    def read_pwm(self, servo_id: Any, timeout_s: float = DEFAULT_TIMEOUT_S) -> dict[str, Any]:
        if not self._valid_id(servo_id):
            return _error("invalid_request", "舵机 ID 必须是 0 到 254 的整数")
        return self._submit("read_pwm", servo_id, timeout_s=timeout_s)

    def start_reading(self, servo_id: Any, release_torque: Any = True,
                      timeout_s: float = DEFAULT_TIMEOUT_S) -> dict[str, Any]:
        if not self._valid_id(servo_id, 1, 7):
            return _error("invalid_request", "舵机 ID 必须是 1 到 7 的整数")
        if not isinstance(release_torque, bool):
            return _error("invalid_request", "release_torque 必须是布尔值")
        return self._submit("start_reading", servo_id, release_torque, timeout_s=timeout_s)

    def stop_reading(self, servo_id: Any, timeout_s: float = DEFAULT_TIMEOUT_S) -> dict[str, Any]:
        if not self._valid_id(servo_id, 1, 7):
            return _error("invalid_request", "舵机 ID 必须是 1 到 7 的整数")
        return self._submit("stop_reading", servo_id, timeout_s=timeout_s)

    def readings(self) -> dict[str, Any]:
        with self._lock:
            return {"ok": True, "active_ids": sorted(self._active_ids), "tolerance": self._tolerance,
                    "readings": {key: dict(value) for key, value in self._readings.items()}}

    def status(self, timeout_s: float = DEFAULT_TIMEOUT_S) -> dict[str, Any]:
        del timeout_s
        return {"ok": True, "status": self._status_snapshot()}

    def set_tolerance(self, tolerance: Any) -> dict[str, Any]:
        if not isinstance(tolerance, int) or isinstance(tolerance, bool) or not 0 <= tolerance <= 500:
            return _error("invalid_request", "容差必须是 0 到 500 之间的整数")
        with self._lock:
            self._tolerance = tolerance
        return {"ok": True, "tolerance": tolerance}

    def set_id(self, source_id: Any, target_id: Any, confirm_text: Any,
               auto_release_on_boot: Any = True, *,
               timeout_s: float = DEFAULT_WRITE_TIMEOUT_S) -> dict[str, Any]:
        if not self._valid_id(source_id):
            return _error("invalid_request", "源 ID 必须是 0 到 254 的整数")
        if not self._valid_id(target_id, 1, 7):
            return _error("invalid_request", "目标 ID 必须是 1 到 7 的整数")
        if source_id == target_id:
            return _error("invalid_request", "源 ID 与目标 ID 不能相同")
        if not isinstance(confirm_text, str) or not isinstance(auto_release_on_boot, bool):
            return _error("invalid_request", "确认文本和写入选项类型无效")
        return self._submit("set_id", source_id, target_id, confirm_text,
                            auto_release_on_boot, timeout_s=timeout_s)

    def calibrate_zero(self, servo_id: Any, confirm_text: Any, *,
                       manual_release_confirmed: Any = False,
                       timeout_s: float = DEFAULT_WRITE_TIMEOUT_S) -> dict[str, Any]:
        if not self._valid_id(servo_id, 1, 7):
            return _error("invalid_request", "舵机 ID 必须是 1 到 7 的整数")
        if not isinstance(confirm_text, str):
            return _error("invalid_request", "确认文本必须是字符串")
        if not isinstance(manual_release_confirmed, bool):
            return _error("invalid_request", "人工释力确认必须是布尔值")
        return self._submit("calibrate_zero", servo_id, confirm_text, manual_release_confirmed,
                            timeout_s=timeout_s)

    def startup_torque(self, servo_id: Any, mode: Any, confirm_text: Any,
                       *, timeout_s: float = DEFAULT_WRITE_TIMEOUT_S) -> dict[str, Any]:
        if not self._valid_id(servo_id, 1, 7):
            return _error("invalid_request", "舵机 ID 必须是 1 到 7 的整数")
        if mode not in {"release", "restore"}:
            return _error("invalid_request", "mode 只能是 release 或 restore")
        if not isinstance(confirm_text, str):
            return _error("invalid_request", "确认文本必须是字符串")
        return self._submit("startup_torque", servo_id, mode, confirm_text,
                            timeout_s=timeout_s)

    def shutdown(self, timeout_s: float = DEFAULT_TIMEOUT_S) -> None:
        self._requests.put(None)
        self._thread.join(timeout_s)
