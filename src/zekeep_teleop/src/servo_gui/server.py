"""ZEKEEP 指擎串口舵机标定工具 HTTP 服务。"""

from __future__ import annotations

import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from serial.tools import list_ports

from .bus_worker import CALIBRATION_PLAN, BusWorker, _SimulatedBus


INDEX_HTML = Path(__file__).with_name("static") / "index.html"
MAX_BODY_BYTES = 64 * 1024
NOT_IMPLEMENTED_PATHS = {
    "/api/startup_position": "本工具不实现 PCSD：开机释力（PCSM）会让 PCSD 失效，二者互斥"
}


def _ports(simulate: bool) -> list[dict[str, Any]]:
    if simulate:
        return [{"device": "模拟串口", "description": "ZEKEEP指擎舵机模拟设备",
                 "hwid": "SIMULATED", "vid": None, "pid": None,
                 "serial_number": None, "manufacturer": None, "product": None,
                 "interface": None, "location": None}]
    fields = ("device", "description", "hwid", "vid", "pid", "serial_number",
              "manufacturer", "product", "interface", "location")
    return [{field: getattr(port, field, None) for field in fields}
            for port in list_ports.comports() if port.device.startswith("/dev/ttyUSB")]


def _http_status(result: dict[str, Any]) -> int:
    if result.get("ok"):
        return 200
    code = result.get("error", {}).get("code")
    if code in {"invalid_request", "confirm_mismatch"}:
        return 400
    if code in {"not_connected", "already_connected", "bus_not_exclusive", "wrong_servo",
                "reading_active", "manual_only", "joint_not_stable"}:
        return 409
    if code == "timeout":
        return 504
    if code in {"no_response", "id_write_failed", "boot_release_failed", "pwm_out_of_tolerance"}:
        return 502
    return 503


def make_handler(worker: BusWorker, simulate: bool) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            path = urlsplit(self.path).path
            if path == "/":
                return self._send_index()
            if path == "/api/ports":
                return self._send_json(200, {"ports": _ports(simulate)})
            if path == "/api/status":
                result = worker.status()
                return self._send_json(_http_status(result), result["status"] if result.get("ok") else result)
            if path == "/api/targets":
                plan = {f"{sid:03d}": {"id": sid, **value}
                        for sid, value in sorted(CALIBRATION_PLAN.items())}
                return self._send_json(200, {"plan": plan, "tolerance": worker.readings()["tolerance"]})
            if path == "/api/readings":
                return self._send_json(200, worker.readings())
            if path.startswith("/api/reading/"):
                text = path.removeprefix("/api/reading/")
                if not text.isdigit():
                    return self._send_json(400, {"error": "舵机 ID 必须是整数"})
                result = worker.read_pwm(int(text))
                return self._send_json(_http_status(result), result)
            self._send_json(404, {"error": "未找到接口"})

        def do_POST(self) -> None:
            path = urlsplit(self.path).path
            if path in NOT_IMPLEMENTED_PATHS:
                return self._send_json(501, {"error": NOT_IMPLEMENTED_PATHS[path]})
            try:
                body = self._read_json()
            except ValueError as exc:
                return self._send_json(400, {"error": str(exc)})
            write_fields = {
                "/api/set_id": {"source_id", "target_id", "confirm_text", "auto_release_on_boot"},
                "/api/calibrate_zero": {"id", "confirm_text"},
                "/api/startup_torque": {"id", "mode", "confirm_text"},
            }
            if path in write_fields and body.keys() - write_fields[path]:
                return self._send_json(400, {"error": "请求包含不支持的写入参数"})
            routes = {
                "/api/connect": lambda: worker.open_port(body.get("port"), body.get("baudrate")),
                "/api/disconnect": worker.close_port,
                "/api/scan": lambda: worker.scan(body.get("start", 0), body.get("end", 10)),
                "/api/reading/start": lambda: worker.start_reading(body.get("id"), body.get("release_torque", True)),
                "/api/reading/stop": lambda: worker.stop_reading(body.get("id")),
                "/api/tolerance": lambda: worker.set_tolerance(body.get("tolerance")),
                "/api/set_id": lambda: worker.set_id(body.get("source_id"), body.get("target_id"),
                                                       body.get("confirm_text", ""),
                                                       body.get("auto_release_on_boot", True)),
                "/api/calibrate_zero": lambda: worker.calibrate_zero(body.get("id"),
                                                                       body.get("confirm_text", "")),
                "/api/startup_torque": lambda: worker.startup_torque(body.get("id"), body.get("mode", ""),
                                                                       body.get("confirm_text", "")),
            }
            action = routes.get(path)
            if action is None:
                return self._send_json(404, {"error": "未找到接口"})
            result = action()
            self._send_json(_http_status(result), result)

        def _read_json(self) -> dict[str, Any]:
            try:
                length = int(self.headers.get("Content-Length", "0"), 10)
            except ValueError as exc:
                raise ValueError("Content-Length 无效") from exc
            if not 0 <= length <= MAX_BODY_BYTES:
                raise ValueError(f"请求体不能超过 {MAX_BODY_BYTES} 字节")
            if self.headers.get_content_type() != "application/json":
                raise ValueError("Content-Type 必须是 application/json")
            if length == 0:
                return {}
            try:
                value = json.loads(self.rfile.read(length).decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ValueError("请求体必须是有效的 JSON") from exc
            if not isinstance(value, dict):
                raise ValueError("JSON 请求体必须是对象")
            return value

        def _send_json(self, status: int, payload: dict[str, Any]) -> None:
            data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.end_headers()
            self.wfile.write(data)

        def _send_index(self) -> None:
            try:
                data = INDEX_HTML.read_bytes()
            except OSError as exc:
                return self._send_json(500, {"error": f"无法读取界面文件：{exc}"})
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self' 'unsafe-inline'; script-src 'self' 'unsafe-inline'; img-src data:; connect-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(data)

    return Handler


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ZEKEEP 指擎串口舵机标定工具")
    parser.add_argument("--port", type=int, default=8770, help="HTTP 端口（默认 8770）")
    parser.add_argument("--simulate", action="store_true", help="使用模拟舵机，不访问串口")
    parser.add_argument("--simulate-profile", choices=_SimulatedBus.PROFILES, default="assembled")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    worker = BusWorker(simulate=args.simulate, profile=args.simulate_profile)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(worker, args.simulate))
    print(f"ZEKEEP 指擎舵机工具：http://127.0.0.1:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        worker.shutdown()


if __name__ == "__main__":
    main()
