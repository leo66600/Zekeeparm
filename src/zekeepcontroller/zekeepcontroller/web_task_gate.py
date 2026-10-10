"""Exclude ordinary controller commands while a web task owns motion."""
from contextlib import contextmanager
import threading
import time


class WebTaskGate:
    def __init__(self, hardware, *, clock=time.monotonic):
        self.hardware = hardware
        self.owner = ''
        self.active_calls = 0
        self.lock = threading.RLock()
        self.clock = clock
        self.renewed_at = 0.0
        self.expired = False

    def allowed(self, internal=False):
        return bool(self.owner) and not self.expired if internal else not self.owner

    @contextmanager
    def operation(self, internal=False):
        with self.lock:
            if not self.allowed(internal):
                raise RuntimeError('controller motion is reserved for a web task')
            self.active_calls += 1
        try:
            yield
        finally:
            with self.lock:
                self.active_calls -= 1

    def lease(self, request, response):
        try:
            if request.owner not in ('teach', 'grasp', 'ai'):
                raise ValueError('unknown task owner')
            with self.lock:
                if getattr(request, 'renew', False):
                    if self.owner != request.owner or self.expired:
                        raise RuntimeError('task lease expired or owner mismatch')
                    self.renewed_at = self.clock()
                elif request.acquire:
                    if self.owner or self.active_calls or self.hardware.state_machine != 'IDLE':
                        raise RuntimeError('controller is busy; task lease rejected')
                    self.owner = request.owner
                    self.renewed_at = self.clock()
                    self.expired = False
                else:
                    if self.owner != request.owner or self.active_calls:
                        raise RuntimeError('task lease owner mismatch or command still active')
                    if self.hardware.state_machine not in ('IDLE', 'DISABLED'):
                        raise RuntimeError('hold before releasing task ownership')
                    self.owner = ''
                    self.expired = False
            response.success = True
            response.message = 'task lease renewed' if getattr(request, 'renew', False) else ('task lease acquired' if request.acquire else 'task lease released')
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def watchdog(self):
        with self.lock:
            if not self.owner or self.expired or self.clock() - self.renewed_at <= 3.0:
                return
            self.expired = True
        # Retain ownership on failure and reject late task commands.
        self.hardware.stop_and_hold()

    def service(self, callback, *, internal=False):
        def guarded(request, response):
            try:
                with self.operation(internal):
                    return callback(request, response)
            except Exception as exc:
                response.success = False
                if hasattr(response, 'message'):
                    response.message = str(exc)
                return response
        return guarded

    def action(self, callback, result_type, *, internal=False):
        def guarded(handle):
            try:
                with self.operation(internal):
                    return callback(handle)
            except Exception as exc:
                result = result_type()
                if hasattr(result, 'success'):
                    result.success = False
                if hasattr(result, 'error_code'):
                    result.error_code = -1
                if hasattr(result, 'message'):
                    result.message = str(exc)
                if hasattr(result, 'error_string'):
                    result.error_string = str(exc)
                if handle.is_active:
                    handle.abort()
                return result
        return guarded
