"""Validate task exclusivity and lost-heartbeat hold without motor connections."""
from types import SimpleNamespace
from unittest.mock import Mock
from zekeepcontroller.web_task_gate import WebTaskGate


def test_lease_excludes_normal_commands_and_expiry_holds_until_release():
    clock = [0.0]
    hardware = Mock(state_machine='IDLE')
    gate = WebTaskGate(hardware, clock=lambda: clock[0])
    request = SimpleNamespace(owner='teach', acquire=True, renew=False)
    response = SimpleNamespace(success=False, message='')
    assert gate.lease(request, response).success
    assert not gate.allowed() and gate.allowed(internal=True)
    ordinary = Mock()
    assert not gate.service(ordinary)(None, response).success
    ordinary.assert_not_called()
    clock[0] = 3.1
    gate.watchdog()
    hardware.stop_and_hold.assert_called_once()
    assert not gate.allowed(internal=True) and not gate.allowed()
    request.acquire = False
    assert gate.lease(request, response).success
    assert gate.allowed()
