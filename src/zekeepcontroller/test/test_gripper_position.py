"""Position commands recover configured gains without connecting any motors."""
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from zekeepcontroller.hardware_manager import HardwareManager
from zekeepcontroller.joint_mapping import JointCalibration, JointMapping


def manager():
    hardware = HardwareManager.__new__(HardwareManager)
    hardware._cmd_lock = threading.RLock()
    hardware._begin_gripper_command = Mock()
    hardware._gripper_mapping = JointMapping([JointCalibration(
        'gripper', hard_lower=0.0, hard_upper=1.35)])
    hardware._gripper_mit_kp = np.array([8.0])
    hardware._gripper_mit_kd = np.array([1.0])
    hardware._gripper_group = SimpleNamespace(
        _mit_kp=np.zeros(1), _mit_kd=np.zeros(1), send_mit=Mock())
    hardware._endpos_ctrl = SimpleNamespace(_gripper_target=0.0)
    hardware._endpos_ctrl.set_gripper_target = lambda value: setattr(
        hardware._endpos_ctrl, '_gripper_target', value)
    hardware._gripper_target_position = None
    hardware._gripper_grasp_state = object()
    return hardware


def test_open_after_free_gripper_restores_gains_for_command_and_hold_loop():
    hardware = manager()
    hardware.set_gripper_target(1.35)
    hardware._send_gripper_hold_once()
    hardware._begin_gripper_command.assert_called_once_with(allow_endpos=True)
    assert hardware._gripper_grasp_state is None
    assert hardware._gripper_target_position == 1.35
    for call in hardware._gripper_group.send_mit.call_args_list:
        np.testing.assert_allclose(call.args[0], [1.35])
        np.testing.assert_allclose(call.kwargs['kp'], [8.0])
        np.testing.assert_allclose(call.kwargs['kd'], [1.0])
    hardware._gripper_group._mit_kp.fill(0)
    np.testing.assert_allclose(hardware._gripper_mit_kp, [8.0])


@pytest.mark.parametrize('target', [-0.1, 1.36, 1.45, 2.0, float('nan')])
def test_invalid_target_does_not_restore_gains_or_send(target):
    hardware = manager()
    with pytest.raises(ValueError):
        hardware.set_gripper_target(target)
    hardware._gripper_group.send_mit.assert_not_called()
    np.testing.assert_allclose(hardware._gripper_group._mit_kp, [0.0])


def test_rejected_command_does_not_restore_gains_or_send():
    hardware = manager()
    hardware._begin_gripper_command.side_effect = RuntimeError('disabled')
    with pytest.raises(RuntimeError, match='disabled'):
        hardware.set_gripper_target(1.35)
    hardware._gripper_group.send_mit.assert_not_called()
    np.testing.assert_allclose(hardware._gripper_group._mit_kp, [0.0])
