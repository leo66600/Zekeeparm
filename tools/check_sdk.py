"""Check SDK imports, DM defaults, and model paths without connecting hardware."""

import importlib
from pathlib import Path
import pkgutil
import sys

ROOT = Path(__file__).resolve().parents[1]
SDK_ROOT = ROOT / "zekeeparm_SDK"
sys.path.insert(0, str(SDK_ROOT))
sys.path.insert(0, str(ROOT / "src/zekeep_grasp"))

import zekeeparm_SDK
from zekeeparm_SDK.actuator import RebotArm, load_cfg
from zekeeparm_SDK.controllers import RebotArmEndPose
from zekeeparm_SDK.kinematics import get_end_effector_frame, load_robot_model
from zekeeparm_SDK.kinematics.robot_model import _resolve_urdf
from drivers.robot.grasp_driver import (
    ensure_rebot_sdk_in_syspath,
    selected_arm_config,
    selected_hardware_yaml,
)


def main():
    assert Path(zekeeparm_SDK.__file__).resolve().parent == SDK_ROOT / "zekeeparm_SDK"
    for module in pkgutil.walk_packages(zekeeparm_SDK.__path__, "zekeeparm_SDK."):
        importlib.import_module(module.name)
    assert sorted(p.name for p in (SDK_ROOT / "config").glob("*.yaml")) == [
        "rebotarm_dm.yaml"
    ]
    assert not (SDK_ROOT / "urdf").exists()
    assert ensure_rebot_sdk_in_syspath() == SDK_ROOT
    assert ensure_rebot_sdk_in_syspath("../../zekeeparm_SDK") == SDK_ROOT
    assert selected_hardware_yaml() == SDK_ROOT / "config/rebotarm_dm.yaml"
    assert selected_arm_config().arm_type == "dm"
    assert selected_arm_config().controller_mode == "posvel"
    assert selected_arm_config(controller_mode="mit").controller_mode == "mit"
    try:
        selected_arm_config(controller_mode="invalid")
    except ValueError:
        pass
    else:
        raise AssertionError("Invalid controller mode was accepted")
    arm = RebotArm()
    assert not arm._connected
    assert arm.hardware_yaml == "rebotarm_dm.yaml"
    assert arm.num_joints == 7 and arm.has_gripper
    assert all(j.vendor == "damiao" for j in load_cfg()["joints"])
    urdf_path, _ = _resolve_urdf()
    assert Path(urdf_path).resolve() == ROOT / "src/zekeep_grasp/config/sixaxis.urdf"
    model = load_robot_model()
    assert model.existFrame(get_end_effector_frame())
    assert model.nq == 8
    controller = RebotArmEndPose.__new__(RebotArmEndPose)
    controller.set_gripper_target(1.35)
    for invalid in (-.01, 1.36, 1.45, float("nan")):
        try:
            controller.set_gripper_target(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError("SDK accepted an out-of-range gripper target")
    assert controller._gripper_target == 1.35
    print("SDK imports, ROS model loading, and gripper limits: OK")


if __name__ == "__main__":
    main()
