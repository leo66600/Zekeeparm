"""Check SDK imports, DM defaults, and model paths without connecting hardware."""

import importlib
from pathlib import Path
import pkgutil
import sys

ROOT = Path(__file__).resolve().parents[1]
SDK_ROOT = ROOT / "zekeeparm_SDK"
sys.path.insert(0, str(SDK_ROOT))
sys.path.insert(0, str(ROOT / "src/rebot_grasp"))

import zekeeparm_SDK
from zekeeparm_SDK.actuator import RebotArm, load_cfg
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
    assert Path(urdf_path).resolve() == ROOT / "src/rebot_grasp/config/sixaxis.urdf"
    model = load_robot_model()
    assert model.existFrame(get_end_effector_frame())
    print("SDK imports, DM defaults, controller modes, and model paths: OK")


if __name__ == "__main__":
    main()
