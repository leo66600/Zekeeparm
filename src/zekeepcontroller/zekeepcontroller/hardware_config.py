from __future__ import annotations

import copy
import math
import sys
import tempfile
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)


def resolve_hardware_config(
    hardware_config: str | None,
    model: str,
    channel: str,
) -> tuple[Path, dict[str, Any]]:
    """Resolve ROS and SDK configuration into one runtime hardware config."""
    sdk_root = _ensure_rebot_sdk_in_syspath()
    model_name, data = _load_ros_hardware_config(
        sdk_root,
        hardware_config,
        model,
        channel,
    )
    path = _write_resolved_hardware_config(model_name, data)
    _sync_sdk_robot_model_config(data)
    return path, copy.deepcopy(data)


def _workspace_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "zekeeparm_SDK").is_dir():
            return parent
    return here.parents[3]


def _ensure_rebot_sdk_in_syspath() -> Path:
    root = _workspace_root() / "zekeeparm_SDK"
    if not (root / "zekeeparm_SDK").is_dir():
        raise FileNotFoundError(
            f"Cannot find zekeeparm_SDK at {root}. "
            "Restore zekeeparm_SDK from the workspace bundle."
        )
    root_str = str(root)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)
    return root


def _default_hardware_config_path() -> Path:
    try:
        path = (
            Path(get_package_share_directory("zekeep_bringup"))
            / "config"
            / "zekeep_hardware.yaml"
        )
        if path.exists():
            return path
    except PackageNotFoundError:
        pass
    return (
        _workspace_root()
        / "src"
        / "zekeep_bringup"
        / "config"
        / "zekeep_hardware.yaml"
    )


def resolve_package_uri(value: str) -> Path:
    """Resolve a filesystem path or ``package://`` URI."""
    prefix = "package://"
    if not value.startswith(prefix):
        return Path(value).expanduser()

    package_and_path = value[len(prefix) :]
    package_name, separator, relative_path = package_and_path.partition("/")
    if not separator or not package_name or not relative_path:
        raise ValueError(f"invalid package URI: {value}")

    try:
        installed = Path(get_package_share_directory(package_name)) / relative_path
        if installed.is_file():
            return installed
    except PackageNotFoundError:
        pass

    source = _workspace_root() / "src" / package_name / relative_path
    if source.is_file():
        return source
    raise FileNotFoundError(f"cannot resolve package URI: {value}")


def _deep_merge(base: Any, override: Any) -> Any:
    if isinstance(base, dict) and isinstance(override, dict):
        merged = copy.deepcopy(base)
        for key, value in override.items():
            merged[key] = _deep_merge(merged.get(key), value)
        return merged
    return copy.deepcopy(override)


def _apply_pos_vel_overrides(data: dict[str, Any]) -> None:
    overrides = (data.get("control", {}) or {}).get("pos_vel", {}) or {}
    if not isinstance(overrides, dict):
        raise ValueError("control.pos_vel must be a mapping keyed by joint name")

    joints_by_name = {
        str(joint.get("name")): joint for joint in data.get("joints", [])
    }
    unknown = sorted(set(overrides) - set(joints_by_name))
    if unknown:
        raise ValueError(
            "control.pos_vel references unknown joints: " + ", ".join(unknown)
        )

    for name, values in overrides.items():
        if not isinstance(values, dict):
            raise ValueError(f"control.pos_vel.{name} must be a mapping")
        joint = joints_by_name[name]
        joint["POS_VEL"] = _deep_merge(joint.get("POS_VEL", {}), values)


def _validate_pos_vel_config(data: dict[str, Any]) -> list[float]:
    """Validate arm POS_VEL parameters and return configured velocity limits."""
    arm_joints = _arm_joint_names(data)
    joint_map = {str(joint.get("name")): joint for joint in data.get("joints", [])}
    calibration = data.get("joint_calibration", {}) or {}
    required = ("vel_kp", "vel_ki", "pos_kp", "pos_ki", "vlim")
    vlims: list[float] = []

    for name in arm_joints:
        joint = joint_map.get(name)
        if joint is None:
            raise ValueError(f"groups.arm references unknown joint {name!r}")
        params = joint.get("POS_VEL") or {}
        missing = [key for key in required if key not in params]
        if missing:
            raise ValueError(
                f"POS_VEL parameters missing for {name}: {', '.join(missing)}"
            )
        for key in required:
            value = float(params[key])
            if not math.isfinite(value):
                raise ValueError(f"POS_VEL.{key} for {name} must be finite")
            if key == "vlim" and value <= 0.0:
                raise ValueError(f"POS_VEL.vlim for {name} must be > 0")
            if key != "vlim" and value < 0.0:
                raise ValueError(f"POS_VEL.{key} for {name} must be >= 0")
        vlim = float(params["vlim"])
        max_velocity = (calibration.get(name) or {}).get("max_velocity")
        if max_velocity is not None and vlim > float(max_velocity):
            raise ValueError(
                f"POS_VEL.vlim for {name} exceeds joint max_velocity"
            )
        vlims.append(vlim)
    return vlims


def _load_ros_hardware_config(
    sdk_root: Path,
    hardware_config: str | None,
    model: str,
    channel: str,
) -> tuple[str, dict[str, Any]]:
    config_path = (
        Path(hardware_config).expanduser()
        if hardware_config
        else _default_hardware_config_path()
    )
    if not config_path.exists():
        raise FileNotFoundError(f"ROS hardware config not found: {config_path}")

    with open(config_path, "r", encoding="utf-8") as f:
        ros_config = yaml.safe_load(f) or {}

    model_name = (model or ros_config.get("default_model") or "sixaxis").strip().lower()
    models = ros_config.get("models", {})
    if model_name not in models:
        choices = ", ".join(sorted(models))
        raise ValueError(f"unknown hardware model {model_name!r}; choices: {choices}")

    model_config = models[model_name] or {}
    sdk_config = model_config.get("sdk_config")
    if not sdk_config:
        raise ValueError(f"models.{model_name}.sdk_config is required")

    sdk_config_path = Path(str(sdk_config)).expanduser()
    if not sdk_config_path.is_absolute():
        sdk_config_path = sdk_root / "config" / sdk_config_path
    if not sdk_config_path.exists():
        raise FileNotFoundError(f"SDK hardware config not found: {sdk_config_path}")

    with open(sdk_config_path, "r", encoding="utf-8") as f:
        merged = yaml.safe_load(f) or {}

    merged = _deep_merge(merged, model_config.get("overrides", {}) or {})
    _apply_pos_vel_overrides(merged)
    if merged.get("urdf_path"):
        merged["urdf_path"] = str(resolve_package_uri(str(merged["urdf_path"])))
    if channel:
        merged["channel"] = channel
    _add_runtime_config(merged)

    return model_name, merged


def _add_runtime_config(data: dict[str, Any]) -> dict[str, Any]:
    arm_joints = _arm_joint_names(data)
    n = len(arm_joints)
    gravity_config = data.get("gravity_compensation", {}) or {}
    control_config = data.get("control", {}) or {}
    arm_control_mode = _arm_control_mode(data)
    gravity_feedforward = control_config.get("gravity_feedforward", {}) or {}
    if not isinstance(gravity_feedforward, dict):
        raise ValueError("control.gravity_feedforward must be a mapping")
    gravity_feedforward_enabled = gravity_feedforward.get("enabled", False)
    if not isinstance(gravity_feedforward_enabled, bool):
        raise ValueError("control.gravity_feedforward.enabled must be boolean")
    configured_ff_joints = gravity_feedforward.get("joints", None)
    if configured_ff_joints is None:
        ff_joints = list(arm_joints) if arm_control_mode == "mit" else []
    else:
        if not isinstance(configured_ff_joints, list) or any(
            not isinstance(name, str) or not name.strip()
            for name in configured_ff_joints
        ):
            raise ValueError(
                "control.gravity_feedforward.joints must be a list of names"
            )
        ff_joints = [name.strip() for name in configured_ff_joints]
        if len(ff_joints) != len(set(ff_joints)):
            raise ValueError(
                "control.gravity_feedforward.joints must not contain duplicates"
            )
    if gravity_feedforward_enabled and not ff_joints:
        raise ValueError(
            "control.gravity_feedforward.joints must be a non-empty list"
        )
    unknown_ff_joints = sorted(set(ff_joints) - set(arm_joints))
    if unknown_ff_joints:
        raise ValueError(
            "control.gravity_feedforward.joints references unknown arm joints: "
            + ", ".join(unknown_ff_joints)
        )
    pos_vel_vlim = (
        _validate_pos_vel_config(data) if arm_control_mode == "posvel" else None
    )

    data["_runtime"] = {
        "control": {
            "arm_control_mode": arm_control_mode,
            "gravity_feedforward_enabled": gravity_feedforward_enabled,
            "gravity_feedforward_joints": ff_joints,
            "gravity_feedforward_mask": [name in ff_joints for name in arm_joints],
            "mit_kp": _control_gain(data, arm_joints, control_config, "mit_kp", "kp"),
            "mit_kd": _control_gain(data, arm_joints, control_config, "mit_kd", "kd"),
            "pos_vel_vlim": pos_vel_vlim,
        },
        "gravity_compensation": {
            "kp": _gravity_gain(data, arm_joints, gravity_config, "kp"),
            "kd": _gravity_gain(data, arm_joints, gravity_config, "kd"),
            "tau_scale": _runtime_vector(
                gravity_config.get("tau_scale", 1.0),
                n,
                "gravity_compensation.tau_scale",
            ),
            "tau_scale_positive": _runtime_vector(
                gravity_config.get(
                    "tau_scale_positive",
                    gravity_config.get("tau_scale", 1.0),
                ),
                n,
                "gravity_compensation.tau_scale_positive",
            ),
            "tau_limit": _runtime_vector(
                gravity_config.get("tau_limit", float("inf")),
                n,
                "gravity_compensation.tau_limit",
            ),
            "tau_rate_limit": _runtime_vector(
                gravity_config.get("tau_rate_limit", float("inf")),
                n,
                "gravity_compensation.tau_rate_limit",
            ),
        }
    }
    return data


def _arm_control_mode(data: dict[str, Any]) -> str:
    mode = str(
        (data.get("control", {}) or {}).get("arm_control_mode", "posvel")
    ).strip().lower()
    if mode == "pos_vel":
        mode = "posvel"
    if mode not in ("posvel", "mit"):
        raise ValueError("control.arm_control_mode must be 'posvel' or 'mit'")
    return mode


def _arm_joint_names(data: dict[str, Any]) -> list[str]:
    joints = data.get("groups", {}).get("arm", {}).get("joints", [])
    if not joints:
        raise ValueError("hardware config must define groups.arm.joints")
    return [str(name) for name in joints]


def _gravity_gain(
    data: dict[str, Any],
    arm_joints: list[str],
    gravity_config: dict[str, Any],
    key: str,
) -> list[float]:
    if key in gravity_config:
        return _runtime_vector(
            gravity_config[key],
            len(arm_joints),
            f"gravity_compensation.{key}",
        )

    joint_map = {str(joint.get("name")): joint for joint in data.get("joints", [])}
    gains = []
    for name in arm_joints:
        joint = joint_map.get(name)
        if joint is None:
            raise ValueError(f"groups.arm references unknown joint {name!r}")
        gains.append(float((joint.get("MIT", {}) or {}).get(key, 0.0)))
    return gains


def _control_gain(
    data: dict[str, Any],
    arm_joints: list[str],
    control_config: dict[str, Any],
    config_key: str,
    mit_key: str,
) -> list[float]:
    if config_key in control_config:
        return _runtime_vector(
            control_config[config_key],
            len(arm_joints),
            f"control.{config_key}",
        )

    joint_map = {str(joint.get("name")): joint for joint in data.get("joints", [])}
    gains = []
    for name in arm_joints:
        joint = joint_map.get(name)
        if joint is None:
            raise ValueError(f"groups.arm references unknown joint {name!r}")
        gains.append(float((joint.get("MIT", {}) or {}).get(mit_key, 0.0)))
    return gains


def _runtime_vector(value: Any, size: int, label: str) -> list[float]:
    if isinstance(value, (int, float)):
        return [float(value)] * size
    if not isinstance(value, list):
        raise ValueError(f"{label} must be a scalar or {size} values")
    values = [float(item) for item in value]
    if len(values) == 1:
        return values * size
    if len(values) != size:
        raise ValueError(f"{label} must be a scalar or {size} values")
    return values


_resolved_config_dir: Path | None = None


def _write_resolved_hardware_config(model: str, data: dict[str, Any]) -> Path:
    global _resolved_config_dir
    if _resolved_config_dir is None:
        _resolved_config_dir = Path(tempfile.mkdtemp(prefix="zekeep_ros2_"))
    tmp_path = _resolved_config_dir / f"{model}_hardware.yaml"
    with open(tmp_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return tmp_path


def _sync_sdk_robot_model_config(data: dict[str, Any]) -> None:
    import zekeeparm_SDK.kinematics.robot_model as robot_model
    import zekeeparm_SDK.dynamics.robot_model as dynamics_model

    robot_model._hw_cfg_cache = copy.deepcopy(data)
    dynamics_model._CACHED_MODEL = None
