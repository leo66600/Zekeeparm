"""Bind local calibration to a device, installation, and configuration."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path


def calibration_identity(project_root, cfg, camera_serial):
    calibration = cfg.get("calibration") or {}
    camera = cfg.get("camera") or {}
    robot = cfg.get("robot") or {}
    serial = str(camera_serial or "").strip()
    if not serial or serial != str(camera.get("serial") or "").strip():
        raise ValueError("camera serial missing or differs from camera.serial")
    robot_id = str(robot.get("id") or "").strip()
    installation_id = str(calibration.get("installation_id") or "").strip()
    if not robot_id or not installation_id:
        raise ValueError("set robot.id and calibration.installation_id for this installation")
    directory = Path(project_root) / "config" / "calibration" / str(camera["type"]).lower()
    hashes = {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
              for name in ("hand_eye.npz", "intrinsics.npz")}
    # Paths and usernames can change; geometry, compensation and fixed targets cannot.
    site = {key: cfg.get(key) for key in ("calibration", "place", "safety")}
    site["robot"] = {key: robot.get(key) for key in
                     ("joint_mapping", "ready_pose", "moveit", "grasp_reference_frame", "end_effector_frame")}
    site["official_sdk"] = {key: value for key, value in (cfg.get("official_sdk") or {}).items()
                            if key not in ("socket_path", "request_timeout_s", "urdf")}
    if robot.get("urdf_path"):
        model = Path(str(robot["urdf_path"]))
        if not model.is_absolute():
            model = Path(project_root) / model
        hashes["robot_urdf"] = hashlib.sha256(model.read_bytes()).hexdigest()
    site["camera"] = {key: camera.get(key) for key in
                      ("type", "serial", "color_width", "color_height", "color_distortion_mode")}
    return {"version": 1, "camera_serial": serial, "robot_id": robot_id,
            "installation_id": installation_id,
            "reference_frame": robot.get("end_effector_frame"), "files": hashes,
            "configuration_sha256": hashlib.sha256(json.dumps(site, sort_keys=True).encode()).hexdigest()}


def validate_calibration_identity(project_root, cfg, camera_serial):
    expected = calibration_identity(project_root, cfg, camera_serial)
    path = Path(project_root) / "config" / "calibration" / str(cfg["camera"]["type"]).lower() / "identity.local.json"
    if not path.is_file():
        raise ValueError("calibration identity unconfirmed; run scripts/confirm_calibration.py after checking this installation")
    recorded = json.loads(path.read_text())
    if recorded != expected:
        changed = [key for key in recorded.keys() | expected.keys()
                   if recorded.get(key) != expected.get(key) and key != "files"]
        old_files, new_files = recorded.get("files", {}), expected["files"]
        changed.extend(f"files.{key}" for key in old_files.keys() | new_files.keys()
                       if old_files.get(key) != new_files.get(key))
        raise ValueError(
            f"calibration identity mismatch ({', '.join(sorted(changed))}); "
            "changed device/installation requires recalibration"
        )


def confirm_calibration_identity(project_root, cfg, camera_serial):
    identity = calibration_identity(project_root, cfg, camera_serial)
    path = Path(project_root) / "config" / "calibration" / str(cfg["camera"]["type"]).lower() / "identity.local.json"
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(identity, indent=2) + "\n")
    temporary.replace(path)
    return path
