"""Plan base_link -> official_tcp motion and report joint errors (m/rad).

Default: offline IK from the configured ready joints. --execute uses live
feedback and SDK Cartesian planning; it does not provide obstacle avoidance.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from utils.native_runtime import preload_environment_libstdcpp

preload_environment_libstdcpp()

import numpy as np
from drivers.robot.official_sdk_robot import OfficialSdkRobot
from utils.camera_utils import load_config


class ReportingRobot(OfficialSdkRobot):
    def _assert_target_reached(self, expected_joints):
        # Report the same stable feedback sample used for the acceptance check.
        actual = self.current_joints()
        expected = np.asarray(expected_joints, dtype=float).reshape(6)
        tolerance = float(self.cfg.get("safety", {}).get("motion_position_tolerance_rad", 0.005))
        errors = actual - expected
        print("Joint  IK target(rad)  Actual(rad)  Error(rad)  Abs error(deg)  Result", flush=True)
        for i, (target, measured, error) in enumerate(zip(expected, actual, errors), 1):
            result = "PASS" if abs(error) <= tolerance else "FAIL"
            print(f"J{i:<5} {target: .6f}      {measured: .6f}    {error: .6f}"
                  f"    {abs(np.degrees(error)):.6f}        {result}", flush=True)
        worst = int(np.argmax(np.abs(errors)))
        print(f"Max: J{worst + 1}, error={abs(errors[worst]):.6f}rad, limit={tolerance:.6f}rad", flush=True)
        if abs(errors[worst]) > tolerance:
            raise RuntimeError(f"joint target did not settle: joint{worst + 1}")


def plan_target(robot, pose, start):
    config = robot.cfg["official_sdk"]
    if not np.all(np.isfinite(pose)):
        raise ValueError("pose must contain finite values")
    for axis, value in zip("xyz", pose[:3]):
        low, high = config["workspace"][axis]
        if not low <= value <= high:
            raise ValueError(f"{axis}={value} outside workspace [{low}, {high}]")
    if pose[2] < config["min_tcp_z_m"]:
        raise ValueError("TCP below minimum height")
    target = robot.kinematics.solve_pose_sequence([pose], start)[0]
    limits = robot.kinematics.joint_limits
    if not np.all(np.isfinite(target)) or np.any(target < limits[:, 0]) or np.any(target > limits[:, 1]):
        raise ValueError("IK target outside joint limits or non-finite")
    if np.max(np.abs(target - start)) > config["max_segment_joint_delta_rad"]:
        raise ValueError("joint segment delta exceeds configured limit")
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config/default.yaml"))
    parser.add_argument("--pose", type=float, nargs=6, required=True,
                        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"))
    parser.add_argument("--duration", type=float, default=6.0, help="motion duration in seconds")
    parser.add_argument("--execute", action="store_true", help="connect and move physical robot")
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be finite and positive")
    cfg = load_config(args.config)
    if cfg["official_sdk"].get("tcp_frame") != "official_tcp":
        parser.error("official_sdk.tcp_frame must be official_tcp")
    pose = np.asarray(args.pose)
    robot = ReportingRobot(cfg)
    try:
        # Check the requested pose from the configured seed before enabling hardware.
        start = np.asarray(cfg["official_sdk"]["ready_joints"], dtype=float)
        target = plan_target(robot, pose, start)
        if not args.execute:
            print("[Offline IK] seed=configured ready joints; no hardware connected")
            print("IK target (rad):", target)
            return 0
        robot.connect()
        start = robot.current_joints()
        target = plan_target(robot, pose, start)
        print("[Move] base_link -> official_tcp:", pose, flush=True)
        robot.move_to_traj_and_wait(pose, target, args.duration)
        return 0
    except (RuntimeError, ValueError) as error:
        print(f"[ERROR] {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        try:
            if robot.control_loop_active:
                # Stop a partially sent trajectory before waiting for shutdown.
                robot.hold_current()
                input("Support the arm if needed; press Enter to disable motors and exit: ")
        finally:
            robot.close()


if __name__ == "__main__":
    raise SystemExit(main())
