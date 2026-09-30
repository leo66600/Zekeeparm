"""Plan base_link -> official_tcp using MoveIt and report six joint errors.

Requires the ROS driver and MoveIt. Default is planning only; --execute moves.
This client never opens the serial port, enables, or disables motors.
"""
from pathlib import Path
import argparse
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from drivers.robot.ros_robot_client import RosRobotClient
from utils.transforms import pose6d_to_mat4


def tcp_transform(urdf):
    root = ET.parse(urdf).getroot()
    joint = root.find("./joint[@name='official_tcp_fixed']")
    if (joint is None or joint.get("type") != "fixed"
            or joint.find("parent").get("link") != "link6"
            or joint.find("child").get("link") != "official_tcp"):
        raise ValueError("expected fixed link6 -> official_tcp transform in SDK URDF")
    origin = joint.find("origin")
    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    return pose6d_to_mat4(*xyz, *rpy)


def execute_and_report(client, trajectory, cfg):
    target = client.trajectory_final_positions(trajectory)
    safety = cfg["safety"]
    tolerance = float(safety["motion_position_tolerance_rad"])
    try:
        client.execute_moveit_trajectory(
            trajectory, timeout_s=float(cfg["robot"]["moveit"]["execution_timeout_s"]))
        client.wait_for_settle(
            target, position_tolerance_rad=tolerance,
            velocity_tolerance_rad_s=float(safety["motion_velocity_tolerance_rad_s"]),
            required_samples=int(safety["motion_settle_samples"]),
            sample_interval_s=float(safety["motion_sample_interval_s"]),
            timeout_s=float(safety["motion_settle_timeout_s"]))
    finally:
        # Also show feedback after a controller failure or settle timeout.
        motion_failed = sys.exc_info()[0] is not None
        try:
            print("[Observe] 5 seconds after execution/settle check; sampling every 0.5s", flush=True)
            started = time.monotonic()
            for index in range(11):
                time.sleep(max(0.0, started + index * 0.5 - time.monotonic()))
                sample = client.latest_sample()
                print(f"\n[Observe t={time.monotonic() - started:.2f}s]", flush=True)
                print("Joint  Target(rad)  Actual(rad)  Error(rad)  Abs(deg)  Velocity(rad/s)  Result", flush=True)
                errors = sample.positions - target
                for i, (goal, actual, error, velocity) in enumerate(
                        zip(target, sample.positions, errors, sample.velocities), 1):
                    result = "PASS" if abs(error) <= tolerance else "FAIL"
                    print(f"J{i}  {goal: .6f}  {actual: .6f}  {error: .6f}  "
                          f"{abs(np.degrees(error)):.6f}  {velocity: .6f}  {result}", flush=True)
                worst = int(np.argmax(np.abs(errors)))
                print(f"Max: J{worst + 1}, error={abs(errors[worst]):.6f}rad; limit={tolerance:.6f}rad", flush=True)
        except Exception as error:
            print(f"[Feedback unavailable] {error}", file=sys.stderr)
            if not motion_failed:
                raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config/default.yaml"))
    parser.add_argument("--pose", type=float, nargs=6, required=True,
                        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"))
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    with open(args.config, encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    pose = np.asarray(args.pose)
    sdk = cfg["official_sdk"]
    if not np.all(np.isfinite(pose)):
        parser.error("pose must contain finite values (m/rad)")
    for axis, value in zip("xyz", pose[:3]):
        low, high = sdk["workspace"][axis]
        if not low <= value <= high:
            parser.error(f"{axis}={value} outside workspace [{low}, {high}]")
    if pose[2] < sdk["min_tcp_z_m"]:
        parser.error("TCP below minimum height")
    robot = cfg["robot"]
    moveit = robot["moveit"]
    client = RosRobotClient(
        namespace=robot["namespace"], joint_state_topic=robot["joint_state_topic"],
        timeout_s=float(robot["ros_timeout_s"]),
        joint_state_max_age_s=float(robot["joint_state_max_age_s"]),
        T_link6_to_end=tcp_transform(PROJECT_ROOT / sdk["urdf"]))
    try:
        client.start(require_gripper=False, require_moveit=True)
        start = client.latest_sample().positions
        target, code = client.solve_end_pose_ik(
            pose6d_to_mat4(*pose), start, timeout_s=1.0, attempts=10)
        if target is None:
            raise RuntimeError(f"MoveIt IK failed: code={code}")
        trajectory = client.plan_joint_goal(
            start, target, pipeline_id=moveit["planning_pipeline_id"],
            planner_id=moveit["planner_id"], num_planning_attempts=int(moveit["planning_attempts"]),
            allowed_planning_time_s=float(moveit["planning_time_s"]),
            max_velocity_scaling_factor=min(0.1, float(moveit["max_velocity_scaling_factor"])),
            max_acceleration_scaling_factor=min(0.1, float(moveit["max_acceleration_scaling_factor"])),
            joint_tolerance_rad=0.001, label="pose_error_test")
        print("[Plan] final joints (rad):", client.trajectory_final_positions(trajectory), flush=True)
        print("[Plan] duration (s):", client.trajectory_duration_s(trajectory), flush=True)
        if args.execute:
            execute_and_report(client, trajectory, cfg)
        else:
            print("[Plan only] add --execute to move")
        return 0
    except (RuntimeError, TimeoutError, ValueError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
