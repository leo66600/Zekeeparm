"""Check generated samples with MoveIt and animate them in isolated RViz.

Only publishes DisplayRobotState; never sends a motion action or motor command.
Run the mock demo in ROS_DOMAIN_ID=77 before using this script.
"""
import argparse
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from moveit_msgs.msg import DisplayRobotState
from moveit_msgs.srv import GetStateValidity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path", type=Path, default=Path(__file__).resolve().parents[1] /
                        "data/auto_poses_candidate.npz")
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "77":
        raise RuntimeError("preview requires isolated ROS_DOMAIN_ID=77")
    with np.load(args.path) as archive:
        points = archive["points"].copy()
    rclpy.init()
    node = rclpy.create_node("calibration_path_preview")
    client = node.create_client(GetStateValidity, "/check_state_validity")
    publisher = node.create_publisher(DisplayRobotState, "/display_robot_state", 10)
    try:
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("MoveIt state validity service unavailable")
        message = DisplayRobotState()
        message.state.joint_state.name = [f"joint{i}" for i in range(1, 7)] + ["gripper_joint", "right_joint"]
        # The generator densely checks every sample. MoveIt independently checks
        # each tenth sample plus the endpoint using its current planning scene.
        indices = sorted(set(range(0, len(points), 10)) | {len(points) - 1})
        for count, index in enumerate(indices):
            message.state.joint_state.position = [*map(float, points[index]), 0.035, 0.035]
            publisher.publish(message)
            request = GetStateValidity.Request()
            request.robot_state = message.state
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if not future.done() or future.result() is None:
                raise RuntimeError(f"MoveIt check timed out at sample {index}")
            result = future.result()
            if not result.valid:
                contacts = [(c.contact_body_1, c.contact_body_2) for c in result.contacts]
                raise RuntimeError(f"MoveIt rejected sample {index}: {contacts}")
            if count % 200 == 0:
                print(f"MoveIt checked {count + 1}/{len(indices)} samples", flush=True)
        print(f"PASS: MoveIt checked {len(indices)} samples. Starting RViz preview.", flush=True)
        # Slow enough to observe each perturbation; replay does not touch hardware.
        for index in indices:
            message.state.joint_state.position = [*map(float, points[index]), 0.035, 0.035]
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.025)
        print("RViz preview completed.", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
