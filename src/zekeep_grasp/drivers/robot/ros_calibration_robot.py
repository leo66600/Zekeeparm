"""ROS-backed robot adapter for eye-in-hand calibration.

The ROS hardware layer is the single source of calibrated joint positions.
This module deliberately never opens the motor serial port and never applies
joint direction, ratio, or zero-offset mapping a second time.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from drivers.robot.grasp_driver import ensure_rebot_sdk_in_syspath
from drivers.robot.ros_robot_client import RosRobotClient
from utils.transforms import pose6d_to_mat4
from utils.robot_model_config import resolve_robot_model_config


class RosCalibrationRobot:
    """Expose the small robot interface needed by the calibration collector."""

    def __init__(
        self,
        client: Any,
        tcp_pose_from_joints: Callable[[np.ndarray], np.ndarray],
    ) -> None:
        self._client = client
        self._tcp_pose_from_joints = tcp_pose_from_joints

    def start(self) -> None:
        # Calibration does not move the gripper, so a gripper communication
        # problem must not prevent hand-eye data collection.
        self._client.start(require_gripper=False)

    def start_manual(self) -> None:
        """Connect to ROS and enter controller-owned gravity compensation."""
        try:
            self.start()
            self._client.start_gravity_compensation()
        except BaseException:
            self.close()
            raise

    def get_tcp_pose(self) -> np.ndarray:
        # These values have already passed through HardwareManager's
        # direction/ratio/encoder-zero/ROS-zero mapping.
        mapped_q = np.asarray(
            self._client.latest_sample().positions,
            dtype=np.float64,
        ).reshape(-1)
        if mapped_q.size < 6 or not np.all(np.isfinite(mapped_q[:6])):
            raise RuntimeError(
                "joint_states must contain six finite mapped joint positions"
            )
        transform = np.asarray(
            self._tcp_pose_from_joints(mapped_q[:6].copy()),
            dtype=np.float64,
        )
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise RuntimeError("mapped joint FK returned an invalid TCP transform")
        return transform.copy()

    def get_joint_positions(self) -> np.ndarray:
        """Return the latest six mapped ROS joint positions."""
        mapped_q = np.asarray(
            self._client.latest_sample().positions,
            dtype=np.float64,
        ).reshape(-1)
        if mapped_q.size < 6 or not np.all(np.isfinite(mapped_q[:6])):
            raise RuntimeError(
                "joint_states must contain six finite mapped joint positions"
            )
        return mapped_q[:6].copy()

    def move_to_pose(
        self,
        pose6d: Sequence[float],
        *,
        duration: float,
    ) -> bool:
        values = tuple(float(value) for value in pose6d)
        if len(values) != 6 or not np.all(np.isfinite(values)):
            raise ValueError("calibration pose must contain six finite values")
        return bool(
            self._client.move_end_pose(
                pose6d_to_mat4(*values),
                float(duration),
            )
        )

    def close(self) -> None:
        """Close only this ROS client; the hardware controller keeps holding."""
        self._client.close()

    def close_manual(self) -> None:
        """Leave gravity compensation in position hold, then close the client."""
        try:
            self._client.stop_gravity_compensation()
        finally:
            self.close()


def make_ros_calibration_robot(
    robot_cfg: Mapping[str, Any],
) -> RosCalibrationRobot:
    """Build a calibration adapter using the same FK frame as grasp runtime."""
    ensure_rebot_sdk_in_syspath(robot_cfg.get("repo_root"))
    from zekeeparm_SDK.kinematics import (
        compute_fk,
        load_robot_model,
        pad_q_for_model,
    )

    project_root = Path(__file__).resolve().parents[2]
    urdf_path, end_frame = resolve_robot_model_config(project_root, robot_cfg)
    model = load_robot_model(str(urdf_path))
    q_zero = np.zeros(model.nq, dtype=np.float64)
    T_base_to_link6 = compute_fk(model, q_zero, frame_name="link6")[2]
    T_base_to_end = compute_fk(model, q_zero, frame_name=end_frame)[2]
    T_link6_to_end = np.linalg.inv(T_base_to_link6) @ T_base_to_end

    def tcp_pose_from_mapped_joints(mapped_q: np.ndarray) -> np.ndarray:
        model_q = pad_q_for_model(model, mapped_q, 6)
        return compute_fk(model, model_q, frame_name=end_frame)[2]

    client = RosRobotClient(
        namespace=str(robot_cfg.get("namespace", "zekeep")),
        joint_state_topic=str(
            robot_cfg.get("joint_state_topic", "/zekeep/joint_states")
        ),
        timeout_s=float(robot_cfg.get("ros_timeout_s", 8.0)),
        joint_state_max_age_s=float(
            robot_cfg.get("joint_state_max_age_s", 0.5)
        ),
        T_link6_to_end=T_link6_to_end,
    )
    return RosCalibrationRobot(client, tcp_pose_from_mapped_joints)
