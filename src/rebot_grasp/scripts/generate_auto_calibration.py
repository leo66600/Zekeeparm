"""Generate and collision-check automatic calibration paths without hardware.

Uses the ROS controller's actual Cartesian planner, current URDF/SRDF, and
POS_VEL limits. Output remains a candidate until reviewed in RViz. Camera
visibility and physical calibration accuracy require on-site verification.
"""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import argparse
import hashlib
import json
import sys

import numpy as np
import pinocchio as pin
import yaml
import coal

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(WORKSPACE / "third_party/reBotArm_control_py"))
from reBotArm_control_py.controllers import RebotArmEndPose
import reBotArm_control_py.kinematics.robot_model as sdk_model
from utils.transforms import mat4_to_pose6d


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def candidate_joints():
    # Center near the previously visible board, recomputed with the current model.
    center = np.array([0.0, 1.45, 0.97, -0.36, 0.0, 0.0])
    targets = [center * f for f in (0.25, 0.5, 0.75, 1.0)]
    # Each observation changes position and all three wrist joints. Complementary
    # sign ordering avoids the highly correlated center-to-single-axis pattern.
    signs = ((1, 1, 1), (-1, -1, -1), (1, 1, -1), (-1, -1, 1),
             (1, -1, -1), (-1, 1, 1), (1, -1, 1), (-1, 1, -1))
    positions = ((0.08, 0.06), (-0.08, -0.06),
                 (0.08, -0.06), (-0.08, 0.06))
    for wrist in ((0.14, 0.18, 0.18), (0.18, 0.22, 0.22)):
        for index, sign in enumerate(signs):
            q = center.copy()
            q[1:3] += positions[index % len(positions)]
            q[3:] += np.asarray(sign) * wrist
            targets.append(q)
    return targets


def generate(output):
    urdf = WORKSPACE / "src/zekeep_bringup/description/urdf/sixaxis.urdf"
    srdf = WORKSPACE / "src/zekeep_moveit_config/config/zekeep.srdf"
    hardware = WORKSPACE / "src/zekeep_bringup/config/zekeep_hardware.yaml"
    hw = yaml.safe_load(hardware.read_text())["models"]["sixaxis"]["overrides"]
    assert hw["control"]["arm_control_mode"] == "posvel"
    sdk_model._hw_cfg_cache = dict(hw, urdf_path=str(urdf), end_effector_frame="link6")
    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    geometry = pin.buildGeomFromUrdf(model, str(urdf), pin.GeometryType.COLLISION,
                                     package_dirs=[str(WORKSPACE / "src")])
    geometry.addAllCollisionPairs()
    pin.removeCollisionPairs(model, geometry, str(srdf))
    table = pin.GeometryObject("fixed_worktable", 0,
        pin.SE3(np.eye(3), np.array([0.388, 0.0, -0.4])), coal.Box(0.9, 1.2, 0.8))
    table_id = geometry.addGeometryObject(table)
    for index, obj in enumerate(geometry.geometryObjects):
        if index != table_id and model.frames[obj.parentFrame].name != "base_link":
            geometry.addCollisionPair(pin.CollisionPair(index, table_id))
    gd = pin.GeometryData(geometry)
    names = [f"joint{i}" for i in range(1, 7)]
    limits = np.array([[hw["joint_calibration"][n]["hard_lower"],
                        hw["joint_calibration"][n]["hard_upper"]] for n in names])
    vlim = np.array([hw["control"]["pos_vel"][n]["vlim"] for n in names])
    q = pin.neutral(model)
    group = SimpleNamespace(num_joints=6, _pv_vlim=vlim)
    robot = SimpleNamespace(groups={"arm": group}, has_gripper=False,
                            get_state=lambda: (q.copy(), np.zeros(model.nv), np.zeros(model.nv)))
    controller = RebotArmEndPose(robot, urdf_path=urdf, end_effector_frame="link6")
    controller._running = True
    poses, segments, durations, joints = [], [], [], []
    max_error = 0.0

    def check(sample, label):
        if np.any(sample[:6] < limits[:, 0] - 1e-8) or np.any(sample[:6] > limits[:, 1] + 1e-8):
            raise ValueError(f"{label}: hardware joint limit exceeded: {sample[:6]}")
        # Closed fingers collide with a rod in the current CAD model. Validate
        # only a fixed open gripper; do not exempt that collision pair.
        for opening in (0.035,):
            full = pin.neutral(model)
            full[:6] = sample[:6]
            full[6:] = opening
            if pin.computeCollisions(model, data, geometry, gd, full, False):
                pairs = [(geometry.geometryObjects[p.first].name,
                          geometry.geometryObjects[p.second].name)
                         for p, result in zip(geometry.collisionPairs, gd.collisionResults)
                         if result.isCollision()]
                raise ValueError(f"{label}: collision at opening={opening}: {pairs}")

    check(q, "zero start")
    for index, target_q in enumerate(candidate_joints(), 1):
        full = pin.neutral(model)
        full[:6] = target_q
        pin.forwardKinematics(model, data, full)
        pin.updateFramePlacements(model, data)
        target = data.oMf[model.getFrameId("link6")].copy()
        pose = mat4_to_pose6d(data.oMf[model.getFrameId("gripper_base")].homogeneous)
        # No send thread or serial interface exists during generation.
        with patch("reBotArm_control_py.controllers.rebotarm_endpose_controller.threading.Thread"):
            if not controller.move_to_traj(*mat4_to_pose6d(target.homogeneous), duration=8.0):
                raise ValueError(f"pose {index}: controller IK/path generation failed")
        points = np.asarray(controller._traj)
        for sample_index, sample in enumerate(points):
            check(sample, f"pose {index} sample {sample_index}")
        q[:6] = points[-1]
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        error = np.linalg.norm(pin.log6(data.oMf[model.getFrameId("link6")].inverse() * target).vector)
        if error > 1e-4:
            raise ValueError(f"pose {index}: final Cartesian residual {error}")
        max_error = max(max_error, float(error))
        poses.append(list(pose))
        joints.append(q[:6].tolist())
        segments.append(points)
        durations.append(float(controller._traj_duration))
        print(f"{index:02d}: checked {len(points)} samples, duration={durations[-1]:.2f}s", flush=True)
    canonical = json.dumps(poses, separators=(",", ":"), ensure_ascii=True).encode("ascii")
    result = dict(base_frame="base_link", reference_frame="gripper_base", position_unit="meter",
        angle_unit="radian", euler_convention="intrinsic_zyx_rpy",
        validated_start_joint_positions=[0.0] * 6, start_tolerance_rad=0.05,
        move_duration_s=8.0, source="current-model coupled multi-axis poses; first three poses are approach stages",
        pose_sha256=hashlib.sha256(canonical).hexdigest(),
        validated_model=dict(urdf_sha256=digest(urdf), srdf_sha256=digest(srdf),
                             hardware_sha256=digest(hardware)),
        validation_result=dict(controller_path_model="cartesian_geodesic_clik",
            endpoint_ik_success=len(poses), path_samples=sum(map(len, segments)),
            modeled_self_collisions=0, modeled_worktable_collisions=0,
            maximum_endpoint_se3_error=max_error, maximum_duration_s=max(durations),
            gripper_openings_checked_m=[0.035], rviz_review="pending",
            camera_visibility="requires live verification"),
        poses=poses)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(result, sort_keys=False))
    np.savez_compressed(output.with_suffix(".npz"), points=np.concatenate(segments),
        lengths=np.array(list(map(len, segments))), durations=durations, endpoints=joints)
    print(f"Candidate saved: {output}", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "data/auto_poses_candidate.yaml")
    generate(parser.parse_args().output)
