"""Check ROS/SDK FK and six-axis IK in the vision environment, without hardware."""
from pathlib import Path
import sys

import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/zekeep_grasp'))
from drivers.robot.official_sdk_robot import OfficialKinematics, READY_JOINTS


def main():
    path = ROOT / 'src/zekeep_grasp/config/sixaxis.urdf'
    model = pin.buildModelFromUrdf(str(path))
    kinematics = OfficialKinematics(path)
    assert model.nq == 8 and kinematics.model.nq == 6
    q = np.zeros(model.nq)
    q[:6] = READY_JOINTS
    data = model.createData()
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    for frame in ('official_tcp', 'grasp_tcp', 'gripper_base'):
        np.testing.assert_allclose(kinematics.fk_frame(READY_JOINTS, frame),
                                   data.oMf[model.getFrameId(frame)].homogeneous, atol=1e-12)
    target = READY_JOINTS.copy()
    target[0] += .02
    transform = kinematics.fk(target)
    pose = np.r_[transform[:3, 3], pin.rpy.matrixToRpy(transform[:3, :3])]
    solved = kinematics.solve_pose_sequence([pose], READY_JOINTS)[0]
    np.testing.assert_allclose(kinematics.fk(solved), transform, atol=2e-4)
    print('ROS/SDK FK equivalence and six-axis IK: OK')


if __name__ == '__main__':
    main()
