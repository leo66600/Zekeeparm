import pytest

from lerobot_teleoperator_zhongling import ZhonglingLeader, ZhonglingLeaderConfig


def test_gripper_uses_calibrated_center_instead_of_startup_sample() -> None:
    config = ZhonglingLeaderConfig(
        port="/dev/fake",
        pwm_ranges=[(500, 1500, 2500)] * 6 + [(1300, 1500, 2500)],
        joint_ranges_deg=[(-90.0, 90.0)] * 6 + [(-77.34930234266115, 0.0)],
        fixed_pwm_center_ids=[7],
    )
    leader = ZhonglingLeader(config)

    leader._set_startup_pwm_centers([1490] + [1500] * 5 + [1514])

    assert leader._pwm_to_target(1490, 0) == 0.0
    assert leader._pwm_to_target(1500, 6) == 0.0
    assert leader._pwm_to_target(1300, 6) == pytest.approx(-77.34930234266115)
