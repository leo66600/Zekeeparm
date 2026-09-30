#!/usr/bin/env python

"""Low-latency Zhongling leader to zk_xm115 follower teleoperation."""

import logging
import time
from dataclasses import asdict, dataclass
from pprint import pformat

from lerobot.configs import parser
from lerobot.processor import (
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
from lerobot.robots import Robot, RobotConfig, make_robot_from_config
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig, make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging

# Importing the package registers zhongling_leader before draccus parses YAML.
from . import ZhonglingLeader, ZhonglingLeaderConfig  # noqa: F401


@dataclass
class TeleoperateConfig:
    teleop: TeleoperatorConfig
    robot: RobotConfig
    fps: int = 50
    teleop_time_s: float | None = None
    display_data: bool = False
    read_robot_observation: bool = False
    loop_status_interval_s: float = 1.0


def teleop_loop(
    teleop: Teleoperator,
    robot: Robot,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ],
    robot_action_processor: RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ],
    duration: float | None = None,
    read_robot_observation: bool = False,
    loop_status_interval_s: float = 1.0,
) -> None:
    start = time.perf_counter()
    status_start = start
    status_cycles = 0
    status_work_s = 0.0

    while True:
        loop_start = time.perf_counter()
        observation = robot.get_observation() if read_robot_observation else {}
        raw_action = teleop.get_action()
        teleop_action = teleop_action_processor((raw_action, observation))
        robot_action = robot_action_processor((teleop_action, observation))
        robot.send_action(robot_action)

        work_s = time.perf_counter() - loop_start
        precise_sleep(max(1 / fps - work_s, 0.0))
        status_cycles += 1
        status_work_s += work_s
        now = time.perf_counter()
        elapsed = now - status_start
        if loop_status_interval_s > 0 and elapsed >= loop_status_interval_s:
            print(
                f"Teleop: {status_cycles / elapsed:.1f} Hz | "
                f"avg work {status_work_s * 1e3 / status_cycles:.2f}ms | target {fps} Hz"
            )
            status_start = now
            status_cycles = 0
            status_work_s = 0.0
        if duration is not None and now - start >= duration:
            return


@parser.wrap()
def teleoperate(cfg: TeleoperateConfig) -> None:
    init_logging()
    logging.info(pformat(asdict(cfg)))
    if cfg.fps <= 0:
        raise ValueError("fps must be positive")
    if cfg.loop_status_interval_s < 0:
        raise ValueError("loop_status_interval_s cannot be negative")
    if cfg.display_data:
        raise ValueError("This focused teleoperation package does not include camera display")

    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)
    teleop_action_processor, robot_action_processor, _ = make_default_processors()

    try:
        teleop.connect()
        robot.connect()
        teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=cfg.fps,
            duration=cfg.teleop_time_s,
            read_robot_observation=cfg.read_robot_observation,
            loop_status_interval_s=cfg.loop_status_interval_s,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
        )
    except KeyboardInterrupt:
        pass
    finally:
        # Disable/disconnect the powered follower before closing the passive leader.
        if robot.is_connected:
            robot.disconnect()
        if teleop.is_connected:
            teleop.disconnect()


def main() -> None:
    register_third_party_plugins()
    teleoperate()


if __name__ == "__main__":
    main()
