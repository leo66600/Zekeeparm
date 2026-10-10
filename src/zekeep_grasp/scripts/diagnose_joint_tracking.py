"""Record SDK joint commands and feedback during one supervised motion.

Angles are model/URDF radians. Default is offline validation; --execute enables
hardware. This script provides no collision avoidance. CSV command values are
host-side controller targets, not motor acknowledgements; samples are sequential.
Default arm mode is POS_VEL; --control-mode mit enables comparison runs.
After motion or failure, hold the measured position until the operator exits.
--gravity-follow uses zero position gains and records hand-guided compensation
until the supported operator presses Enter; it never commands a trajectory.
"""
from __future__ import annotations

import argparse
import csv
import select
from datetime import datetime
from pathlib import Path
import sys
import threading
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from utils.native_runtime import preload_environment_libstdcpp

preload_environment_libstdcpp()

import numpy as np
import pinocchio as pin
from drivers.robot.grasp_driver import ArmJointMapping
from drivers.robot.official_sdk_robot import OfficialSdkRobot
from scripts.diagnose_pose_error import plan_target
from utils.camera_utils import load_config


def joint_target(robot, values, start):
    target = np.asarray(values, dtype=float).reshape(6)
    limits = robot.kinematics.joint_limits
    if (not np.all(np.isfinite(target)) or np.any(target < limits[:, 0])
            or np.any(target > limits[:, 1])):
        raise ValueError("joint target is non-finite or outside model limits")
    if np.max(np.abs(target - start)) > robot.cfg["official_sdk"]["max_segment_joint_delta_rad"]:
        raise ValueError("joint segment delta exceeds configured limit")
    return target


def print_direction_plan(robot, start, target):
    mapping = ArmJointMapping.from_config(
        robot.cfg["robot"]["joint_mapping"], [f"joint{i}" for i in range(1, 7)]
    )
    motor_start = mapping.model_to_motor_positions(start)
    motor_target = mapping.model_to_motor_positions(target)
    print("[Start model rad]", start, flush=True)
    print("[Target model rad]", target, flush=True)
    for i in range(6):
        print(f"[J{i + 1}] direction={mapping.directions[i]:+.0f} "
              f"model_delta={target[i] - start[i]:+.6f}rad "
              f"mapped_motor_start={motor_start[i]:+.6f}rad "
              f"mapped_motor_target={motor_target[i]:+.6f}rad "
              f"motor_delta={motor_target[i] - motor_start[i]:+.6f}rad", flush=True)
    print("[Mapping] Motor values are mapped host targets, not motor acknowledgements. "
          "Observe rotation along the tested joint axis.", flush=True)


def record_sample(robot, expected, writer, stream, started):
    command = np.asarray(robot._controller._q_target[:6], dtype=float).copy()
    actual, velocity, _ = robot._arm.get_state(request_feedback=True)
    actual = np.asarray(actual[:6], dtype=float)
    velocity = np.asarray(velocity[:6], dtype=float)
    if any(v.shape != (6,) or not np.all(np.isfinite(v))
           for v in (command, actual, velocity)):
        raise RuntimeError("invalid command or joint feedback")
    elapsed = time.monotonic() - started
    for i in range(6):
        writer.writerow((elapsed, i + 1, expected[i], command[i], actual[i],
                         actual[i] - command[i], command[i] - expected[i],
                         actual[i] - expected[i], velocity[i]))
    stream.flush()
    return command, actual, velocity


def execute_and_record(robot, expected, pose, duration, interval, stream):
    writer = csv.writer(stream)
    writer.writerow(("elapsed_s", "joint", "expected_rad", "command_rad", "actual_rad",
                     "tracking_error_rad", "target_difference_rad",
                     "acceptance_error_rad", "velocity_rad_s"))
    started = time.monotonic()
    stop = threading.Event()
    cancel = threading.Event()
    sampling_errors = []

    def sample():
        try:
            while not stop.wait(interval):
                record_sample(robot, expected, writer, stream, started)
        except Exception as error:
            sampling_errors.append(error)
            cancel.set()

    record_sample(robot, expected, writer, stream, started)
    sampler = threading.Thread(target=sample, name="joint-feedback-recorder")
    sampler.start()
    try:
        if pose is None:
            robot.move_joints_and_wait(expected, duration, cancel_event=cancel)
        else:
            robot.move_to_traj_and_wait(pose, expected, duration, cancel_event=cancel)
    finally:
        # Capture the endpoint even on settle failure, BEFORE hold_current changes it.
        stop.set()
        sampler.join()
        command, actual, velocity = record_sample(robot, expected, writer, stream, started)
        print("Joint Expected(rad) Command(rad) Actual(rad) Tracking(rad) TargetDiff(rad) Velocity(rad/s)", flush=True)
        for i in range(6):
            print(f"J{i + 1} {expected[i]: .6f} {command[i]: .6f} {actual[i]: .6f} "
                  f"{actual[i] - command[i]: .6f} {command[i] - expected[i]: .6f} "
                  f"{velocity[i]: .6f}", flush=True)
        if sampling_errors:
            raise RuntimeError(f"feedback recording failed: {sampling_errors[0]}") from sampling_errors[0]


def record_gravity_follow(robot, interval, stream):
    """Record the model/feedback torques without a fixed-pose restoring spring."""
    writer = csv.writer(stream)
    writer.writerow(('elapsed_s', 'joint', 'actual_rad', 'velocity_rad_s',
                     'model_gravity_ff_nm', 'feedback_torque_nm'))
    model = robot._controller._model
    data = model.createData()
    started = time.monotonic()
    reported = 0.0
    print('重力补偿已开启，位置增益为零。保持支撑，仅小范围拖动；'
          '托稳后按 Enter 结束采集并失能。日志持续记录。', flush=True)
    while True:
        if select.select([sys.stdin], [], [], interval)[0]:
            line = sys.stdin.readline()
            if line == '':
                raise RuntimeError('operator terminal closed')
            if not line.strip():
                return
            continue
        if not robot.control_loop_active:
            raise RuntimeError('SDK control loop stopped')
        actual, velocity, effort = robot._arm.get_state(request_feedback=True)
        vectors = [np.asarray(value[:6], dtype=float) for value in (actual, velocity, effort)]
        if any(value.shape != (6,) or not np.all(np.isfinite(value)) for value in vectors):
            raise RuntimeError('invalid gravity-test feedback')
        actual, velocity, effort = vectors
        limits = robot.kinematics.joint_limits
        if np.any(actual < limits[:, 0] - .001) or np.any(actual > limits[:, 1] + .001):
            raise RuntimeError('gravity-test feedback outside joint limits')
        q = np.zeros(model.nq)
        q[:6] = actual
        torque = pin.computeGeneralizedGravity(model, data, q)[:6]
        torque *= np.where(torque > 0, robot.gravity_tau_scale_positive, robot.gravity_tau_scale)
        elapsed = time.monotonic() - started
        for index in range(6):
            writer.writerow((elapsed, index + 1, actual[index], velocity[index], torque[index], effort[index]))
        stream.flush()
        if elapsed - reported >= .5:
            print(f'J2={np.degrees(actual[1]):.2f}° ({actual[1]:.4f}rad), '
                  f'velocity={velocity[1]:+.4f}rad/s, model gravity={torque[1]:+.3f}N·m', flush=True)
            reported = elapsed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config/default.yaml"))
    parser.add_argument("--control-mode", choices=("posvel", "mit"), default="posvel",
                        help="arm mode for this test only (default: posvel)")
    parser.add_argument("--position-tolerance", type=float, default=0.005,
                        help="settled joint error limit (rad; default: 0.005)")
    target_args = parser.add_mutually_exclusive_group(required=True)
    target_args.add_argument('--gravity-follow', action='store_true',
                             help='MIT gravity test with zero position gains; record until supported exit')
    target_args.add_argument("--joints", type=float, nargs=6, help="J1-J6 absolute model angles (rad)")
    target_args.add_argument("--pose", type=float, nargs=6, help="official_tcp X Y Z ROLL PITCH YAW (m/rad)")
    target_args.add_argument("--joint-delta", type=float, nargs=6,
                             help="J1-J6 increments from measured start (rad)")
    parser.add_argument("--return-to-start", action="store_true",
                        help="after successful joint motion and operator Enter, return to measured start")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--interval", type=float, default=0.05, help="sampling interval (s)")
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "data" /
                        f"joint_tracking_{datetime.now():%Y%m%d_%H%M%S_%f}.csv")
    parser.add_argument("--execute", action="store_true", help="enable and move physical robot")
    args = parser.parse_args(argv)
    if args.gravity_follow and (args.control_mode != 'mit' or args.return_to_start):
        parser.error('--gravity-follow requires --control-mode mit and cannot return to start')
    if args.return_to_start and args.pose is not None:
        parser.error("--return-to-start supports joint tests only")
    for name in ("duration", "interval", "position_tolerance"):
        value = getattr(args, name)
        if not np.isfinite(value) or value <= 0:
            parser.error(f"--{name} must be finite and positive")
    if args.execute and not sys.stdin.isatty():
        parser.error("--execute requires an interactive terminal for supported shutdown")
    cfg = load_config(args.config)
    cfg["official_sdk"]["arm_control_mode"] = args.control_mode
    if args.gravity_follow:
        cfg['official_sdk']['mit_kp'] = [0.0] * 6
    cfg.setdefault("safety", {})["motion_position_tolerance_rad"] = args.position_tolerance
    robot = OfficialSdkRobot(cfg)
    pose = None if args.pose is None else np.asarray(args.pose, dtype=float)

    def plan(start):
        if args.gravity_follow:
            return joint_target(robot, start, start)
        values = start + np.asarray(args.joint_delta) if args.joint_delta is not None else args.joints
        return (joint_target(robot, values, start) if pose is None
                else plan_target(robot, pose, start))

    supported_exit = False
    try:
        # Offline validation does not open the serial port or enable motors.
        target = plan(np.asarray(cfg["official_sdk"]["ready_joints"], dtype=float))
        print("[Target rad]", target, flush=True)
        if args.control_mode == "mit":
            print(f"[MIT] kp={robot.mit_kp}, kd={robot.mit_kd}", flush=True)
        else:
            print("[POS_VEL] SDK loads motor position/velocity loop gains and speed limits "
                  "from the hardware YAML; MIT gravity feedforward is inactive.", flush=True)
        print(f"[Settle] position_tolerance={args.position_tolerance:.6f}rad", flush=True)
        if not args.execute:
            print_direction_plan(robot, np.asarray(cfg["official_sdk"]["ready_joints"]), target)
            print("[Offline] checked using ready-joint seed; add --execute for live motion.")
            return 0
        args.output.parent.mkdir(parents=True, exist_ok=True)
        # Refuse to overwrite earlier measurements; check storage before enabling.
        with args.output.open("x", newline="", encoding="utf-8") as stream:
            print(f"[CSV] {args.output.resolve()}", flush=True)
            robot.connect()
            if args.gravity_follow:
                record_gravity_follow(robot, args.interval, stream)
                supported_exit = True
                print('[DONE] gravity recording complete; no stability verdict inferred', flush=True)
                return 0
            if args.control_mode == "posvel":
                print(f"[POS_VEL] vlim={robot._controller._arm_group._pv_vlim} rad/s", flush=True)
            start = robot.current_joints().copy()
            target = plan(start)
            print_direction_plan(robot, start, target)
            execute_and_record(robot, target, pose, args.duration, args.interval, stream)
        if args.return_to_start:
            input("去程完成。记录被测关节的观察方向和转向；确认返程路径可用后按 Enter 返回起点：")
            return_path = args.output.with_name(args.output.stem + "_return.csv")
            with return_path.open("x", newline="", encoding="utf-8") as stream:
                current = robot.current_joints()
                joint_target(robot, start, current)
                print(f"[Return CSV] {return_path.resolve()}", flush=True)
                print_direction_plan(robot, current, start)
                execute_and_record(robot, start, None, args.duration, args.interval, stream)
        print("[PASS] existing position/velocity settle checks passed", flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print(f"[ERROR] {type(error).__name__}: {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        try:
            if robot.control_loop_active:
                if args.gravity_follow:
                    if not supported_exit:
                        input('重力测试已中断，补偿可能仍开启。托住机械臂后按 Enter 失能并退出：')
                else:
                    robot.hold_current()
                    input("已保持当前位置。托住机械臂后，按 Enter 失能并退出：")
        finally:
            robot.close()


if __name__ == "__main__":
    raise SystemExit(main())
