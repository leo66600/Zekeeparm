"""
Eye-in-hand calibration data collection and solving (Gemini2 + Zekeep).

Modes:
  Auto mode (default): uses the ROS controller and mapped /joint_states while
                       traversing 50 preset poses, captures samples when the
                       target is detected, and skips timed-out poses.
  Manual mode (--manual): asks the ROS controller for gravity compensation so
                          the user can move the arm by hand, then press Enter.

Setup:
  The camera is mounted on the end effector.
  The ArUco marker is fixed on the work surface.

Usage:
    python scripts/collect_handeye_eih.py           # auto mode
    python scripts/collect_handeye_eih.py --manual  # manual gravity mode
"""

import os
import sys
import threading
import argparse
import queue
import time
import traceback
import cv2
import numpy as np
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
WORKSPACE_ROOT = PROJECT_ROOT.parents[1]
SDK_ROOT = WORKSPACE_ROOT / "third_party" / "reBotArm_control_py"
CONTROLLER_ROOT = WORKSPACE_ROOT / "src" / "zekeepcontroller"

for path in (PROJECT_ROOT, SDK_ROOT, CONTROLLER_ROOT):
    path_str = str(path)
    if path.exists() and path_str not in sys.path:
        sys.path.insert(0, path_str)

os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype")

from drivers.camera import make_camera
from drivers.robot.ros_calibration_robot import (
    RosCalibrationRobot,
    make_ros_calibration_robot,
)
from calibration.auto_poses import load_auto_calibration_poses
from calibration.hand_eye import CalibMode, HandEyeCalibrator
from utils.camera_utils import load_config
from utils.robot_model_config import resolve_robot_model_config
from utils.transforms import rotation_matrix_to_euler_zyx


AUTO_MOVE_DURATION_S = 8.0
AUTO_SETTLE_EXTRA_S = 0.6
AUTO_MARKER_TIMEOUT_S = 2.5
AUTO_MARKER_STABLE_FRAMES = 4
MIN_CALIB_SAMPLES = 5


def _cleanup_and_save(cleanup_actions, save_action) -> None:
    """Run every cleanup action, then save even if cleanup reports an error."""
    for label, action in cleanup_actions:
        try:
            action()
        except Exception as exc:
            print(f"[Cleanup] {label} failed: {exc}")
    save_action()


def _target_quality_error(
    pose,
    *,
    target_type: str,
    max_reprojection_rmse_px: float,
) -> str | None:
    if str(target_type).lower() != "charuco":
        return None
    rmse = getattr(pose, "reprojection_rmse_px", None)
    if rmse is None or not np.isfinite(float(rmse)):
        return "ChArUco reprojection RMSE is unavailable"
    threshold = float(max_reprojection_rmse_px)
    if float(rmse) > threshold:
        return (
            f"ChArUco reprojection RMSE {float(rmse):.3f}px exceeds "
            f"{threshold:.3f}px"
        )
    return None


def _result_quality_error(
    result,
    *,
    max_translation_rmse_m: float,
    max_rotation_rmse_deg: float,
) -> str | None:
    errors = []
    if float(result.translation_rmse_m) > float(max_translation_rmse_m):
        errors.append(
            "translation closure RMSE "
            f"{float(result.translation_rmse_m) * 1000.0:.1f}mm exceeds "
            f"{float(max_translation_rmse_m) * 1000.0:.1f}mm"
        )
    if float(result.rotation_rmse_deg) > float(max_rotation_rmse_deg):
        errors.append(
            "rotation closure RMSE "
            f"{float(result.rotation_rmse_deg):.2f}deg exceeds "
            f"{float(max_rotation_rmse_deg):.2f}deg"
        )
    return "; ".join(errors) if errors else None


def make_input_thread(line_queue: queue.Queue) -> threading.Thread:
    def _loop():
        while True:
            try:
                line_queue.put(input())
            except EOFError:
                line_queue.put(None)
                break
            except KeyboardInterrupt:
                line_queue.put(None)
                break
    t = threading.Thread(target=_loop, daemon=True)
    t.start()
    return t


# ==========================================
# Main flow.
# ==========================================
def main():
    parser = argparse.ArgumentParser(description="Eye-in-hand calibration data collection")
    parser.add_argument("--manual", action="store_true",
                        help="manual mode: gravity compensation; move the arm by hand and press Enter to capture")
    args = parser.parse_args()

    root = Path(__file__).resolve().parent.parent
    cfg  = load_config(root / "config" / "default.yaml")

    cam_type   = cfg["camera"]["type"]
    calib_dir  = root / "config" / "calibration" / cam_type
    calibration_cfg = cfg["calibration"]
    target_type = str(calibration_cfg.get("target_type", "aruco")).lower()
    aruco_cfg  = calibration_cfg["aruco"]
    charuco_cfg = calibration_cfg.get("charuco", {})
    max_reprojection_rmse_px = float(
        charuco_cfg.get("max_reprojection_rmse_px", 1.0)
    )
    quality_cfg = calibration_cfg.get("quality", {})
    max_translation_rmse_m = float(
        quality_cfg.get("max_translation_rmse_m", 0.010)
    )
    max_rotation_rmse_deg = float(
        quality_cfg.get("max_rotation_rmse_deg", 3.0)
    )
    he_method  = cfg["calibration"].get("hand_eye_method", "TSAI")
    save_path  = calib_dir / "hand_eye.npz"
    robot_cfg = cfg.get("robot", {})
    robot_urdf_path, robot_end_frame = resolve_robot_model_config(
        root,
        robot_cfg,
    )
    auto_poses = (
        ()
        if args.manual
        else load_auto_calibration_poses(
            calib_dir / "auto_poses.yaml",
            expected_reference_frame=robot_end_frame,
        )
    )

    # Camera.
    cam = make_camera(cfg)
    if target_type == "charuco":
        cam.setup_charuco(
            squares_x=charuco_cfg["squares_x"],
            squares_y=charuco_cfg["squares_y"],
            square_length_m=charuco_cfg["square_length_m"],
            marker_length_m=charuco_cfg["marker_length_m"],
            dict_id=charuco_cfg.get("dict_id", 0),
            min_corners=charuco_cfg.get("min_corners", 6),
        )
        target_label = (
            f"ChArUco {charuco_cfg['squares_x']}x{charuco_cfg['squares_y']} "
            f"square={charuco_cfg['square_length_m']*1000:.2f}mm "
            f"marker={charuco_cfg['marker_length_m']*1000:.2f}mm"
        )
    elif target_type == "aruco":
        cam.setup_aruco(
            marker_length_m=aruco_cfg["marker_length_m"],
            dict_id=aruco_cfg.get("dict_id", 0),
            target_marker_id=aruco_cfg.get("target_marker_id"),
        )
        target_label = f"ArUco size={aruco_cfg['marker_length_m']*100:.0f}cm"
    else:
        raise ValueError(f"unsupported calibration.target_type: {target_type}")

    # Calibrator.
    calibrator = HandEyeCalibrator(
        CalibMode.EYE_IN_HAND,
        method=he_method,
        reference_frame=robot_end_frame,
        urdf_path=str(robot_urdf_path),
    )

    # Robot.
    mode_str = (
        "manual (gravity compensation)"
        if args.manual
        else f"auto ({len(auto_poses)} frame-validated poses)"
    )
    manual_robot: RosCalibrationRobot | None = None
    auto_robot: RosCalibrationRobot | None = None
    pose_source: RosCalibrationRobot | None = None
    auto = {
        "enabled": not args.manual,
        "idx": 0,
        "pose_idx": None,
        "phase": "idle",
        "settle_until": 0.0,
        "timeout_at": 0.0,
        "stable_frames": 0,
        "status": "waiting to start",
        "finished": False,
    }
    result_saved = False

    print("\n=== Eye-in-Hand Calibration ===")
    print(f"Camera: {cam_type}  |  Mode: {mode_str}  |  Solver: {he_method}")
    print(f"Target: {target_label}  |  Output: {save_path}")
    print()

    # Open the camera first so the arm is not enabled if camera setup fails.
    try:
        cam.open()
        print("Warming up camera...", end="", flush=True)
        cam.warm_up(20)
        print(" ready\n")
    except Exception as e:
        try:
            cam.close()
        except Exception:
            pass
        print(f"[Camera] Initialization failed: {e}")
        sys.exit(1)

    try:
        if args.manual:
            manual_robot = make_ros_calibration_robot(robot_cfg)
            manual_robot.start_manual()
            pose_source = manual_robot
            print("[GravityComp] ROS controller gravity compensation is active.")
            print("[Robot] Manual mode ready. Move the arm by hand, then press Enter to capture.")
        else:
            auto_robot = make_ros_calibration_robot(robot_cfg)
            auto_robot.start()
            auto_poses.require_validated_start(auto_robot.get_joint_positions())
            pose_source = auto_robot
            print(
                "[Robot] Auto mode ready through ROS controller. "
                "FK uses the calibrated joint mapping from "
                f"{robot_cfg.get('joint_state_topic', '/zekeep/joint_states')}. "
                f"{len(auto_poses)} frame-validated poses will be traversed."
            )
    except Exception as e:
        if auto_robot is not None:
            try:
                auto_robot.close()
            except Exception:
                pass
        try:
            cam.close()
        except Exception:
            pass
        print(f"[Robot] Connection failed: {e}")
        sys.exit(1)

    if args.manual:
        print("[Controls] Enter=capture  c/q=finish and solve  pos=print current TCP pose")
    else:
        print("[Controls] Auto traversal and capture  c/q=stop and solve  pos=print current TCP pose")
    print()

    latest_pose = None
    line_queue: queue.Queue | None = None
    if sys.stdin.isatty():
        line_queue = queue.Queue()
        make_input_thread(line_queue)
    else:
        print("[Hint] Non-interactive terminal detected; terminal commands are disabled")

    def _print_fk() -> None:
        try:
            if pose_source is None:
                raise RuntimeError("robot pose source is not initialized")
            T = pose_source.get_tcp_pose()
            t = T[:3, 3]
            R = T[:3, :3]
            _r, _p, _y = rotation_matrix_to_euler_zyx(R)
            print(f"  FK: x={t[0]:+.3f} y={t[1]:+.3f} z={t[2]:+.3f} m"
                  f"  rpy=[{_r:+.2f} {_p:+.2f} {_y:+.2f}] rad")
        except Exception as e:
            print(f"  [Error] {e}")

    def capture_sample(cur, source: str) -> bool:
        if cur is None:
            print("  [Skip] Marker is not visible; adjust the pose and try again")
            return False
        quality_error = _target_quality_error(
            cur,
            target_type=target_type,
            max_reprojection_rmse_px=max_reprojection_rmse_px,
        )
        if quality_error is not None:
            print(f"  [Skip] {quality_error}; adjust the view and try again")
            return False

        print(f"\n[Sample {calibrator.n_samples + 1}] {source}")
        print(f"  Target: x={cur.T_marker2cam[0,3]:.3f} "
              f"y={cur.T_marker2cam[1,3]:.3f} "
              f"z={cur.T_marker2cam[2,3]:.3f} m")
        if cur.reprojection_rmse_px is not None:
            print(
                f"  ChArUco: corners={cur.corner_count} "
                f"reprojection_rmse={cur.reprojection_rmse_px:.3f}px "
                f"max={cur.reprojection_max_px:.3f}px"
            )
        try:
            if pose_source is None:
                raise RuntimeError("robot pose source is not initialized")
            T_g2b = pose_source.get_tcp_pose()
            t = T_g2b[:3, 3]
            print(f"  End effector (FK): x={t[0]:.4f} y={t[1]:.4f} z={t[2]:.4f} m")
            calibrator.add_sample(
                T_g2b,
                cur.T_marker2cam,
                reprojection_rmse_px=(
                    cur.reprojection_rmse_px
                    if cur.reprojection_rmse_px is not None
                    else float("nan")
                ),
                corner_count=(
                    cur.corner_count if cur.corner_count is not None else -1
                ),
            )
            print(f"  [OK] Recorded, total samples: {calibrator.n_samples}"
                  + ("  will solve automatically on finish" if calibrator.n_samples >= 15 else ""))
            return True
        except Exception as e:
            print(f"  [Error] Failed to read TCP pose: {e}")
            return False

    def compute_and_save(reason: str) -> bool:
        nonlocal result_saved
        print(f"\n[Finish] {reason}")
        if calibrator.n_samples < MIN_CALIB_SAMPLES:
            print(f"[Result] Not enough samples ({calibrator.n_samples} < {MIN_CALIB_SAMPLES}); calibration was not solved")
            if save_path.exists():
                print("[Result] Existing hand_eye.npz was not updated")
            return False

        print(f"[Result] Solving with {calibrator.n_samples} samples...")
        try:
            result = calibrator.calibrate(min_samples=MIN_CALIB_SAMPLES)
            t = result.T_result[:3, 3]
            R = result.T_result[:3, :3]
            print(f"[Result] T_cam2gripper translation: x={t[0]:.4f} y={t[1]:.4f} z={t[2]:.4f} m")
            print(f"[Result] Rotation matrix:\n{R}")
            print(
                "[Result] Closure RMSE: "
                f"translation={result.translation_rmse_m * 1000.0:.2f}mm "
                f"rotation={result.rotation_rmse_deg:.3f}deg"
            )
            quality_error = _result_quality_error(
                result,
                max_translation_rmse_m=max_translation_rmse_m,
                max_rotation_rmse_deg=max_rotation_rmse_deg,
            )
            if quality_error is not None:
                rejected_path = save_path.with_name("hand_eye_rejected.npz")
                HandEyeCalibrator.save(result, rejected_path)
                print(f"[Result] [Rejected] {quality_error}")
                print(
                    "[Result] Raw samples and diagnostics saved to "
                    f"{rejected_path}"
                )
                print("[Result] Existing hand_eye.npz was not updated")
                return False

            HandEyeCalibrator.save(result, save_path)
            print(f"[Result] [OK] Saved to {save_path}")
            if calibrator.n_samples < 15:
                print("[Result] Tip: fewer than 15 samples; collect more samples for better accuracy")
            result_saved = True
            return True
        except Exception as e:
            print(f"[Result] [Error] Solve failed: {e}")
            return False

    def start_next_auto_pose() -> bool:
        if not auto["enabled"] or auto_robot is None:
            return False

        total = len(auto_poses)
        while auto["idx"] < total:
            idx = auto["idx"]
            x, y, z, roll, pitch, yaw = auto_poses[idx]
            print(f"\n[Auto] Pose {idx+1}/{total}: "
                  f"pos=({x:.2f},{y:.2f},{z:.2f}) rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})")
            ok = auto_robot.move_to_pose(
                (x, y, z, roll, pitch, yaw),
                duration=AUTO_MOVE_DURATION_S,
            )
            if ok:
                now = time.monotonic()
                auto["pose_idx"] = idx
                auto["phase"] = "settling"
                # The ROS action returns after trajectory completion. Keep only
                # the short camera/feedback settling margin here.
                auto["settle_until"] = now + AUTO_SETTLE_EXTRA_S
                auto["timeout_at"] = auto["settle_until"] + AUTO_MARKER_TIMEOUT_S
                auto["stable_frames"] = 0
                auto["status"] = f"pose {idx+1}/{total} moving"
                return False

            print(f"[Auto] Pose {idx+1}/{total} has no IK solution, skipping")
            auto["idx"] += 1

        auto["phase"] = "done"
        auto["finished"] = True
        auto["status"] = "all poses completed"
        print("\n[Auto] All preset poses completed")
        return True

    def tick_auto(cur) -> bool:
        if not auto["enabled"] or auto["finished"]:
            return auto["finished"]

        if auto["phase"] == "idle":
            return start_next_auto_pose()

        pose_idx = auto["pose_idx"]
        total = len(auto_poses)
        now = time.monotonic()

        if auto["phase"] == "settling":
            remain = auto["settle_until"] - now
            if remain > 0.0:
                auto["stable_frames"] = 0
                auto["status"] = f"pose {pose_idx+1}/{total} moving/settling {remain:.1f}s"
                return False
            auto["phase"] = "searching"

        if cur is not None:
            auto["stable_frames"] += 1
            remain = max(0.0, auto["timeout_at"] - now)
            auto["status"] = (
                f"pose {pose_idx+1}/{total} marker stable "
                f"{auto['stable_frames']}/{AUTO_MARKER_STABLE_FRAMES}  remaining {remain:.1f}s"
            )
            if auto["stable_frames"] >= AUTO_MARKER_STABLE_FRAMES:
                if capture_sample(cur, f"auto pose {pose_idx+1}/{total}"):
                    auto["idx"] += 1
                    auto["phase"] = "idle"
                    auto["stable_frames"] = 0
                    return start_next_auto_pose()
                auto["stable_frames"] = 0
        else:
            auto["stable_frames"] = 0
            remain = max(0.0, auto["timeout_at"] - now)
            auto["status"] = (
                f"pose {pose_idx+1}/{total} waiting for target {remain:.1f}s"
            )

        if now >= auto["timeout_at"]:
            print(
                f"[Auto] Pose {pose_idx+1}/{total} timed out without "
                "calibration target, skipping"
            )
            auto["idx"] += 1
            auto["phase"] = "idle"
            auto["stable_frames"] = 0
            return start_next_auto_pose()

        return False

    def handle_line(raw: str) -> bool:
        if raw is None:
            print("\n[Interrupt] Terminal input closed; stopping and trying to solve")
            return True

        line = raw.strip().lower()

        if line in {"q", "c"}:
            return True

        if line == "pos":
            _print_fk()
            return False

        if args.manual and line == "":
            capture_sample(latest_pose, "manual capture")
            return False

        if line:
            if args.manual:
                print("  Manual commands: Enter=capture  c/q=finish and solve  pos=print current TCP pose")
            else:
                print("  Auto commands: c/q=finish and solve  pos=print current TCP pose")
        return False

    # Main loop.
    WIN = "Eye-in-Hand Calibration  (operate in terminal)"
    finish_reason = "normal finish"
    try:
        cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)

        while True:
            bgr, _ = cam.get_frame()
            if bgr is not None:
                pose = cam.detect_calibration_target(bgr)
                latest_pose = pose
                if tick_auto(pose):
                    finish_reason = "auto traversal completed"
                vis  = cam.draw_calibration_target(bgr)
                n    = calibrator.n_samples

                def osd(text, y, color=(220, 220, 220)):
                    cv2.putText(vis, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, color, 1, cv2.LINE_AA)

                if pose:
                    detected = "ChArUco" if target_type == "charuco" else f"ID={pose.id}"
                    if args.manual:
                        osd(f"[{detected}] z={pose.T_marker2cam[2,3]:.3f}m  "
                            f"samples:{n}  Enter=capture  c/q=finish",
                            28, (80, 220, 80))
                    else:
                        osd(f"[{detected}] z={pose.T_marker2cam[2,3]:.3f}m  samples:{n}",
                            28, (80, 220, 80))
                else:
                    if args.manual:
                        osd(f"No marker  samples:{n}  move arm to see marker",
                            28, (80, 80, 220))
                    else:
                        osd(f"No marker  samples:{n}",
                            28, (80, 80, 220))

                if args.manual:
                    osd("MANUAL: Enter=capture  c/q=finish  pos=print fk", 50, (180, 180, 60))
                else:
                    osd(f"AUTO: {auto['status']}", 50, (180, 180, 60))

                filled = min(n, 15) * (400 // 15)
                cv2.rectangle(vis, (10, 70), (10 + filled, 82), (0, 200, 100), -1)
                cv2.rectangle(vis, (10, 70), (410, 82), (160, 160, 160), 1)
                cv2.putText(vis, f"{n}/15", (10, 95),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

                mode_label = "MANUAL(GravComp)" if args.manual else "AUTO"
                cv2.putText(vis, mode_label, (vis.shape[1] - 200, vis.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 60), 1)

                cv2.imshow(WIN, vis)

            if cv2.waitKey(30) & 0xFF in [ord('q'), ord('Q'), 27]:
                finish_reason = "window exit"
                break
            if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                finish_reason = "window closed"
                break

            try:
                if line_queue is not None and handle_line(line_queue.get_nowait()):
                    finish_reason = "user interrupted"
                    break
            except queue.Empty:
                pass

            if auto["finished"]:
                break

    except KeyboardInterrupt:
        finish_reason = "Ctrl+C interrupt"
        print("\n[Ctrl+C] Stopping and trying to solve")

    except Exception as e:
        finish_reason = "runtime error"
        print(f"\n[Runtime] Error in calibration loop: {e}")
        traceback.print_exc()

    finally:
        cleanup_actions = [
            ("OpenCV windows", cv2.destroyAllWindows),
            ("camera", cam.close),
        ]
        if manual_robot is not None:
            cleanup_actions.append(
                ("manual gravity compensation", manual_robot.close_manual)
            )
        elif auto_robot is not None:
            cleanup_actions.append(
                ("automatic ROS calibration client", auto_robot.close)
            )
        _cleanup_and_save(
            cleanup_actions,
            lambda: compute_and_save(finish_reason),
        )

    print(f"\nDone, total samples: {calibrator.n_samples}.")
    if calibrator.n_samples > 0 and not result_saved:
        print("Tip: hand_eye.npz was not generated; collect more samples and try again.")


if __name__ == "__main__":
    main()
    os._exit(0)
