# Spec: Standalone ROS Hand-Guided Teaching and Replay

## Objective

Build a supervised terminal tool for the six-axis Zekeep that uses the existing
ROS hardware controller to:

- enter controller-owned gravity compensation for manual hand guiding;
- record discrete joint-space teaching points;
- record a continuously sampled joint-space path;
- replay discrete points at a uniform low speed, settling at every point;
- replay a continuous path smoothly after low-speed retiming; and
- perform one safe exit sequence: hold, safe-home, disable, then exit.

The tool is for attended development use. It has no safety-rated enabling device
and must not be represented as a safety-certified hand-guiding system.

## Confirmed Behavior

### Control ownership

- `ZekeepController` exclusively owns the motor connection.
- The teaching tool is a ROS client and never imports or creates
  `OfficialSdkRobot`.
- Gravity compensation uses `/zekeep/gravity_compensation/start` and
  `/zekeep/gravity_compensation/stop`.
- Replay uses `/zekeep/follow_joint_trajectory`.
- Safe exit uses `/zekeep/safe_home`, followed by `/zekeep/disable` only
  after safe-home succeeds.

### State machine

```text
HOLD
  -> GUIDING
  -> HOLD
  -> POINT_REPLAY -> HOLD
  -> PATH_REPLAY  -> HOLD
  -> SAFE_EXIT -> DISABLED
```

- Recording is allowed only in `GUIDING`.
- Replay is forbidden while gravity compensation is active. A replay command
  first stops gravity compensation and verifies stationary feedback.
- Point and path replay are mutually exclusive.
- Canceling or failing replay requests controller position hold.
- `Q`, window-independent EOF, and `Ctrl-C` all request the same safe-exit
  sequence.
- If stopping gravity compensation, safe-home, feedback validation, or settling
  fails, the tool does not call disable. It reports the failure and leaves the
  ROS controller responsible for holding the arm.

### Terminal controls

The first implementation uses immediate single-key input on an interactive TTY:

```text
T      enter gravity compensation / guiding
Space  save one discrete teaching point
C      start or stop continuous path recording
U      remove the most recent discrete point
1      replay discrete points, stopping at every point
2      replay the continuous path smoothly
S      save the current session
Q      safe-home, disable, and exit
```

Commands invalid in the current state are rejected without changing controller
state. Keyboard input is functional control only, not a safety-rated enabling
device.

## Recording Model

### Discrete points

- A point contains six finite mapped ROS joint positions and a capture timestamp.
- Saving is rejected unless all six joint velocities are below the configured
  stationary threshold.
- Consecutive points closer than the configured joint-distance threshold are
  rejected as duplicates.

### Continuous path

- Sampling consumes new `/zekeep/joint_states` sequence numbers at 20 Hz.
- Each sample stores six mapped joint positions and elapsed capture time.
- Samples with non-finite values or stale feedback are rejected.
- Adjacent samples with negligible joint displacement are removed before replay.
- Original timestamps are retained as metadata but do not control replay speed.

### Session file

Sessions are written as versioned JSON under `logs/teach_sessions/` by default.
An explicit `--output` path may override the destination.

```json
{
  "schema_version": 1,
  "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
  "created_at": "2026-08-20T10:00:00+08:00",
  "points": [{"captured_s": 1.2, "positions": [0, 0, 0, 0, 0, 0]}],
  "path": [{"captured_s": 2.0, "positions": [0, 0, 0, 0, 0, 0]}]
}
```

Loading rejects unknown schema versions, wrong joint names, non-monotonic path
timestamps, non-finite values, and samples that do not contain exactly six joint
positions.

## Replay

### Point replay

- Stop gravity compensation and verify stationary feedback before moving.
- Execute one direct joint target at a time.
- Compute each segment duration from the maximum joint displacement and the
  configured replay speed.
- Require the existing action server's final position and velocity settle checks
  to pass before sending the next point.
- Default maximum joint speed: `0.10 rad/s`.

### Continuous replay

- Stop gravity compensation and verify stationary feedback before moving.
- First move slowly from the current position to the first recorded path sample
  and settle.
- Retiming uses joint displacement and a configured `0.10 rad/s` maximum speed,
  with a minimum segment duration of `0.05 s`.
- A single `FollowJointTrajectory` goal carries the retimed path.
- Interior waypoint velocities are derived from neighboring retimed samples and
  clamped to the configured maximum; endpoint velocities are zero.
- The controller performs cubic Hermite interpolation and must settle at the
  final sample.

No original hand-guiding speed or pauses are reproduced.

## Safe Exit

Safe exit is one indivisible workflow:

1. Stop accepting record/replay commands.
2. Cancel an active replay and request position hold.
3. Stop gravity compensation, which locks the latest position into controller
   hold.
4. Verify fresh feedback and near-zero joint velocity.
5. Call `/zekeep/safe_home` and wait for successful completion.
6. Verify fresh feedback and near-zero joint velocity again.
7. Call `/zekeep/disable`.
8. Close the ROS client and exit successfully.

Steps 5-7 are not attempted if an earlier safety step fails. Disable is never
called after a failed safe-home response.

## Tech Stack

- Python 3.10
- ROS 2 Humble `rclpy`
- `sensor_msgs/JointState`
- `control_msgs/action/FollowJointTrajectory`
- `std_srvs/srv/Trigger`
- NumPy and Python standard-library JSON/TTY support
- Existing `RosRobotClient`, `RosCalibrationRobot`, and controller services

No new third-party dependency is introduced.

## Commands

Complete the workspace installation first. In every terminal, set `ZKEEP_WS`
to the actual workspace, for example:

```bash
export ZKEEP_WS="$HOME/Desktop/Zekeeparm"
```

Build after controller/client interface changes:

```bash
cd "$ZKEEP_WS"
source tools/activate_ros.sh
python -m colcon build --symlink-install --packages-select \
  zekeep_msgs zekeepcontroller zekeep_bringup zekeep_moveit_config zekeep_teach
```

Terminal 1: start the sole hardware driver. Motors are not automatically enabled.

```bash
cd "$ZKEEP_WS"
source tools/activate_ros.sh
ros2 launch zekeep_bringup driver.launch.py
```

Terminal 2: start MoveIt to validate safe-home paths. Do not start a second
MoveIt stack if one is already running.

```bash
cd "$ZKEEP_WS"
source install/setup.bash
ros2 launch zekeep_moveit_config hardware.launch.py arm_namespace:=zekeep
```

Terminal 3: confirm fresh status and joint feedback, then run teaching.

```bash
cd "$ZKEEP_WS"
source install/setup.bash
ros2 topic echo /zekeep/arm_status --once
ros2 topic echo /zekeep/joint_states --once
ros2 run zekeep_teach teach_replay
```

Entering gravity compensation can enable the motors; support the arm and keep
the operator present. Keep the driver and MoveIt running during teaching exit.
After teaching exits successfully, exit the driver and wait for successful
completion before closing MoveIt. A failed safe-home keeps the exit blocked;
resolve the fault and retry instead of force-killing the driver.

## Project Structure

```text
zekeep_teach/cli.py                    terminal interaction and lifecycle
zekeep_teach/ros_client.py             ROS actions/services and path execution
zekeep_teach/robot.py                  teaching-specific robot facade
zekeep_teach/session.py                pure recording, validation, JSON, retiming
config/default.yaml                      conservative teaching defaults
docs/ros-hand-guided-teaching.md         this specification
```

## Code Style

Use existing typed Python and NumPy conventions. Keep ROS message creation at the
client boundary and keep recording/retiming logic pure.

```python
def segment_duration_s(
    start: np.ndarray,
    target: np.ndarray,
    *,
    max_velocity_rad_s: float,
    minimum_s: float,
) -> float:
    delta = float(np.max(np.abs(np.asarray(target) - np.asarray(start))))
    return max(float(minimum_s), delta / float(max_velocity_rad_s))
```

## Boundaries

### Always

- Validate fresh finite six-joint feedback before recording or moving.
- Use the mapped ROS joint convention exactly once.
- Reject replay while recording.
- Keep speeds conservative and configurable.
- Hold on cancellation or execution failure.
- Keep disable conditional on successful safe-home and stationary feedback.

### Ask first

- Adding a hardware enabling device or external I/O.
- Raising replay speed above the initial supervised value.
- Adding blended discrete-point motion.
- Adding environment collision planning through MoveIt.
- Changing controller-level gravity compensation gains or dynamics parameters.

### Never

- Open the motor serial port from the teaching tool.
- Run `OfficialSdkRobot` beside `ZekeepController`.
- Treat keyboard input as a safety-rated enabling signal.
- Replay original hand-guiding timing.
- Disable directly from gravity compensation or after failed safe-home.
- Continue a replay after stale feedback, action failure, or user cancellation.

## Success Criteria

- The tool can record, save, load, and list valid discrete points and a continuous
  path without hardware.
- Discrete replay sends points sequentially and does not send point N+1 until
  point N reports settled success.
- Continuous replay produces monotonic low-speed trajectory timestamps, bounded
  waypoint velocities, and zero endpoint velocities.
- Gravity compensation and replay are mutually exclusive.
- `Q` and `Ctrl-C` use the same safe-exit sequence.
- A failed stop, stale feedback, failed safe-home, or non-stationary safe-home
  result prevents disable.
- A supervised hardware trial confirms: hand guiding works, stopping hand guiding
  holds position, one point replays and settles, a short path replays smoothly,
  and safe exit reaches home before disabling.

## Open Questions

None blocking. Initial rates and thresholds are conservative defaults and remain
configuration values for supervised tuning.
