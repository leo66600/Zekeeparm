"""Supervised web session coordinator; hardware remains controller-owned."""
from __future__ import annotations

import json
import os
from pathlib import Path
import re
import tempfile
import threading
import time

import numpy as np

from .robot import TeachState
from .session import load_teaching_session


def validate_session(session):
    if len(session.points) + len(session.path) > 10000:
        raise ValueError('session exceeds 10000 samples')
    for sample in (*session.points, *session.path):
        q = sample.positions
        if np.any(q < [-2.58, 0, -0.01, -1.57, -1.57, -1.57]) or np.any(q > [2.58, 3.7, 3.7, 1.57, 1.57, 1.57]):
            raise ValueError('session joint limits exceeded')


class WebSession:
    def __init__(self, robot_factory, directory, *, authorized=False, clock=time.monotonic, acquire=None, release=None):
        self.acquire = acquire or (lambda owner: None)
        self.release = release or (lambda owner: None)
        self.robot_factory = robot_factory
        self.directory = Path(directory).expanduser().resolve()
        self.authorized = authorized
        self.clock = clock
        self.robot = None
        self.worker = None
        self.owner = ''
        self.generation = 0
        self.message = 'idle'
        self.phase = 'IDLE'
        self.cancelled = threading.Event()
        self.lock = threading.RLock()
        self.heartbeat_at = clock()
        self.cancel_external = None

    def status(self):
        with self.lock:
            robot = self.robot
            return dict(owner=self.owner, busy=bool(self.worker and self.worker.is_alive()),
                        state=robot.state.value if robot else 'DISCONNECTED', phase=self.phase,
                        recording=bool(robot and robot.continuous_recording), message=self.message,
                        source='hardware-feedback', motion_authorized=self.authorized,
                        points=len(robot.session.points) if robot else 0,
                        path=len(robot.session.path) if robot else 0,
                        sessions=self.list_sessions())

    def list_sessions(self):
        if not self.directory.exists():
            return []
        return sorted(p.stem for p in self.directory.glob('*.json')
                      if not p.is_symlink() and re.fullmatch(r'[A-Za-z0-9_-]{1,64}', p.stem))[:200]

    def session_path(self, name):
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', name):
            raise ValueError('session name must contain 1–64 letters, digits, _ or -')
        path = self.directory / (name + '.json')
        if path.is_symlink():
            raise ValueError('session symlinks are forbidden')
        return path

    def checkpoint(self):
        if self.cancelled.is_set():
            raise RuntimeError('task cancelled')

    def reserve(self, owner):
        with self.lock:
            if not self.authorized:
                raise RuntimeError('motion_authorized is false in task backend')
            if self.owner and (self.owner != owner or owner == 'grasp'):
                raise RuntimeError(f'{self.owner} owns the task session')
            if self.worker and self.worker.is_alive():
                raise RuntimeError('task already executing')
            if not self.owner:
                self.acquire(owner)
                self.generation += 1
            self.owner = owner
            self.cancelled.clear()
            self.heartbeat_at = self.clock()

    def run(self, action, label):
        with self.lock:
            if self.worker and self.worker.is_alive():
                raise RuntimeError('task already executing')
            self.cancelled.clear()
            self.heartbeat_at = self.clock()
            self.phase = label
            self.message = 'accepted; awaiting completion'
            def work():
                try:
                    action()
                    self.checkpoint()
                    self.phase = 'COMPLETED'
                    self.message = label + ' completed'
                except Exception as exc:
                    self.phase = 'FAILED'
                    self.message = str(exc)
                    try:
                        self._hold()
                    except Exception as hold_error:
                        self.message += '; hold unconfirmed: ' + str(hold_error)
            self.worker = threading.Thread(target=work, name='web-teach-task', daemon=True)
            self.worker.start()

    def _hold(self):
        if self.robot:
            self.robot.cancel_replay()
            self.robot._client.stop_and_hold()
            if self.robot.state is TeachState.GUIDING:
                self.robot.hold()

    def cancel(self):
        self.cancelled.set()
        if self.cancel_external:
            self.cancel_external()
        if self.robot:
            self.robot.cancel_replay()
            self.robot._client.stop_and_hold()
        with self.lock:
            if self.robot and self.robot.state is TeachState.GUIDING and not (self.worker and self.worker.is_alive()):
                self.run(self.robot.hold, 'HOLD')
                self.cancelled.set()  # No later motion may follow this hold.
        self.message = 'cancel requested; controller hold requested'

    def watchdog(self):
        if self.owner and self.clock() - self.heartbeat_at > 3.0 and not self.cancelled.is_set():
            self.cancel()

    def sample(self):
        if self.robot and self.robot.continuous_recording:
            try:
                if len(self.robot.session.path) >= 10000:
                    raise RuntimeError('recording reached 10000 samples')
                self.robot.sample_continuous()
            except Exception as exc:
                self.cancel()
                self.phase = 'FAILED'
                self.message = str(exc)

    def command(self, command, name='', supervised=False):
        if command == 'status':
            self.heartbeat_at = self.clock()
            return
        if command == 'cancel':
            self.cancel()
            return
        with self.lock:
            if command == 'begin':
                if not supervised:
                    raise RuntimeError('attended hand guiding acknowledgement required')
                self.reserve('teach')
                def begin():
                    if self.robot is None:
                        self.robot = self.robot_factory()
                        try:
                            self.robot.start()
                        except Exception:
                            self.robot._client.close()
                            self.robot = None
                            self.release('teach')
                            self.owner = ''
                            raise
                    self.checkpoint()
                    self.robot._client.latest_sample()
                    self.robot.enter_guiding()
                self.run(begin, 'GUIDING')
                return
            if self.owner != 'teach' or self.robot is None:
                raise RuntimeError('no active teaching session')
            if self.worker and self.worker.is_alive():
                raise RuntimeError('wait for the active task result')
            if command == 'capture':
                self.robot.capture_point()
            elif command == 'clear':
                self.robot.clear_recording()
            elif command == 'record':
                self.robot.toggle_continuous_recording()  # Never silently replace a path.
            elif command == 'hold':
                self.run(self.robot.hold, 'HOLD')
            elif command in ('replay_points', 'replay_path'):
                validate_session(self.robot.session)
                self.run(getattr(self.robot, command), command.upper())
            elif command in ('release', 'exit'):
                def finish():
                    if command == 'exit':
                        self.robot.safe_exit()
                    else:
                        if self.robot.state is TeachState.GUIDING:
                            self.robot.hold()
                        self.robot._client.stop_and_hold()
                        self.robot._client.close()
                    self.release('teach')
                    self.robot = None
                    self.owner = ''
                self.run(finish, command.upper())
            elif command == 'save':
                validate_session(self.robot.session)
                target = self.session_path(name)
                self.directory.mkdir(parents=True, exist_ok=True)
                # An atomic hard link refuses overwriting even if another process creates the name.
                with tempfile.TemporaryDirectory(dir=self.directory) as temporary:
                    saved = self.robot.save(Path(temporary) / 'session.json')
                    data = json.loads(saved.read_text())
                    data['source'] = 'hardware-feedback'
                    saved.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2) + '\n')
                    os.link(saved, target)
            elif command == 'load':
                if self.robot.state is not TeachState.HOLD:
                    raise RuntimeError('hold before loading a session')
                source = self.session_path(name)
                if source.stat().st_size > 4 * 1024 * 1024:
                    raise ValueError('session exceeds 4 MiB')
                session = load_teaching_session(source)
                validate_session(session)
                self.robot.session = session
            else:
                raise ValueError('unknown teaching command')
            self.message = command + ' completed'


class DetectionStore:
    """Reject uncalibrated, stale or malformed detection snapshots."""
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.targets = {}
        self.latest = []
        self.received = 0.0
        self.sequence = 0

    def update(self, body, now_ros_s):
        self.sequence += 1
        self.latest = []
        if body.get('version') != 1 or body.get('frame_id') != 'base_link' or body.get('source') != 'hardware-rgbd' or body.get('calibration_status') != 'CALIBRATED_STATIONARY':
            return
        identity = body.get('calibration_identity', '')
        if not isinstance(identity, str) or not re.fullmatch('[a-f0-9]{64}', identity):
            return
        stamp = body.get('stamp', {})
        age = now_ros_s - (stamp.get('sec', 0) + stamp.get('nanosec', 0)*1e-9)
        if not 0 <= age <= 1.0:
            return
        targets = body.get('detections', [])
        if not isinstance(targets, list) or len(targets) > 10:
            raise ValueError('invalid detection list')
        self.received = self.clock()
        self.targets = {key: value for key,value in self.targets.items() if self.clock()-value[0] <= 1.0}
        for target in targets:
            xyz = np.asarray(target.get('position'), dtype=float)
            visual = target.get('backend') == 'yolo-graspnet'
            class_name = target.get('class_name', '')
            if (not re.fullmatch('[a-f0-9]{32}', str(target.get('id', '')))
                    or (not visual and target.get('color') != 'red')
                    or (visual and (not isinstance(class_name, str) or not 0 < len(class_name) <= 100))
                    or xyz.shape != (3,) or not np.all(np.isfinite(xyz))):
                raise ValueError('invalid detection')
            snapshot = dict(target, calibration_identity=identity)
            self.targets[target['id']] = (self.received, snapshot)
            self.latest.append(snapshot)

    def get(self, detection_id):
        captured, target = self.targets[detection_id]
        if self.clock()-captured > 1.0 or not self.latest:
            raise RuntimeError('detection stale or calibration unavailable')
        return target

    def verify(self, target):
        if self.clock()-self.received > 1.0 or not self.latest:
            raise RuntimeError('fresh calibrated target unavailable; recapture and retry')
        if not any(item.get('calibration_identity') == target.get('calibration_identity') and item.get('color') == target.get('color') and item.get('position_reference') == target.get('position_reference') and np.linalg.norm(np.asarray(item['position'])-target['position']) < 0.01 for item in self.latest):
            raise RuntimeError('target changed during planning; recapture and retry')
