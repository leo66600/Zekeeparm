const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const policy = require('../public/js/ros/rebot-control-policy.js');
const THREE = require('../public/lib/three-r128.min.js');
const source = name => fs.readFileSync(path.join(__dirname, '../public/js', name), 'utf8');

function fixture(connected = true) {
  const fields = new Map(['x', 'y', 'z', 'duration', 'status'].map(name => [`ros-pose-${name === 'status' ? 'preview-status' : name}`, {
    value: name === 'duration' ? '2' : name === 'x' ? '.3' : name === 'z' ? '.3' : '0',
    textContent: '', events: {}, addEventListener(type, callback) { this.events[type] = callback; },
  }]));
  fields.set('ros-pose-preview-toggle', {
    textContent: '', events: {}, attributes: {},
    addEventListener(type, callback) { this.events[type] = callback; },
    setAttribute(name, value) { this.attributes[name] = value; },
  });
  const events = {}, timers = new Map(), calls = [], drawings = [], pending = [];
  let clears = 0;
  const angles = Object.fromEntries(Array.from({length: 6}, (_, i) => [`joint${i + 1}`, 0]));
  const definitions = Object.keys(angles).map(name => ({name, min: -2, max: 3.7}));
  const client = {connected, addEventListener(type, callback) { events[type] = callback; },
    previewPoseIK(...args) { calls.push(args); return new Promise(resolve => pending.push(resolve)); }};
  const window = {reBotRos: client, ReBotControlPolicy: policy,
    rebotI18n: {t: (key, values) => key + JSON.stringify(values || {}),
      onLangChange(callback) { events.language = callback; }},
    reBotSim: {getAngles: () => ({...angles}), getJointDefs: () => definitions,
      getTcpPose: () => ({orientation: {x: 0, y: 0, z: 0, w: 1}}),
      previewPose(...args) { drawings.push(args); }, clearPosePreview() { clears += 1; }},
    setTimeout(callback) { const id = Symbol(); timers.set(id, callback); return id; },
    clearTimeout(id) { timers.delete(id); }, addEventListener(type, callback) { events[type] = callback; }};
  vm.runInNewContext(source('ros/rebot-pose-preview.js'), {window, document: {getElementById: id => fields.get(id)}});
  const runTimer = () => {const callback = [...timers.values()].at(-1); timers.clear(); return callback();};
  const result = q => ({error_code: {val: 1}, solution: {joint_state: {name: Object.keys(angles), position: Array(6).fill(q)}}});
  return {fields, calls, drawings, pending, runTimer, result, window, angles, events, timers, get clears() { return clears; }};
}

test('MuJoCo entry removed; editing pose previews without sending movement', async () => {
  const html = fs.readFileSync(path.join(__dirname, '../public/index.html'), 'utf8');
  assert.doesNotMatch(html, /id="physics-|rebot-physics\.js|MuJoCo 物理仿真/);
  const f = fixture();
  const work = f.runTimer();
  assert.equal(f.calls.length, 1);
  f.pending.shift()(f.result(.2));
  await work;
  assert.equal(f.drawings.at(-1)[2].joint1, .2);
  assert.match(f.fields.get('ros-pose-preview-status').textContent, /previewReady/);
  f.fields.get('ros-pose-duration').value = '6';
  f.fields.get('ros-pose-duration').events.input();
  const next = f.runTimer(); f.pending.shift()(f.result(.2)); await next;
  assert.equal(f.drawings.at(-1)[1], 6);
  assert.equal(f.window.reBotPosePreview.getTarget().duration, 6);
  assert.equal(f.angles.joint1, 0);
});

test('stale IK replies and invalid values cannot replace the current preview', async () => {
  const f = fixture();
  const old = f.runTimer();
  f.fields.get('ros-pose-x').value = '.4'; f.fields.get('ros-pose-x').events.input();
  const next = f.runTimer();
  f.pending[1](f.result(.4)); await next;
  const count = f.drawings.length;
  f.pending[0](f.result(.2)); await old;
  assert.equal(f.drawings.length, count);
  assert.equal(f.drawings.at(-1)[0].x, .4);
  f.fields.get('ros-pose-x').value = ''; f.fields.get('ros-pose-x').events.input();
  assert.equal(f.window.reBotPosePreview.getTarget(), null);
  assert.match(f.fields.get('ros-pose-preview-status').textContent, /previewInvalid/);
  assert.ok(f.clears > 0);
});

test('preview toggle ignores late IK and stays off through edits, reconnect and language changes', async () => {
  const f = fixture();
  const button = f.fields.get('ros-pose-preview-toggle');
  assert.equal(button.attributes['aria-pressed'], 'true');
  const pending = f.runTimer();
  button.events.click();
  assert.equal(button.attributes['aria-pressed'], 'false');
  assert.match(button.textContent, /previewShow/);
  assert.match(f.fields.get('ros-pose-preview-status').textContent, /previewOff/);
  const count = f.drawings.length;
  f.pending.shift()(f.result(.2)); await pending;
  assert.equal(f.drawings.length, count, 'late IK cannot restore a hidden preview');
  f.fields.get('ros-pose-x').value = '.4';
  f.fields.get('ros-pose-x').events.input();
  f.events.status();
  f.events.language();
  assert.equal(f.drawings.length, count);
  assert.equal(f.calls.length, 1);
  assert.equal(f.timers.size, 0);
  assert.equal(f.window.reBotPosePreview.getTarget().pose.position.x, .4);
  button.events.click();
  assert.equal(button.attributes['aria-pressed'], 'true');
  assert.match(button.textContent, /previewHide/);
  assert.equal(f.drawings.at(-1)[0].x, .4);
  const resumed = f.runTimer();
  f.pending.shift()(f.result(.3)); await resumed;
  assert.equal(f.drawings.at(-1)[2].joint1, .3);
  assert.equal(f.angles.joint1, 0);
  button.events.click();
  button.events.click();
  assert.equal(f.timers.size, 1);
  button.events.click();
  assert.equal(f.timers.size, 0, 'hiding cancels an IK request before it starts');
});

test('offline and failed IK show target point with explicit status', async () => {
  const offline = fixture(false);
  assert.equal(offline.calls.length, 0);
  assert.match(offline.fields.get('ros-pose-preview-status').textContent, /previewOffline/);
  const f = fixture(); const work = f.runTimer();
  f.pending.shift()({error_code: {val: -31}}); await work;
  assert.match(f.fields.get('ros-pose-preview-status').textContent, /previewNoIK/);
  assert.equal(f.drawings.at(-1).length, 2);
});

function functionSource(text, name) {
  const start = text.indexOf(`function ${name}(`);
  assert.ok(start >= 0, name);
  const brace = text.indexOf('{', start);
  let depth = 0;
  for (let i = brace; i < text.length; i += 1) {
    if (text[i] === '{') depth += 1;
    if (text[i] === '}' && --depth === 0) return text.slice(start, i + 1);
  }
  throw new Error(`Unclosed ${name}`);
}

test('rendered preview animates only ghost joints and converts ROS coordinates', () => {
  const names = ['joint1', 'joint2', 'joint3'];
  const current = {joint1: 0, joint2: 0, joint3: 0};
  const model = new THREE.Group(), ghost = new THREE.Group();
  const marker = new THREE.Mesh(new THREE.SphereGeometry(.01), new THREE.MeshBasicMaterial());
  const line = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial());
  let clock = 0;
  const context = vm.createContext({THREE, robot: model, ghostRobot: ghost, currentAngles: current,
    posePreview: null, targetGhost: marker, posePreviewLine: line,
    jointDefs: names.map(name => ({name})), performance: {now: () => clock},
    document: {getElementById: () => ({checked: true})},
    clamp: (v, lo, hi) => Math.max(lo, Math.min(v, hi)),
    setGhostJoint: (name, value) => {ghost.userData[name] = value;},
    getTcpPosition: root => new THREE.Vector3(root.userData.joint1 || 0, 0, 0),
    syncGhostToRobot: () => {names.forEach(name => {ghost.userData[name] = current[name];});}});
  const text = source('rebot-sim.js');
  for (const name of ['previewPose', 'updatePosePreview', 'clearPosePreview']) vm.runInContext(functionSource(text, name), context);
  context.previewPose({x: .3, y: -.1, z: .25}, 2, {joint1: .2, joint2: .4, joint3: .6});
  assert.deepEqual(marker.position.toArray(), [.3, .25, .1]);
  clock = 1000; context.updatePosePreview(clock);
  assert.equal(ghost.userData.joint1, .1);
  clock = 2000; context.updatePosePreview(clock);
  assert.equal(ghost.userData.joint1, .2);
  assert.deepEqual(current, {joint1: 0, joint2: 0, joint3: 0});
  context.clearPosePreview();
  assert.equal(marker.visible, false); assert.equal(line.visible, false);
});

test('IK execution keeps control guards and avoids optimistic movement of feedback model', () => {
  const text = source('ros/rebot-ros-ui.js');
  const begin = text.indexOf('async function checkIk()');
  const end = text.indexOf('async function queryGravityCompensation', begin);
  const body = text.slice(begin, end);
  assert.ok(body.indexOf('controlAllowed(true)') < body.indexOf('moveToPoseViaIkTrajectory('));
  assert.doesNotMatch(body.slice(body.indexOf('controlAllowed(true)')), /moveToTcp\(/);
});

test('preview requests use only MoveIt IK and preserve scene attachments', async () => {
  const window = {};
  vm.runInNewContext(source('ros/rebot-ros-client.js'), {window, EventTarget, CustomEvent});
  const calls = [];
  const fake = {callService: (...args) => calls.push(args)};
  const pose = {position: {x: .3, y: 0, z: .3}, orientation: {x: 0, y: 0, z: 0, w: 1}};
  window.ReBotRosClient.prototype.previewPoseIK.call(fake, pose, ['joint1'], [0]);
  assert.equal(calls[0][0], '/compute_ik');
  assert.equal(calls[0][1], 'moveit_msgs/srv/GetPositionIK');
  assert.equal(calls[0][2].ik_request.avoid_collisions, true);
  assert.equal(calls[0][2].ik_request.robot_state.is_diff, true);
});
