const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const html = fs.readFileSync(path.join(__dirname, '../public/index.html'), 'utf8');
assert.doesNotMatch(html, /id="(?:ros-camera-[^"]*|web-grasp-[^"]*|web-calibration-status|web-release-object)"/);
const fields = new Map([...html.matchAll(/id="([^"]+)"/g)].map((match) => [match[1], {
  value: '', checked: true, dataset: {}, textContent: '', events: {}, children: [],
  addEventListener(type, callback) { this.events[type] = callback; },
  replaceChildren() { this.children = []; }, appendChild(child) { this.children.push(child); },
}]));
const begin = {dataset: {teachCommand: 'begin'}, addEventListener(type, callback) { this.click = callback; }};
const clear = fields.get('hardware-teach-clear');
clear.dataset.teachCommand = 'clear';
const subscriptions = new Map(), listeners = new Map(), calls = [];
let interval, confirmed = true;
let state = {owner: '', phase: 'IDLE', sessions: ['saved'], motion_authorized: true};
const client = {
  connected: true,
  subscribe(topic, type, callback) { subscriptions.set(topic, callback); },
  addEventListener(type, callback) { listeners.set(type, callback); },
  async callService(service, type, args) {
    calls.push({service, args});
    if (args.command === 'clear') state = {...state, points: 0, path: 0};
    return {success: true, status_json: JSON.stringify(state)};
  },
};
const window = {reBotRos: client, rebotI18n: {t: key => key}, confirm: () => confirmed, setInterval(callback) { interval = callback; }};
const element = id => fields.get(id) || null;
vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../public/js/ros/rebot-web-tasks.js'), 'utf8'), {
  window, document: {getElementById: element, querySelectorAll: () => [begin, clear], createElement: () => ({})},
});
const status = () => subscriptions.get('/zekeep/web_tasks/status')({data: JSON.stringify(state)});
(async () => {
  assert.deepEqual([...subscriptions.keys()], ['/zekeep/web_tasks/status']);
  await interval();
  assert.equal(calls.length, 0, 'No service polling without an available task backend');
  status();
  assert.equal(window.reBotTaskState.owner, '');
  assert.equal(element('hardware-teach-files').children[0].value, 'saved');
  await interval();
  assert.equal(calls.at(-1).args.command, 'status');
  element('ros-control-enable').checked = false;
  let count = calls.length;
  await begin.click();
  assert.equal(calls.length, count, 'Control lock still blocks teaching motion');
  element('ros-control-enable').checked = true;
  window.reBotAIActive = true;
  await begin.click();
  assert.equal(calls.length, count, 'AI ownership still blocks teaching');
  window.reBotAIActive = false;
  await begin.click();
  assert.equal(calls.at(-1).args.command, 'begin');
  state = {...state, owner: 'teach', state: 'HOLD', points: 2, path: 20, busy: false, recording: false};
  status();
  assert.equal(clear.disabled, false);
  confirmed = false;
  count = calls.length;
  await clear.events.click();
  assert.equal(calls.length, count, 'Cancelling confirmation preserves the recording');
  confirmed = true;
  for (const blocked of [{busy: true}, {recording: true}, {state: 'PATH_REPLAY'}]) {
    const before = state;
    state = {...state, ...blocked}; status();
    await clear.events.click();
    assert.equal(calls.length, count, 'Active recording/replay must block clearing');
    state = before;
  }
  status();
  await clear.events.click();
  assert.equal(calls.at(-1).args.command, 'clear');
  assert.equal(window.reBotTaskState.points, 0);
  assert.equal(window.reBotTaskState.path, 0);
  assert.equal(clear.disabled, true);
  state = {...state, owner: 'grasp'};
  status();
  listeners.get('status')({detail: {state: 'closed'}});
  assert.equal(window.reBotTaskState.owner, 'grasp', 'Ownership survives disconnect');
  await window.reBotTasks.cancel();
  assert.equal(calls.at(-1).service, '/zekeep/web_tasks/cancel');
  console.log('PASS: camera/grasp UI removed; teaching status, control guards and cancellation retained');
})().catch(error => { console.error(error); process.exitCode = 1; });
