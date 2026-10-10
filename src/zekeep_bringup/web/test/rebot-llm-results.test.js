const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const ids = ['llm-start', 'llm-stop', 'llm-send', 'llm-confirm', 'llm-cancel', 'llm-input',
  'llm-plan', 'llm-status', 'llm-chat-messages', 'llm-supervised', 'ros-control-enable', 'mcp-refresh', 'mcp-run'];
const elements = Object.fromEntries(ids.map(id => [id, {
  children: [], checked: false, value: '', textContent: '', appendChild(row) { this.children.push(row); },
}]));
const document = {readyState: 'complete', documentElement: {}, querySelectorAll: () => [],
  getElementById: id => elements[id] || null,
  createElement: () => ({textContent: '', set innerHTML(_) { throw new Error('unsafe HTML'); }}),
};
let result, executions = 0, requiresConfirmation = false;
const context = vm.createContext({document, window: {confirm: () => true}, setInterval: () => {},
  localStorage: {getItem: () => 'zh', setItem: () => {}},
  fetch: async url => ({ok: true, json: async () => {
    if (url.endsWith('/health')) return {mode: 'plan-review-v1'};
    if (url.endsWith('/status')) return {motion_authorized: false};
    if (url.endsWith('/chat')) return {plan_id: 'test', text: '查询当前检测',
      requires_confirmation: requiresConfirmation, plan: {steps: [{tool: 'detect_blocks', arguments: {}}]}};
    if (url.endsWith('/execute')) { executions++; return result; }
    throw new Error(`Unexpected request ${url}`);
  }}),
});
for (const name of ['i18n.js', 'rebot-llm.js']) {
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../public/js', name), 'utf8'), context);
}
const messages = () => elements['llm-chat-messages'].children.map(row => row.textContent).join('\n');
async function query(detections, extra = {}) {
  const event = {type: 'tool', name: 'detect_blocks', result: JSON.stringify({detections, ...extra})};
  result = {text: '计划执行完成', execution_events: [event], events: [{type: 'tool', result: 'OLD HISTORY'}, event]};
  elements['llm-input'].value = '当前桌上有什么？';
  await elements['llm-send'].onclick();
  await elements['llm-confirm'].onclick();
}
(async () => {
  await elements['llm-start'].onclick();
  await query([{class_name: 'blue block', confidence: .613}, {color: 'red', confidence: .305}]);
  assert.match(messages(), /当前检测到 2 个物体/);
  assert.match(messages(), /蓝色物块 · 置信度 61%/);
  assert.match(messages(), /红色物块 · 置信度 31%/);
  assert.doesNotMatch(messages(), /OLD HISTORY|"detections"|"result"/);
  await query([{class_name: 'purple block', confidence: .85}]);
  assert.match(messages(), /紫色物块 · 置信度 85%/);
  await query([]);
  assert.match(messages(), /未检测到已配置类别的物体/);
  await query([{class_name: '<img src=x onerror=alert(1)>', confidence: null}]);
  assert.match(messages(), /<img src=x onerror=alert\(1\)> · 置信度 未知/);
  assert.ok(elements['llm-chat-messages'].children.every(row => typeof row.textContent === 'string'));
  await query(null);
  assert.match(messages(), /检测结果格式异常/);
  await query([{class_name: 'blue block', confidence: .3}], {
    observation_window_s: 2, observations: [
      {class_name: 'blue block', confidence: .4}, {class_name: 'blue block', confidence: .5},
      {class_name: 'red block', confidence: .6},
    ],
  });
  assert.match(messages(), /最近 2 秒内重复检测到 3 个物体/);
  await query([], {observations: [], observation_window_s: 2});
  assert.match(messages(), /最近 2 秒内未确认到重复检测的物体/);
  requiresConfirmation = true;
  const before = executions;
  await query([]);
  assert.equal(executions, before, 'Motion confirmation guard must still block');
  console.log('PASS: readable detections, current results only, empty/invalid data, safe text, motion guard');
})().catch(error => { console.error(error); process.exitCode = 1; });
