const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const attempts = [];
class FakeWebSocket extends EventTarget {
  static OPEN = 1;
  static CONNECTING = 0;
  constructor(url) {
    super();
    const parsed = new URL(url);
    if (!['ws:', 'wss:'].includes(parsed.protocol)) throw new SyntaxError('Invalid WebSocket protocol');
    attempts.push(url);
    this.readyState = FakeWebSocket.CONNECTING;
  }
}
const window = {rebotI18n: {t: key => key}, clearTimeout() {}, setTimeout() {}};
vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../public/js/ros/rebot-ros-client.js'), 'utf8'),
  {window, WebSocket: FakeWebSocket, EventTarget, CustomEvent});
const client = new window.ReBotRosClient({url: ''});
client.connect();
assert.equal(attempts[0], 'ws://127.0.0.1:9090');
const custom = new window.ReBotRosClient({url: 'ws://custom-host:9090'});
custom.connect();
assert.equal(attempts[1], 'ws://custom-host:9090', 'Explicit saved addresses must remain supported');
const invalid = new window.ReBotRosClient({url: 'bad-address'});
const statuses = [];
invalid.addEventListener('status', event => statuses.push(event.detail));
assert.doesNotThrow(() => invalid.connect());
assert.equal(invalid.connected, false);
assert.equal(statuses.at(-1).state, 'error');
assert.match(statuses.at(-1).message, /Invalid URL/);
console.log('PASS: empty address uses local ROS port, explicit addresses preserved, malformed URL reports error');
