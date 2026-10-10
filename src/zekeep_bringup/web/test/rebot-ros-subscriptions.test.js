const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

global.window = {};
vm.runInThisContext(fs.readFileSync(path.join(__dirname, '../public/js/ros/rebot-ros-client.js'), 'utf8'));

const client = new window.ReBotRosClient();
const sent = [];
client._send = message => { sent.push(message); return true; };
client.connected = true;
const topic = '/zekeep/joint_states';
const calls = [];
const feedback = message => calls.push(['control', message]);
const physics = message => calls.push(['physics', message]);
client.subscribe(topic, 'sensor_msgs/msg/JointState', feedback, {throttleRate: 80});
client.subscribe(topic, 'sensor_msgs/msg/JointState', physics, {throttleRate: 200});
client.subscribe(topic, 'sensor_msgs/msg/JointState', physics, {throttleRate: 200});
const message = {name: ['joint1'], position: [0.1]};
const event = {data: JSON.stringify({op: 'publish', topic, msg: message})};
client._handleMessage(event);
assert.deepEqual(calls, [['control', message], ['physics', message]]);
assert.equal(sent.at(-1).throttle_rate, 80);
assert.ok(client.getLastMessageAt(topic) > 0);

sent.length = 0;
client._resubscribe();
assert.equal(sent.length, 1);
assert.equal(sent[0].throttle_rate, 80);
calls.length = 0;
client._handleMessage(event);
assert.equal(calls.length, 2);

client.unsubscribe(topic);
calls.length = 0;
client._handleMessage(event);
assert.equal(calls.length, 0);
assert.equal(sent.at(-1).op, 'unsubscribe');
console.log('PASS: shared feedback callbacks, deduplication, throttle, reconnect, unsubscribe');
