const test = require('node:test');
const assert = require('node:assert/strict');
const policy = require('../public/js/ros/rebot-control-policy.js');

test('web default speed shortens motion while preserving peak and requested time', () => {
  const start = [0, 0, 0, 0, 0, 0];
  const goal = [0.5, 0, 0, 0, 0, 0];
  const duration = policy.computeJointTrajectoryDuration(start, goal, undefined, 2);
  assert.equal(duration, 2.5);
  assert.equal(1.5 * 0.5 / duration, 0.3);
  assert.equal(policy.computeJointTrajectoryDuration(start, goal, 0.1, 2), 7.5);
  assert.equal(policy.computeJointTrajectoryDuration(start, goal, 0.3, 6), 6);
});
