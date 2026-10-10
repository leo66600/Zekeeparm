const test = require('node:test');
const assert = require('node:assert/strict');
const {spawn} = require('node:child_process');
const {once} = require('node:events');
const fs = require('node:fs');
const http = require('node:http');
const path = require('node:path');

const web = path.resolve(__dirname, '..');

test('desktop web serves assets and J3 limits; proxy preserves local guards without MuJoCo', async t => {
  const backend = http.createServer((req, res) => {
    assert.equal(req.url, '/health');
    res.setHeader('Content-Type', 'application/json');
    res.end('{"ok":true}');
  });
  backend.listen(0, '127.0.0.1');
  await once(backend, 'listening');
  t.after(() => new Promise(resolve => backend.close(resolve)));
  // Reserve a temporary local port without using the running robot's web stack.
  const reservation = http.createServer();
  reservation.listen(0, '127.0.0.1');
  await once(reservation, 'listening');
  const port = reservation.address().port;
  await new Promise(resolve => reservation.close(resolve));
  const server = spawn(process.execPath, [path.join(web, 'server.js')], {
    env: {...process.env, PORT: String(port), HTTPS: '0',
      ZKEEP_WS: path.resolve(web, '../../..'), ZKEEP_BRINGUP_SHARE: path.resolve(web, '..'),
      ZKEEP_TEXT_AGENT_URL: `http://127.0.0.1:${backend.address().port}`},
    stdio: ['ignore', 'pipe', 'pipe'],
  });
  let output = '';
  server.stdout.on('data', chunk => {output += chunk;});
  server.stderr.on('data', chunk => {output += chunk;});
  t.after(async () => {
    if (server.exitCode === null && server.signalCode === null) {
      const exited = once(server, 'exit');
      server.kill('SIGTERM');
      await exited;
    }
  });
  const base = `http://127.0.0.1:${port}`;
  let ready = false;
  for (let attempt = 0; attempt < 100; attempt++) {
    try {ready = (await fetch(base)).ok;} catch (_) {}
    if (ready || server.exitCode !== null) break;
    await new Promise(resolve => setTimeout(resolve, 20));
  }
  assert.ok(ready, output);
  const html = await (await fetch(base)).text();
  assert.doesNotMatch(html, /id="physics-|rebot-physics\.js/);
  for (const match of html.matchAll(/(?:src|href)="(\/[^"#]+)"/g)) {
    assert.equal((await fetch(base + match[1])).status, 200, match[1]);
  }
  const config = await (await fetch(base + '/api/config')).json();
  assert.equal(config.gripper.motorOpenRadians, 1.35);
  assert.ok(Math.abs(config.gripper.openMeters - .07 * 1.35 / 1.45) < 1e-12);
  assert.equal(config.joints.find(joint => joint.name === 'joint3').min, -0.01);
  assert.match(await (await fetch(base + '/api/urdf')).text(), /<joint name="joint3"[\s\S]*?<limit lower="-0.01"/);
  assert.equal((await fetch(base + '/api/description/meshes/base_link.STL')).status, 200);
  assert.deepEqual(await (await fetch(base + '/api/llm/health')).json(), {ok: true});
  assert.equal((await fetch(base + '/api/llm/health', {headers: {Origin: 'http://other.example'}})).status, 403);
  assert.equal((await fetch(base + '/api/llm/health', {method: 'POST'})).status, 405);
  assert.equal((await fetch(base + '/api/llm/chat', {method: 'POST', body: 'bad'})).status, 415);
  assert.equal((await fetch(base + '/api/sim/state')).status, 404);
  assert.equal((await fetch(base + '/js/rebot-physics.js')).status, 404);
  assert.equal((await fetch(base + '/.env')).status, 404);
  assert.equal(fs.existsSync(path.resolve(web, '../../../zekeeparm_SDK/zekeeparm_SDK')), true);
});
