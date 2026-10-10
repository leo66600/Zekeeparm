const http = require('http');
const https = require('https');
const fs = require('fs');
const os = require('os');
const path = require('path');

const { URL } = require('url');

// Load .env file (does not override existing env vars)
(function loadEnv() {
  const envPath = path.join(__dirname, '.env');
  try {
    if (fs.existsSync(envPath)) {
      const lines = fs.readFileSync(envPath, 'utf8').split('\n');
      for (const line of lines) {
        const trimmed = line.trim();
        if (!trimmed || trimmed.startsWith('#')) continue;
        const eq = trimmed.indexOf('=');
        if (eq < 1) continue;
        const key = trimmed.slice(0, eq).trim();
        const val = trimmed.slice(eq + 1).trim().replace(/^["']|["']$/g, '');
        if (!(key in process.env)) process.env[key] = val;
      }
    }
  } catch (_) {}
})();

const USE_HTTPS = process.env.HTTPS === '1';
const PORT = Number(process.env.PORT || (USE_HTTPS ? 3443 : 3001));
const ROOT = __dirname;
const PUBLIC_DIR = path.join(ROOT, 'public');
const BRINGUP_DIR = path.resolve(process.env.ZKEEP_BRINGUP_SHARE ||
  (process.env.ZKEEP_WS ? path.join(process.env.ZKEEP_WS, 'src', 'zekeep_bringup') : path.join(ROOT, '..')));
const URDF_FILE = path.join(BRINGUP_DIR, 'description', 'urdf', 'sixaxis.urdf');
const MESHES_DIR = path.join(BRINGUP_DIR, 'description', 'meshes');
const DEFAULT_KEY_FILE = path.join(ROOT, '.certs', 'zekeep-local-server.key');
const DEFAULT_CERT_FILE = path.join(ROOT, '.certs', 'zekeep-local-server.crt');

// MCP/LLM 配置（前端只负责代理到虚拟机的 text-agent HTTP 服务）
const DEFAULT_TEXT_AGENT_URL = process.env.ZKEEP_TEXT_AGENT_URL || 'http://localhost:8082';
const DEFAULT_MCP_URL = process.env.ZKEEP_MCP_URL || `${DEFAULT_TEXT_AGENT_URL.replace(/\/+$/, '')}/mcp`;
const MAX_REQUEST_BODY_BYTES = 1024 * 1024;

const MIME_TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.js': 'application/javascript; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.webmanifest': 'application/manifest+json; charset=utf-8',
  '.png': 'image/png',
  '.stl': 'model/stl',
  '.STL': 'model/stl',
  '.urdf': 'application/xml; charset=utf-8',
  '.xml': 'application/xml; charset=utf-8'
};

function send(res, status, body, type) {
  res.writeHead(status, {
    'Content-Type': type || 'text/plain; charset=utf-8',
    'Cache-Control': 'no-store'
  });
  res.end(body);
}

function sendJson(res, status, body) {
  send(res, status, JSON.stringify(body, null, 2), MIME_TYPES['.json']);
}

function sendFile(res, filePath) {
  fs.stat(filePath, (statErr, stat) => {
    if (statErr || !stat.isFile()) {
      sendJson(res, 404, { error: 'File not found' });
      return;
    }

    const ext = path.extname(filePath);
    res.writeHead(200, {
      'Content-Type': MIME_TYPES[ext] || 'application/octet-stream',
      'Content-Length': stat.size,
      'Cache-Control': 'no-store'
    });
    fs.createReadStream(filePath).pipe(res);
  });
}

function safePublicPath(urlPath) {
  const cleanPath = decodeURIComponent(urlPath.split('?')[0]);
  const relative = cleanPath === '/' ? 'index.html' : cleanPath.replace(/^\/+/, '');
  const filePath = path.resolve(path.join(PUBLIC_DIR, relative));
  if (filePath !== PUBLIC_DIR && !filePath.startsWith(`${PUBLIC_DIR}${path.sep}`)) {
    return null;
  }
  return filePath;
}

function sendMesh(res, filename) {
  const safeName = path.basename(filename);
  sendFile(res, path.join(MESHES_DIR, safeName));
}

function getLanAddresses() {
  return Object.values(os.networkInterfaces())
    .flat()
    .filter((item) => item && item.family === 'IPv4' && !item.internal)
    .map((item) => item.address);
}

function requestHandler(req, res) {
  let urlPath;
  try {
    urlPath = req.url.split('?')[0];
    if (decodeURIComponent(urlPath).includes('\0')) throw new URIError('NUL path');
  } catch (_) {
    sendJson(res, 400, { error: 'Malformed URL' });
    return;
  }

  // MCP 配置端点
  if (urlPath === '/api/mcp/config') {
    sendJson(res, 200, {
      textAgentUrl: DEFAULT_TEXT_AGENT_URL,
      mcpUrl: DEFAULT_MCP_URL
    });
    return;
  }

  const agentRoutes = {
    '/api/llm/chat': '/plan', '/api/llm/execute': '/execute',
    '/api/llm/cancel': '/cancel', '/api/llm/health': '/health',
    '/api/llm/status': '/status'
  };
  if (agentRoutes[urlPath]) {
    handleLocalBackend(req, res, DEFAULT_TEXT_AGENT_URL, agentRoutes[urlPath]);
    return;
  }
  if (urlPath === '/api/mcp/rpc') {
    handleLocalBackend(req, res, DEFAULT_MCP_URL, '');
    return;
  }
  if (urlPath === '/api/config') {
    sendJson(res, 200, {
      name: 'Six-axis reBot Arm',
      joints: [
        { name: 'joint1', min: -2.58, max: 2.58, home: 0, maxVelocity: 1.5 },
        { name: 'joint2', min: 0, max: 3.7, home: 0, maxVelocity: 1.5 },
        { name: 'joint3', min: -0.01, max: 3.7, home: 0, maxVelocity: 1.5 },
        { name: 'joint4', min: -1.57, max: 1.57, home: 0, maxVelocity: 1.5 },
        { name: 'joint5', min: -1.57, max: 1.57, home: 0, maxVelocity: 1.5 },
        { name: 'joint6', min: -1.57, max: 1.57, home: 0, maxVelocity: 1.5 }
      ],
      frame: {
        rosX: 'forward',
        rosY: 'left',
        rosZ: 'up',
        threeMapping: { x: 'ros_x', y: 'ros_z', z: '-ros_y' }
      },
      reachMeters: 0.65,
      payloadKg: 1.5,
      gripper: {
        name: 'gripper',
        motorId: '0x07',
        closedMeters: 0,
        openMeters: 0.06517241379310346,
        motorOpenRadians: 1.35,
        urdfFingerTravelMeters: 0.03258620689655173,
        rosService: '/zekeep/gripper/set'
      }
    });
    return;
  }

  if (urlPath === '/api/urdf') {
    sendFile(res, URDF_FILE);
    return;
  }

  const meshMatch = urlPath.match(/^\/api\/(?:description\/)?meshes\/(.+)$/);
  if (meshMatch) {
    sendMesh(res, meshMatch[1]);
    return;
  }

  const filePath = safePublicPath(urlPath);
  if (!filePath) {
    sendJson(res, 403, { error: 'Forbidden' });
    return;
  }

  sendFile(res, filePath);
}

function createServer() {
  if (!USE_HTTPS) return http.createServer(requestHandler);

  const keyFile = process.env.HTTPS_KEY || DEFAULT_KEY_FILE;
  const certFile = process.env.HTTPS_CERT || DEFAULT_CERT_FILE;

  if (!fs.existsSync(keyFile) || !fs.existsSync(certFile)) {
    console.error(`HTTPS certificate not found: ${keyFile} / ${certFile}`);
    console.error('Run: npm run cert:dev');
    process.exit(1);
  }

  return https.createServer({
    key: fs.readFileSync(keyFile),
    cert: fs.readFileSync(certFile)
  }, requestHandler);
}

const server = createServer();

// 读取请求体
function readBody(req) {
  return new Promise((resolve, reject) => {
    let body = '';
    let bodyBytes = 0;
    let rejected = false;
    req.on('data', chunk => {
      bodyBytes += Buffer.byteLength(chunk);
      if (bodyBytes > MAX_REQUEST_BODY_BYTES && !rejected) {
        rejected = true;
        req.destroy();
        reject(new Error('request body too large'));
        return;
      }
      body += chunk;
    });
    req.on('end', () => {
      if (!rejected) resolve(body);
    });
    req.on('error', reject);
  });
}

// HTTP 请求代理
function proxyRequest(targetUrl, options, body, timeoutMs = 15000) {
  return new Promise((resolve, reject) => {
    let url;
    try {
      url = new URL(targetUrl);
    } catch (e) {
      reject(new Error(`Invalid URL: ${targetUrl}`));
      return;
    }
    const lib = url.protocol === 'https:' ? https : http;
    const reqOptions = {
      hostname: url.hostname,
      port: url.port || (url.protocol === 'https:' ? 443 : 80),
      path: url.pathname + url.search,
      method: options.method || 'POST',
      timeout: timeoutMs,
      headers: {
        'Content-Type': 'application/json',
        'Content-Length': Buffer.byteLength(body),
        ...options.headers
      }
    };

    const proxyReq = lib.request(reqOptions, (proxyRes) => {
      let data = '';
      // SSE 是流式响应，收到 headers 后立即返回初始数据，不等 end
      const contentType = proxyRes.headers['content-type'] || '';
      const isSSE = contentType.includes('text/event-stream');

      if (isSSE) {
        // 流式响应：读取直到遇到第一个完整事件或超时
        let buffer = '';
        const readTimeout = setTimeout(() => {
          resolve({
            status: proxyRes.statusCode,
            headers: proxyRes.headers,
            body: buffer
          });
          proxyReq.destroy();
        }, timeoutMs);

        proxyRes.on('data', chunk => {
          buffer += chunk.toString('utf8');
          // 如果收到完整的事件（data: ... \n\n），立即返回
          if (buffer.includes('\n\n')) {
            clearTimeout(readTimeout);
            resolve({
              status: proxyRes.statusCode,
              headers: proxyRes.headers,
              body: buffer
            });
            proxyReq.destroy();
          }
        });
        proxyRes.on('end', () => {
          clearTimeout(readTimeout);
          resolve({
            status: proxyRes.statusCode,
            headers: proxyRes.headers,
            body: buffer
          });
        });
        proxyRes.on('error', (err) => {
          clearTimeout(readTimeout);
          reject(err);
        });
      } else {
        proxyRes.on('data', chunk => { data += chunk; if (Buffer.byteLength(data) > 4 * 1024 * 1024) { proxyReq.destroy(); reject(new Error('Backend response too large')); } });
        proxyRes.on('end', () => {
          resolve({
            status: proxyRes.statusCode,
            headers: proxyRes.headers,
            body: data
          });
        });
        proxyRes.on('error', (err) => reject(err));
      }
    });

    proxyReq.on('timeout', () => {
      proxyReq.destroy();
      reject(new Error(`Request timeout after ${timeoutMs}ms: ${targetUrl}`));
    });
    proxyReq.on('error', reject);

    if (body) {
      proxyReq.write(body);
    }
    proxyReq.end();
  });
}

// Fixed routes only. Browser requests cannot choose upstream URLs or ROS endpoints.
async function handleLocalBackend(req, res, backend, endpoint) {
  try {
    const expected = ['health', 'status', 'state', 'camera'].some(name => endpoint === `/${name}`) ? 'GET' : 'POST';
    if (req.method !== expected) return sendJson(res, 405, {ok: false, error: 'Method not allowed'});
    const host = req.headers.host || '';
    if (!/^(localhost|127\.0\.0\.1)(:\d+)?$/.test(host)) return sendJson(res, 403, {ok: false, error: 'Local host required'});
    if (req.headers.origin && req.headers.origin !== `${USE_HTTPS ? 'https' : 'http'}://${host}`) {
      return sendJson(res, 403, {ok: false, error: 'Same origin required'});
    }
    if (expected === 'POST' && !(req.headers['content-type'] || '').startsWith('application/json')) {
      return sendJson(res, 415, {ok: false, error: 'JSON required'});
    }
    const body = expected === 'POST' ? await readBody(req) : '';
    if (body && Buffer.byteLength(body) > 16384) return sendJson(res, 413, {ok: false, error: 'Body too large'});
    const response = await proxyRequest(`${backend.replace(/\/+$/, '')}${endpoint}`, {method: expected}, body, endpoint === '/execute' ? 1810000 : 90000);
    res.writeHead(response.status, {'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'no-store'});
    res.end(response.body);
  } catch (error) {
    sendJson(res, 502, {ok: false, error: `Backend unavailable: ${error.message}`});
  }
}

server.listen(PORT, '127.0.0.1', () => {
  const protocol = USE_HTTPS ? 'https' : 'http';

  console.log('========================================');
  console.log('  Six-axis reBot Arm Simulator Started');
  console.log('========================================');
  console.log(`  Local: ${protocol}://localhost:${PORT}`);

  console.log(`  URDF:  ${protocol}://localhost:${PORT}/api/urdf`);
  console.log(`  Mesh:  ${protocol}://localhost:${PORT}/api/description/meshes/base_link.STL`);
  console.log('----------------------------------------');
  console.log(`  URDF file: ${URDF_FILE}`);
  console.log(`  Mesh dir:  ${MESHES_DIR}`);
});
