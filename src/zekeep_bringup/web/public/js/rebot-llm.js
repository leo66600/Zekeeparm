(function () {
  'use strict';
  const el = id => document.getElementById(id);
  const t = (key, params) => window.rebotI18n ? window.rebotI18n.t(key, params) : key;
  let plan = null, connected = false, busy = false, executing = false, generation = 0;
  const log = message => {
    const row = document.createElement('div');
    row.textContent = message;
    el('llm-chat-messages').appendChild(row);
    while (el('llm-chat-messages').children.length > 200) el('llm-chat-messages').firstChild.remove();
    el('llm-chat-messages').scrollTop = el('llm-chat-messages').scrollHeight;
  };
  function formatEvent(event) {
    if (event.type === 'error') return event.message;
    if (event.type !== 'tool') return null;
    if (event.name !== 'detect_blocks') return event.result;
    try {
      const data = JSON.parse(event.result);
      const detections = data.observations ?? data.detections;
      if (!Array.isArray(detections)) throw new Error('invalid detections');
      const lines = detections.map((item, index) => {
        const name = item.class_name || item.color || t('p45.unknownObject');
        const key = `p45.object.${name}`;
        const translated = t(key);
        const confidence = typeof item.confidence === 'number' && Number.isFinite(item.confidence) && item.confidence >= 0 && item.confidence <= 1
          ? `${Math.round(item.confidence * 100)}%` : t('p45.unknownConfidence');
        return `${index + 1}. ${translated === key ? name : translated} · ${t('p45.confidence', {value: confidence})}`;
      });
      const title = data.observation_window_s ? 'p45.recentObjects' : 'p45.detectedObjects';
      const empty = data.observation_window_s ? 'p45.noRecentObjects' : 'p45.noObjects';
      return [t(lines.length ? title : empty, {count: lines.length, seconds: data.observation_window_s}), ...lines, t('p45.detectionScope')].join('\n');
    } catch (_) {
      return t('p45.invalidDetection');
    }
  }
  async function request(path, body) {
    const response = await fetch(path, body === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
    const value = await response.json();
    if (!response.ok || value.ok === false) throw new Error(value.error || `HTTP ${response.status}`);
    return value;
  }
  let cancelling = null;
  window.reBotAI = {cancel: async () => {
    if (!window.reBotAIActive) return {success: true};
    if (!cancelling) cancelling = request('/api/llm/cancel', {}).then(result => {
      if (result.success) window.reBotAIActive = false;
      return result;
    }).finally(() => { cancelling = null; });
    return cancelling;
  }};
  function buttons() {
    el('llm-send').disabled = !connected || busy;
    el('llm-confirm').disabled = !connected || busy || !plan;
  }
  async function poll() {
    if (!connected) return;
    try {
      const status = await request('/api/llm/status');
      window.reBotAIActive = executing || status.active || status.blocked;
      el('llm-status').textContent = `${t(status.active ? 'p45.running' : 'p45.connected')} · ${t(status.motion_authorized ? 'p45.authorized' : 'p45.locked')}${status.planner_configured === false ? ' · ' + t('p45.noKey') : ''}`;
    } catch (error) {
      el('llm-status').textContent = error.message;
      // Keep ownership on disconnect until backend reports completion/hold.
    }
  }
  async function init() {
    if (!el('llm-start')) return;
    el('llm-start').onclick = async () => {
      try {
        const health = await request('/api/llm/health');
        if (health.mode !== 'plan-review-v1') throw new Error(t('p45.incompatible'));
        connected = true; generation++; await poll(); buttons();
      } catch (error) { log(error.message); }
    };
    el('llm-stop').onclick = () => {
      connected = false; generation++; plan = null;
      el('llm-status').textContent = t('p45.chatStopped'); buttons();
      // Assistant disconnection does not claim that the robot is stopped.
    };
    el('llm-send').onclick = async () => {
      const text = el('llm-input').value.trim();
      if (!connected || busy || !text) return;
      plan = null; busy = true; buttons(); const epoch = generation;
      try {
        log(text);
        const result = await request('/api/llm/chat', {text});
        if (!connected || epoch !== generation) return;
        if (!result.plan || !Array.isArray(result.plan.steps) || typeof result.plan_id !== 'string') throw new Error(t('p45.incompatible'));
        plan = result;
        el('llm-plan').textContent = JSON.stringify(result.plan, null, 2);
        log(result.text);
      } catch (error) { log(error.message); }
      finally { busy = false; buttons(); }
    };
    el('llm-confirm').onclick = async () => {
      if (!plan || busy || !connected) return;
      if (plan.requires_confirmation && (!el('ros-control-enable').checked || !el('llm-supervised').checked || window.reBotTaskState?.owner || window.reBotPhysicsActive)) {
        log(t('p45.confirmGuard')); return;
      }
      if (!window.confirm(t('p45.confirmPrompt') + '\n' + JSON.stringify(plan.plan))) return;
      const selected = plan; plan = null; busy = true; executing = true; window.reBotAIActive = true; buttons();
      try {
        const result = await request('/api/llm/execute', {plan_id: selected.plan_id, confirmed: true});
        for (const event of result.execution_events || result.events || []) {
          const message = formatEvent(event);
          if (message) log(message);
        }
        log(result.text);
      } catch (error) { log(error.message); }
      finally { busy = false; executing = false; await poll(); buttons(); }
    };
    el('llm-cancel').onclick = async () => {
      plan = null; buttons();
      try { log((await request('/api/llm/cancel', {})).text); } catch (error) { log(error.message); }
    };
    el('mcp-refresh').onclick = async () => {
      try {
        const value = await request('/api/mcp/rpc', {jsonrpc: '2.0', id: 1, method: 'tools/list', params: {}});
        if (value.error) throw new Error(value.error.message);
        el('mcp-tools').replaceChildren();
        for (const tool of value.result.tools) {
          const option = document.createElement('option'); option.value = tool.name; option.textContent = tool.name;
          el('mcp-tools').appendChild(option);
        }
        el('mcp-result').textContent = JSON.stringify(value.result, null, 2);
      } catch (error) { el('mcp-result').textContent = error.message; }
    };
    el('mcp-run').onclick = async () => {
      try {
        const args = JSON.parse(el('mcp-arguments').value);
        const value = await request('/api/mcp/rpc', {jsonrpc: '2.0', id: 2, method: 'tools/call', params: {name: el('mcp-tools').value, arguments: args}});
        el('mcp-result').textContent = JSON.stringify(value, null, 2);
      } catch (error) { el('mcp-result').textContent = error.message; }
    };
    setInterval(poll, 750); buttons();
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init); else init();
})();
