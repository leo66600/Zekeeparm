(function () {
  const client = window.reBotRos;
  if (!client) return;
  const t = window.rebotI18n.t;
  const el = (id) => document.getElementById(id);
  let available = false;
  let lastStatusAt = 0;
  let polling = false;
  let requestBusy = false;
  window.reBotTaskState = null;

  function message(value) { el('hardware-teach-status').textContent = value; }
  function updateClearButton() {
    const state = window.reBotTaskState;
    el('hardware-teach-clear').disabled = !(client.connected && available && Date.now()-lastStatusAt <= 3000
      && state?.owner === 'teach' && ['HOLD', 'GUIDING'].includes(state.state)
      && !state.busy && !state.recording && !requestBusy && !window.reBotAIActive && !window.reBotPhysicsActive
      && (state.points > 0 || state.path > 0));
  }
  function applyStatus(status) {
    if (!status || typeof status.owner !== 'string') return;
    available = true;
    lastStatusAt = Date.now();
    window.reBotTaskState = status;
    updateClearButton();
    message(t('p23.sessionStatus', {state: status.state, phase: status.phase, points: status.points, path: status.path, message: status.message}));
    const select = el('hardware-teach-files');
    const selected = select.value;
    const sessions = Array.isArray(status.sessions) ? status.sessions : [];
    if (JSON.stringify(sessions) !== select.dataset.sessions) {
      select.replaceChildren();
      sessions.forEach((name) => { const option = document.createElement('option'); option.value = name; option.textContent = name; select.appendChild(option); });
      if (sessions.includes(selected)) select.value = selected;
      select.dataset.sessions = JSON.stringify(sessions);
    }
  }

  async function teach(command, name = '') {
    if (requestBusy) return;
    const readOnly = ['status', 'save', 'load', 'capture', 'record', 'clear'].includes(command);
    if (command !== 'status' && (!available || Date.now() - lastStatusAt > 3000)) { message(t('p23.unavailable')); return; }
    if (!readOnly && (!client.connected || !el('ros-control-enable').checked || !window.reBotTaskState.motion_authorized)) {
      message(t('p23.locked')); return;
    }
    if (command !== 'status' && (window.reBotAIActive || window.reBotPhysicsActive)) { message(t('p23.locked')); return; }
    if (command === 'begin' && !el('hardware-teach-supervised').checked) { message(t('p23.supervised')); return; }
    if (command === 'clear') {
      updateClearButton();
      if (el('hardware-teach-clear').disabled || !window.confirm(t('p23.clearConfirm'))) return;
    }
    requestBusy = true;
    updateClearButton();
    try {
      const result = await client.callService('/zekeep/teach/command', 'zekeep_msgs/srv/WebTeachCommand', {
        command, name, supervised: el('hardware-teach-supervised').checked
      });
      applyStatus(JSON.parse(result.status_json));
      if (!result.success) message(result.message);
      else if (command === 'clear') message(t('p23.cleared'));
    } catch (error) { message(error.message); }
    finally { requestBusy = false; updateClearButton(); }
  }

  client.subscribe('/zekeep/web_tasks/status', 'std_msgs/msg/String', (msg) => {
    try { applyStatus(JSON.parse(msg.data)); } catch (_) { message(t('p23.invalidStatus')); }
  }, { throttleRate: 400 });
  client.addEventListener('status', (event) => {
    if (['closed', 'error'].includes(event.detail.state)) {
      available = false;
      updateClearButton();
      // Retain known task ownership across connection loss until the backend reports its state.
      message(t('p23.disconnected'));
    }
  });
  window.reBotTasks = {
    cancel() {
      if (!available && !window.reBotTaskState?.owner) return Promise.resolve({success: true});
      return client.callService('/zekeep/web_tasks/cancel', 'std_srvs/srv/Trigger', {});
    }
  };
  document.querySelectorAll('[data-teach-command]').forEach((button) => button.addEventListener('click', () => teach(button.dataset.teachCommand)));
  el('hardware-teach-save').addEventListener('click', () => teach('save', el('hardware-teach-name').value.trim()));
  el('hardware-teach-load').addEventListener('click', () => teach('load', el('hardware-teach-files').value));
  window.setInterval(async () => {
    updateClearButton();
    if (!client.connected || polling || requestBusy) return;
    if (!available) return; // No optional task backend: avoid repeated unavailable service calls.
    polling = true;
    try { await teach('status'); } finally { polling = false; }
  }, 750);
  updateClearButton();
})();
