(function () {
  'use strict';
  const t = (key, values) => window.rebotI18n ? window.rebotI18n.t(key, values) : key;
  const fields = ['x', 'y', 'z', 'duration'].map(name => document.getElementById(`ros-pose-${name}`));
  const status = document.getElementById('ros-pose-preview-status');
  const toggle = document.getElementById('ros-pose-preview-toggle');
  const client = window.reBotRos;
  if (!status || fields.some(field => !field)) return;
  let target = null, targetKey = '', generation = 0, timer = null;
  let enabled = true;

  function syncToggle() {
    if (!toggle) return;
    const key = enabled ? 'action.previewHide' : 'action.previewShow';
    toggle.setAttribute('aria-pressed', String(enabled));
    toggle.setAttribute('data-i18n', key);
    toggle.textContent = t(key);
  }

  function getTarget() {
    const values = fields.map(field => field.value.trim() === '' ? NaN : Number(field.value));
    if (!values.every(Number.isFinite) || values[3] < .4 || values[3] > 8) return null;
    const current = window.reBotSim?.getTcpPose();
    if (!current) return null;
    const key = values.join(',');
    if (!target || key !== targetKey) {
      targetKey = key;
      target = {pose: {position: {x: values[0], y: values[1], z: values[2]},
        orientation: {...current.orientation}}, duration: values[3]};
    }
    return {pose: {position: {...target.pose.position}, orientation: {...target.pose.orientation}}, duration: target.duration};
  }

  function clear() {
    generation += 1;
    window.clearTimeout(timer);
    window.reBotSim?.clearPosePreview();
    status.textContent = '';
  }

  function refresh() {
    clear();
    if (!enabled) {
      status.textContent = t('action.previewOff');
      return;
    }
    const selected = getTarget();
    if (!selected) {
      status.textContent = t('action.previewInvalid');
      return;
    }
    window.reBotSim.previewPose(selected.pose.position, selected.duration);
    if (!client?.connected) {
      status.textContent = t('action.previewOffline');
      return;
    }
    status.textContent = t('action.previewSolving');
    const epoch = generation;
    timer = window.setTimeout(async () => {
      try {
        const definitions = window.reBotSim.getJointDefs().filter(item => item.name !== 'gripper');
        const names = definitions.map(item => item.name);
        const current = window.reBotSim.getAngles();
        const start = names.map(name => current[name]);
        const result = await client.previewPoseIK(selected.pose, names, start);
        if (epoch !== generation || !client.connected) return;
        const state = result.solution?.joint_state;
        const solution = new Map((state?.name || []).map((name, index) => [name, state.position?.[index]]));
        const angles = names.map(name => solution.get(name));
        if (result.error_code?.val !== 1 || !angles.every((value, index) => Number.isFinite(value)
            && value >= definitions[index].min && value <= definitions[index].max)) {
          status.textContent = t('action.previewNoIK', {code: result.error_code?.val ?? '?'});
          return;
        }
        const velocity = Math.max(.05, Math.min(1.5, Number(document.getElementById('ros-vlim')?.value) || .3));
        const seconds = window.ReBotControlPolicy.computeJointTrajectoryDuration(start, angles, velocity, selected.duration);
        window.reBotSim.previewPose(selected.pose.position, seconds, Object.fromEntries(names.map((name, index) => [name, angles[index]])));
        status.textContent = t('action.previewReady', {sec: seconds.toFixed(1)});
      } catch (error) {
        if (epoch === generation) status.textContent = t('action.previewFailed', {error: error.message});
      }
    }, 250);
  }

  fields.forEach(field => {
    field.addEventListener('input', refresh);
    field.addEventListener('change', refresh);
  });
  document.getElementById('ros-vlim')?.addEventListener('input', refresh);
  toggle?.addEventListener('click', () => {
    enabled = !enabled;
    syncToggle();
    refresh();
  });
  window.rebotI18n?.onLangChange?.(() => {
    syncToggle();
    if (!enabled) status.textContent = t('action.previewOff');
  });
  window.addEventListener('rebot-model-ready', () => {target = null; refresh();});
  client?.addEventListener('status', () => {target = null; refresh();});
  window.reBotPosePreview = {getTarget, refresh, clear};
  syncToggle();
  refresh();
})();
