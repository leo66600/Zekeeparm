(function () {
const NS = 'zekeep';
 const t = window.rebotI18n ? window.rebotI18n.t : (k) => k;
 const controlPolicy = window.ReBotControlPolicy;
 const URL_STORAGE_KEY = 'zekeep.ros.url';
  function loadSavedUrl() { try { return localStorage.getItem(URL_STORAGE_KEY) || ''; } catch (_) { return ''; } }
  function saveUrl(url) { try { localStorage.setItem(URL_STORAGE_KEY, url); } catch (_) {} }
 const OPEN_GRIPPER_M = 0.07 * 1.35 / 1.45;
  const CLOSE_GRIPPER_M = 0;
  let gripperMapping = null;
  const GRIPPER_BASE_GAP_M = 0.014;
  const GRIPPER_VISUAL_TRAVEL_M = 0.057;
  const GRIPPER_EFFECTIVE_GAP_M = OPEN_GRIPPER_M;
  const GRASP_SQUEEZE_M = 0.004;
  const MIN_OBJECT_GRASP_M = 0.018;
  const VISION_TRANSIT_Z_M = 0.32;
  const VISION_TRANSIT_Z_BY_COLOR_M = {
    blue: 0.410
  };
  const VISION_FIRST_LIFT_CLEARANCE_M = 0.085;
  const VISION_FIRST_LIFT_MIN_M = 0.275;
  const VISION_POSE_SKIP_M = 0.006;
  const VISION_VERTICAL_ALIGN_CLEARANCE_M = 0.075;
  const VISION_PREGRASP_CLEARANCE_M = 0.038;
  const VISION_MIN_VERTICAL_ALIGN_Z_M = 0.235;
  const VISION_FIRST_LIFT_MIN_BY_COLOR_M = {
    blue: 0.390
  };
  const JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
  const REQUIRED_TOPICS = {
    jointStates: `/${NS}/joint_states`,
    armStatus: `/${NS}/arm_status`,
    gripper: `/${NS}/gripper/state`,
    cameraImage: '/camera/color/image_raw',
    objectStates: `/${NS}/mujoco/object_states`,
    visionDetections: `/${NS}/vision/color_blocks/detections`,
    simAnimation: `/${NS}/sim/animation_event`
  };
  const CAMERA_IMAGE_TOPICS = [
    REQUIRED_TOPICS.cameraImage,
    '/camera/color/image',
    '/zekeep/web/camera/image_raw',
    '/gemini305g/color/image_raw',
    `/${NS}/mujoco/overhead_rgb/image_raw`
  ];
  const REQUIRED_SERVICES = {
    gravityStart: `/${NS}/gravity_compensation/start`,
    gravityStop: `/${NS}/gravity_compensation/stop`,
    gravityStatus: `/${NS}/gravity_compensation/status`,
    recordStart: `/${NS}/mujoco/record/start`,
    recordStop: `/${NS}/mujoco/record/stop`,
    recordReplay: `/${NS}/mujoco/record/replay`,
    recordClear: `/${NS}/mujoco/record/clear`
  };

  const els = {
    url: document.getElementById('ros-url'),
    connect: document.getElementById('ros-connect'),
    disconnect: document.getElementById('ros-disconnect'),
    safeDisconnect: document.getElementById('ros-safe-disconnect'),
    mirror: document.getElementById('ros-mirror'),
    control: document.getElementById('ros-control-enable'),
    status: document.getElementById('ros-status'),
    message: document.getElementById('ros-message'),
    feedbackError: document.getElementById('ros-feedback-error'),
    enable: document.getElementById('ros-enable'),
    disable: document.getElementById('ros-disable'),
    safeHome: document.getElementById('ros-safe-home'),
    gravityStatus: document.getElementById('ros-gravity-status'),
    gravityStart: document.getElementById('ros-gravity-start'),
    gravityStop: document.getElementById('ros-gravity-stop'),
    gravityQuery: document.getElementById('ros-gravity-status-query'),
   rosOpenGripper: document.getElementById('ros-open-gripper'),
   closeGripper: document.getElementById('ros-close-gripper'),
   clearLog: document.getElementById('ros-clear-log'),
   log: document.getElementById('ros-log'),
   cameraCanvas: document.getElementById('ros-camera-canvas'),
    cameraStatus: document.getElementById('ros-camera-status'),
    cameraTopic: document.getElementById('ros-camera-topic'),
    visionStatus: document.getElementById('ros-vision-status'),
    visionTarget: document.getElementById('ros-vision-target'),
    visionColor: document.getElementById('ros-vision-color'),
    visionApproachZ: document.getElementById('ros-vision-approach-z'),
    visionGraspZ: document.getElementById('ros-vision-grasp-z'),
    visionFillPose: document.getElementById('ros-vision-fill-pose'),
    visionMoveAbove: document.getElementById('ros-vision-move-above'),
    visionPickDemo: document.getElementById('ros-vision-pick-demo'),
    visionPlaceDemo: document.getElementById('ros-vision-place-demo'),
    materialX: document.getElementById('ros-material-x'),
    materialY: document.getElementById('ros-material-y'),
    materialZ: document.getElementById('ros-material-z'),
    materialApproach: document.getElementById('ros-material-approach'),
    materialWidth: document.getElementById('ros-material-width'),
    materialPick: document.getElementById('ros-material-pick'),
    vlim: document.getElementById('ros-vlim'),
    trajectoryDuration: document.getElementById('ros-trajectory-duration'),
    poseX: document.getElementById('ros-pose-x'),
    poseY: document.getElementById('ros-pose-y'),
    poseZ: document.getElementById('ros-pose-z'),
    poseDuration: document.getElementById('ros-pose-duration'),
   checkIk: document.getElementById('ros-check-ik'),
   stopPath: document.getElementById('stop-path')
  };

 if (!window.ReBotRosClient || !els.connect) return;

  if (els.url && !els.url.value) {
    const saved = loadSavedUrl();
    els.url.value = saved || 'ws://127.0.0.1:9090';
  }
  const client = new window.ReBotRosClient({ namespace: NS, url: els.url ? els.url.value : '' });
 window.reBotRos = client;

  const lastSent = new Map();
  const simTargetAngles = new Map();
  const mirrorHoldUntil = new Map();
  const COMMAND_INTERVAL_MS = 45;
  const THROTTLED_COMMAND_SOURCES = new Set(['drag', 'trajectory']);
 const MIRROR_HOLD_MS = 1800;
 let latestJointPositions = null;
  let latestJointStateAt = 0;
  let latestGripperPosition = null;
  let latestGripperVelocity = null;
  let latestGripperAt = 0;
  let listedTopics = new Set();
  let listedServices = new Set();
  let listedActionServers = new Set();
  let simulationDriverDetected = false;
  let latestVisionPayload = null;
  let latestVisionAt = 0;
  let selectedVisionTarget = null;
  let lastVisionTarget = null;
  let heldVisionTarget = null;
  let autoVisionTargetColor = '';
  let visionSequenceBusy = false;
  let lastVisionOp = null;
  let safeDisconnectBusy = false;
  let gravityCompensationActive = false;
  let lastStatusState = null;
  let gravityStatusSource = 'initial';
  let gravityStatusPollInFlight = false;
  let activeCameraTopic = REQUIRED_TOPICS.cameraImage;
  let latestArmEnabled = false;
  let latestControlLoopActive = false;
  let trajectoryActionBusy = false;
  let gripperCommandBusy = false;
  let motionEpoch = 0;
  let stopping = false;
  let stopFailed = false;
  let teachingHardwareBusy = false;
  const executionStatus = document.getElementById('ros-execution-status');
  const feedbackSource = document.getElementById('ros-feedback-source');
  const mappingStatus = document.getElementById('ros-gripper-mapping');

  function setExecution(key, params) {
    if (executionStatus) executionStatus.textContent = t(key, params);
  }

  function invalidateMotion() {
    motionEpoch += 1;
    window.reBotPosePreview?.clear();
    if (window.reBotAI) window.reBotAI.cancel().catch((error) => writeLog(error.message, "error"));
    if (window.reBotSim) window.reBotSim.stopLocalMotion();
  }

  async function stopRobot() {
    invalidateMotion();
    if (stopping) return;
    if (!client.connected) {
      setExecution('p01.stopUnconfirmed');
      return;
    }
    stopping = true;
    client.cancelActions();
    setExecution('p01.stopping');
    try {
      const [taskResult, result, aiResult] = await Promise.all([
        window.reBotTasks ? window.reBotTasks.cancel() : Promise.resolve({success: true}),
        client.stop(),
        window.reBotAI ? window.reBotAI.cancel() : Promise.resolve({success: true})
      ]);
      if (!aiResult || aiResult.success !== true) throw new Error(t('p01.stopUnconfirmed'));
      if (!taskResult || taskResult.success !== true) throw new Error(taskResult && taskResult.message || t('p01.stopUnconfirmed'));
      if (!result || result.success !== true) throw new Error(result && result.message || t('p01.stopUnconfirmed'));
      stopFailed = false;
      setExecution('p01.stopped');
      writeLog(result.message || t('p01.stopped'), 'ok');
    } catch (error) {
      stopFailed = true;
      setExecution('p01.stopUnconfirmed');
      writeLog(error.message, 'error');
    } finally {
      stopping = false;
    }
  }

  async function loadGripperMapping() {
    const epoch = motionEpoch;
    try {
      const result = await client.getGripperMapping();
      if (epoch !== motionEpoch || !client.connected) return;
      const values = result.values || [];
      const [open, close, width] = values.map((value) => value.type === 3 ? value.double_value : NaN);
      if (values.length !== 3 || ![open, close, width].every(Number.isFinite)
          || open <= close || width <= 0 || width > OPEN_GRIPPER_M) throw new Error(t('p01.mappingUnavailable'));
      gripperMapping = { open, close, width };
      if (mappingStatus) mappingStatus.textContent = t('p01.mapping', { open, close, mm: width * 1000 });
    } catch (error) {
      if (epoch !== motionEpoch) return;
      gripperMapping = null;
      if (mappingStatus) mappingStatus.textContent = t('p01.mappingUnavailable');
      writeLog(error.message, 'warn');
    }
  }

  document.getElementById('ros-stop')?.addEventListener('click', stopRobot);
  document.getElementById('ros-mapping-query')?.addEventListener('click', loadGripperMapping);
  client.addEventListener('action-feedback', () => setExecution('p01.running'));
  client.addEventListener('action-timeout', stopRobot);

  client.subscribe(REQUIRED_TOPICS.jointStates, 'sensor_msgs/msg/JointState', handleJointStates, { throttleRate: 80 });
  client.subscribe(REQUIRED_TOPICS.gripper, 'zekeep_msgs/msg/JointMotorState', handleGripperState, { throttleRate: 80 });
  client.subscribe(REQUIRED_TOPICS.armStatus, 'zekeep_msgs/msg/ArmStatus', handleArmStatus, { throttleRate: 200 });
  CAMERA_IMAGE_TOPICS.filter((topic) => els.cameraCanvas || topic.includes('/mujoco/')).forEach((topic) => {
    client.subscribe(topic, 'sensor_msgs/msg/Image', handleCameraImage, { throttleRate: 250 });
  });
  client.subscribe(REQUIRED_TOPICS.objectStates, 'std_msgs/msg/String', handleMujocoObjectStates, { throttleRate: 33 });
  client.subscribe(REQUIRED_TOPICS.visionDetections, 'std_msgs/msg/String', handleVisionDetections, { throttleRate: 180 });
  client.subscribe(REQUIRED_TOPICS.simAnimation, 'std_msgs/msg/String', handleSimAnimationEvent, { throttleRate: 0 });
  if (els.cameraTopic) els.cameraTopic.textContent = activeCameraTopic;

  client.addEventListener('status', (event) => {
    const detail = event.detail || {};
    setStatus(detail.state, detail.message);
    if (detail.state !== 'connecting') {
      writeLog(detail.message || detail.state, detail.state === 'error' ? 'error' : detail.state === 'open' ? 'ok' : 'info');
    }
    updateDiagnostics();
   if (detail.state === 'closed' || detail.state === 'error') {
      invalidateMotion();
      latestJointPositions = null;
      latestJointStateAt = 0;
      latestArmEnabled = false;
      latestControlLoopActive = false;
      gripperMapping = null;
      els.control.checked = false;
      setExecution('p01.disconnected');
      listedTopics = new Set();
      listedServices = new Set();
      listedActionServers = new Set();
      simulationDriverDetected = false;
      updateGravityStatus(false, t('msg.rosNotConnected'), 'connection');
      if (detail.state === 'error' && client.connected) stopRobot();
   }
    if (detail.state === 'open') {
      loadGripperMapping();
      window.setTimeout(() => {
        runDiagnostics();
      }, 250);
    }
  });

  els.connect.addEventListener('click', () => {
   const nextUrl = els.url.value.trim() || 'ws://127.0.0.1:9090';
   els.url.value = nextUrl;
   if (!canConnectWebSocketUrl(nextUrl)) return;
    saveUrl(nextUrl);
   client.autoReconnect = true;
    client.connect(nextUrl);
  });
  els.disconnect.addEventListener('click', disconnectRos);
  window.addEventListener('pagehide', () => {
    client.autoReconnect = false;
    if (client.socket) client.socket.close();
  });
  els.enable.addEventListener('click', () => guardedCall(
    () => client.enable(),
    t('msg.reqEnable'),
    false,
    { allowDisabled: true }
  ));
  els.disable.addEventListener('click', () => {
    invalidateMotion();
    client.cancelActions();
    if (window.reBotTasks) window.reBotTasks.cancel().catch((error) => writeLog(error.message, 'error'));
    guardedCall(() => client.disable(), t('msg.reqDisable'), true);
  });
  els.safeHome.addEventListener('click', () => {
    guardedCall(() => client.safeHome(), t('msg.reqSafeHome'), false, { allowLowLevel: true });
  });
  els.gravityStart.addEventListener('click', () => {
    guardedOptionalService(
      REQUIRED_SERVICES.gravityStart,
      () => client.startGravityCompensation(),
      t('msg.reqGravityStart')
    );
  });
  els.gravityStop.addEventListener('click', () => {
    guardedOptionalService(
      REQUIRED_SERVICES.gravityStop,
      () => client.stopGravityCompensation(),
      t('msg.reqGravityStop'),
      true
    );
  });
  els.gravityQuery.addEventListener('click', queryGravityCompensation);
 els.rosOpenGripper.addEventListener('click', () => sendGripper(OPEN_GRIPPER_M, { requireControl: true }));
 els.closeGripper.addEventListener('click', () => sendGripper(CLOSE_GRIPPER_M, { requireControl: true }));
 els.clearLog.addEventListener('click', () => { els.log.innerHTML = ''; });
  els.checkIk.addEventListener('click', checkIk);
  document.getElementById('tcp-execute')?.addEventListener('click', checkIk);
  document.getElementById('ros-help-top')?.addEventListener('click', () => document.getElementById('ros-help-dialog')?.showModal());
  document.getElementById('ros-help-close')?.addEventListener('click', () => document.getElementById('ros-help-dialog')?.close());
  const sidebar = document.querySelector('.control-panel');
  const appShell = document.querySelector('.app-shell');
  const collapseBtn = document.getElementById('sidebar-collapse');
  collapseBtn?.addEventListener('click', () => {
    if (!sidebar) return;
    sidebar.classList.toggle('collapsed');
    const collapsed = sidebar.classList.contains('collapsed');
    collapseBtn.textContent = collapsed ? '▶' : '◀';
    collapseBtn.title = collapsed ? t('panel.expand') : t('panel.collapse');
 });
 if (els.visionColor) els.visionColor.addEventListener('change', () => {
    if (els.visionColor.value === 'auto') autoVisionTargetColor = '';
    updateSelectedVisionTarget();
  });
  if (els.visionFillPose) els.visionFillPose.addEventListener('click', fillPoseFromVisionTarget);
  if (els.visionMoveAbove) els.visionMoveAbove.addEventListener('click', moveAboveVisionTarget);
  if (els.visionPickDemo) els.visionPickDemo.addEventListener('click', runVisionPickDemo);
  if (els.visionPlaceDemo) els.visionPlaceDemo.addEventListener('click', runVisionPlaceDemo);
  if (els.materialPick) els.materialPick.addEventListener('click', runMaterialPick);
  if (els.stopPath) {
    els.stopPath.addEventListener('click', () => {
      stopRobot();
    });
  }

  els.control.addEventListener('change', () => {
    if (els.control.checked && window.reBotSim && window.reBotSim.isRecording()) {
      els.control.checked = false;
      setMessage(t('p01.teachHint'));
      return;
    }
    if (els.control.checked) writeLog(t('log.controlLockOpen'), 'info');
    else stopRobot();
  });

  waitForSimApi((sim) => sim.onCommand((command) => forwardSimCommand(command)));

  setStatus('closed', t('msg.rosNotConnected'));
  updateDiagnostics();
  window.setInterval(updateDiagnostics, 1000);
  window.setInterval(pollGravityCompensationStatus, 500);

  function handleJointStates(msg) {
    if (!Array.isArray(msg.name) || !Array.isArray(msg.position)) return;
    const next = {};
    msg.name.forEach((name, index) => {
      const simName = normalizeJointName(name);
      if (!simName || !Number.isFinite(msg.position[index])) return;
      next[simName] = msg.position[index];
    });

    if (JOINT_NAMES.every((name) => Number.isFinite(next[name]))) {
      latestJointPositions = next;
      latestJointStateAt = performance.now();
    }
    updateFeedbackError(next);

    if (Object.keys(next).length) {
      const mirrored = {};
      const now = performance.now();
      Object.entries(next).forEach(([name, value]) => {
        // The hardware publishes a single finger travel in /joint_states.  The
        // simulator's `gripper` joint represents the complete opening and must
        // drive both fingers symmetrically, so do not write either URDF finger
        // joint directly here.
        if (name === 'gripper_joint' || name === 'right_joint') return;
        const holdUntil = mirrorHoldUntil.get(name) || 0;
        const target = simTargetAngles.get(name);
        const reachedTarget = typeof target === 'number' && Math.abs(target - value) < 0.025;
        if (reachedTarget || now > holdUntil) {
          mirrorHoldUntil.delete(name);
          mirrored[name] = value;
        }
      });

      const leftOpening = Number(next.gripper_joint);
      const rightOpening = Number(next.right_joint);
      const fingerOpening = Number.isFinite(leftOpening)
        ? leftOpening
        : (Number.isFinite(rightOpening) ? rightOpening : NaN);
      if (Number.isFinite(fingerOpening)) {
        const gripperWidth = fingerOpeningToGripperCommand(fingerOpening);
        const holdUntil = mirrorHoldUntil.get('gripper') || 0;
        const target = simTargetAngles.get('gripper');
        const reachedTarget = typeof target === 'number' && Math.abs(target - gripperWidth) < 0.003;
        if (reachedTarget || now > holdUntil) {
          mirrorHoldUntil.delete('gripper');
          mirrored.gripper = gripperWidth;
        }
      }
      if (
        Object.keys(mirrored).length
        && window.reBotSim
        && controlPolicy.shouldMirrorFeedback({
          connected: client.connected,
          mirrorEnabled: Boolean(els.mirror && els.mirror.checked)
        })
      ) window.reBotSim.setAngles(mirrored, { source: 'ros' });
    }
    updateDiagnostics();
  }

  function handleGripperState(msg) {
    if (!gripperMapping || !Number.isFinite(msg.position)) return;
    if (typeof msg.position === 'number') {
      latestGripperPosition = gripperMotorPositionToWidth(msg.position);
      latestGripperAt = performance.now();
    }
    if (typeof msg.velocity === 'number') {
      latestGripperVelocity = msg.velocity * gripperMapping.width / (gripperMapping.open - gripperMapping.close);
    }
    if (window.reBotSim && typeof msg.position === 'number') {
      const width = gripperMotorPositionToWidth(msg.position);
      const holdUntil = mirrorHoldUntil.get('gripper') || 0;
      const target = simTargetAngles.get('gripper');
      const reachedTarget = typeof target === 'number' && Math.abs(target - width) < 0.003;
      if (
        (reachedTarget || performance.now() > holdUntil)
        && controlPolicy.shouldMirrorFeedback({
          connected: client.connected,
          mirrorEnabled: Boolean(els.mirror && els.mirror.checked)
        })
      ) {
        mirrorHoldUntil.delete('gripper');
        window.reBotSim.setGripperWidth(width, { source: 'ros', animate: false });
      }
    }
    if (!visionSequenceBusy && typeof msg.position === 'number' && simTargetAngles.has('gripper')) {
      const target = simTargetAngles.get('gripper');
      const width = gripperMotorPositionToWidth(msg.position);
      const err = Math.abs(target - width);
     if (err < 0.003) {
        setMessage(t('msg.gripperArrived', {mm: Math.round(width * 1000)}));
     } else {
        setMessage(t('msg.gripperMoving', {cmd: Math.round(target * 1000), mm: Math.round(width * 1000)}));
     }
    }
    updateDiagnostics();
  }

  function handleArmStatus(msg) {
    latestArmEnabled = Boolean(msg.enabled);
    latestControlLoopActive = Boolean(msg.control_loop_active);
    const enabled = msg.enabled ? t('st.enabled') : t('st.disabled');
    const mode = msg.mode || 'unknown';
   const machine = msg.state_machine || 'unknown';
    const errors = Array.isArray(msg.error_codes) && msg.error_codes.length ? t('fb.errors', {codes: msg.error_codes.join(', ')}) : '';
   if (!visionSequenceBusy) {
      setMessage(t('fb.armStatus', {enabled, mode, machine, errors}));
   }
    updateGravityStatus(machine === 'GRAVITY_COMP', machine, 'arm');
    updateDiagnostics();
  }

  function forwardSimCommand(command) {
    if (command && command.type === 'teaching-replay') {
      stageTeachingTrajectory(command);
      return;
    }
    if (window.reBotSim && window.reBotSim.isRecording()) return;
    if (command && command.type === 'tcp-target') {
      if (command.target_ros) {
        els.poseX.value = command.target_ros.x.toFixed(4);
        els.poseY.value = command.target_ros.y.toFixed(4);
        els.poseZ.value = command.target_ros.z.toFixed(4);
      }
      return;
    }
    if (command && command.type === 'joint-batch') {
      forwardJointBatch(command);
      return;
    }
    if (!command || command.type !== 'joint') return;
    if (['drag', 'drag-settle', 'solver', 'sim', 'preset-preview', 'teach-replay'].includes(command.source)) return;
    if (teachingHardwareBusy) return;
    simTargetAngles.set(command.name, command.value);
    mirrorHoldUntil.set(command.name, performance.now() + MIRROR_HOLD_MS);

    if (els.mirror.checked && !els.control.checked && command.source === 'slider') {
      els.mirror.checked = false;
      writeLog(t('log.mirrorPaused'), 'warn');
    }

    if (!controlAllowed(false, { allowLowLevel: true })) {
      writeLog(t('log.commandBlockedNotReady'), 'warn');
      return;
    }
    // Slider drag only previews. The release event sends one command.
    if (command.source === 'slider') return;

    const now = performance.now();
    const last = lastSent.get(command.name) || 0;
    if (THROTTLED_COMMAND_SOURCES.has(command.source) && now - last < COMMAND_INTERVAL_MS) {
      writeLog(t('log.commandThrottled', {name: command.name}), 'warn');
      return;
    }
    lastSent.set(command.name, now);

    if (command.name === 'gripper') {
      publishGripper(command.value);
      return;
    }
    sendSingleJointTrajectory(command.name, command.value);
  }

  async function sendSingleJointTrajectory(name, position) {
    const index = JOINT_NAMES.indexOf(name);
    if (index < 0) return;
    const current = getCurrentRosPositions();
    const target = current.slice();
    target[index] = position;
    const duration = controlPolicy.computeJointTrajectoryDuration(
      current,
      target,
      getVlim()
    );
    await sendTrajectory([
      makeTrajectoryPoint(current, 0.05),
      makeTrajectoryPoint(target, duration)
    ], `关节 ${name}`);
  }

  async function forwardJointBatch(command) {
    const epoch = motionEpoch;
    if (teachingHardwareBusy) return;
    const joints = command && command.joints && typeof command.joints === 'object' ? command.joints : {};
    const names = [...JOINT_NAMES, 'gripper'].filter((name) => typeof joints[name] === 'number' && Number.isFinite(joints[name]));
    if (!names.length) return;

    const holdUntil = performance.now() + MIRROR_HOLD_MS;
    names.forEach((name) => {
      simTargetAngles.set(name, joints[name]);
      mirrorHoldUntil.set(name, holdUntil);
    });

    if (!controlAllowed(false)) return;

    const label = command.label || command.source || t('log.batchDefault');
    const armNames = names.filter((name) => JOINT_NAMES.includes(name));
    if (armNames.length) {
      const current = getCurrentRosPositions();
      const target = controlPolicy.buildCoordinatedJointTarget(joints, JOINT_NAMES, current);
      const duration = controlPolicy.computeJointTrajectoryDuration(
        current,
        target,
        getVlim(),
        getTrajectoryDuration()
      );
      const points = [
        makeTrajectoryPoint(current, 0.05),
        makeTrajectoryPoint(target, duration)
      ];
      const result = await sendTrajectory(points, label);
      if (!result || result.success === false || epoch !== motionEpoch) return;
    }
    if (names.includes('gripper') && controlAllowed(false)) {
      await publishGripper(joints.gripper);
    }
    writeLog(t('log.jointBatch', {label, n: names.length}), 'ok');
 }

  async function checkIk() {
    if (teachingHardwareBusy) return;
    if (![els.poseX, els.poseY, els.poseZ].every((el) => el.value.trim() !== '' && Number.isFinite(Number(el.value)))) {
      setMessage(t('p01.invalidPose'));
      return;
    }
    const previewTarget = window.reBotPosePreview?.getTarget();
    if (window.reBotPosePreview && !previewTarget) {
      setMessage(t('p01.invalidPose'));
      return;
    }
    const pose = previewTarget ? previewTarget.pose : readPose();
    const duration = previewTarget ? previewTarget.duration : getPoseDuration();
    const hardwareControl = client.connected && els.control && els.control.checked;
    if (!hardwareControl) {
      if (!window.reBotSim || typeof window.reBotSim.moveToTcp !== 'function') {
        setMessage('仿真模型未加载');
        return;
      }
      window.reBotPosePreview?.clear();
      window.reBotSim.moveToTcp(pose.position, '仿真 Pose');
      setMessage('仿真 Pose 运动已执行');
      writeLog(`仿真 Pose: X=${pose.position.x.toFixed(3)} Y=${pose.position.y.toFixed(3)} Z=${pose.position.z.toFixed(3)}`, 'ok');
      return;
    }
    if (!controlAllowed(true)) return;
    window.reBotPosePreview?.clear();
    const result = await moveToPoseViaIkTrajectory(
      pose,
      duration,
      t('msg.reqIkMove', {sec: duration.toFixed(1)})
    );
    if (result && result.success === false) {
      writeLog(result.message || '实机 IK 执行失败', 'error');
    }
  }

  async function queryGravityCompensation(options) {
    const result = await guardedOptionalService(
      REQUIRED_SERVICES.gravityStatus,
      () => client.gravityCompensationStatus(),
      t('msg.reqGravityQuery'),
      true,
      options
    );
    if (result) updateGravityStatus(Boolean(result.success), result.message || '', 'service');
  }

  async function pollGravityCompensationStatus() {
    if (
      !client.connected ||
      !gravityCompensationActive ||
      gravityStatusPollInFlight
    ) return;

    gravityStatusPollInFlight = true;
    try {
      await queryGravityCompensation({ silent: true });
    } finally {
      gravityStatusPollInFlight = false;
    }
  }

  async function runDiagnostics() {
    updateDiagnostics();
    if (!client.connected) {
      writeLog(t('log.rosOfflineFirst'), 'warn');
      return;
    }
    try {
      const [topics, services, actions] = await Promise.all([
        client.getRosTopics(),
        client.getRosServices(),
        client.getRosActionServers()
      ]);
      const topicList = topics.topics || [];
      const serviceList = services.services || [];
      const actionList = actions.action_servers || [];
      listedTopics = new Set(topicList);
      listedServices = new Set(serviceList);
      listedActionServers = new Set(actionList);
      const hasSimulationTopics = controlPolicy.hasSimulationEvidence(topicList);
      writeLog(
        `rosapi: ${topicList.length} topics, ${serviceList.length} services, ${actionList.length} actions`,
        'ok'
      );
      if (els.visionStatus && !topicList.includes(REQUIRED_TOPICS.visionDetections)) {
        els.visionStatus.textContent = t('st.waitNode');
      }
      if (!listedServices.has(REQUIRED_SERVICES.gravityStatus)) {
        updateGravityStatus(false, t('st.serviceUnavailable'));
      }
      if (hasSimulationTopics) markSimulationDriverDetected(t('reason.rosapiSim'));
      if (!hasActionServer(`/${NS}/follow_joint_trajectory`)) {
        writeLog(t('log.lowLevelFallbackInfo'), 'info');
      }
   } catch (error) {
      writeLog(t('log.rosapiFallback', {err: error.message || error}), 'warn');
   }
  }

  async function sendTrajectory(points, optimisticMessage) {
    const epoch = motionEpoch;
    if (!points.length || !controlAllowed(true)) return { success: false };
    if (trajectoryActionBusy) {
      setMessage(t('p01.busy'));
      return { success: false, message: t('p01.busy') };
    }
    trajectoryActionBusy = true;
    setExecution('p01.running');
    try {
      const result = await guardedCall(() => client.followJointTrajectory(JOINT_NAMES, points), optimisticMessage);
      if (epoch !== motionEpoch) return { success: false, message: t('p01.cancelled') };
      const success = Boolean(result && result.completed && !serviceResultFailed(result));
      setExecution(success ? 'p01.completed' : 'p01.failed');
      // An action timeout or transport loss does not prove the robot stopped.
      if (!success && client.connected) await stopRobot();
      return { ...result, success };
    } finally {
      trajectoryActionBusy = false;
    }
  }

  async function stageTeachingTrajectory(command) {
    if (teachingHardwareBusy || !controlAllowed(true)) return;
    const epoch = motionEpoch;
    const waypoints = command.waypoints || [];
    if (!waypoints.length) return;
    teachingHardwareBusy = true;
    window.reBotSim.setTeachingStatus(t('p01.hardwareReplay'));
    try {
      // Revalidate even browser-recorded trajectories at the execution boundary.
      controlPolicy.validateTeachingPayload(window.reBotSim.getTeachingPayload(), window.reBotSim.getJointDefs());
      if (!gripperMapping) throw new Error(t('p01.mappingUnavailable'));
      const span = Math.max(1, waypoints[waypoints.length - 1].t - waypoints[0].t);
      for (let index = 0; index < waypoints.length; index += 1) {
        if (epoch !== motionEpoch || !controlAllowed(false)) throw new Error(t('p01.cancelled'));
        const current = getCurrentRosPositions();
        const target = JOINT_NAMES.map((name) => waypoints[index].joints[name]);
        const requested = index ? (waypoints[index].t - waypoints[index - 1].t) / span * getTrajectoryDuration() : 1;
        const duration = controlPolicy.computeJointTrajectoryDuration(current, target, getVlim(), requested);
        const result = await sendTrajectory([makeTrajectoryPoint(current, 0.05), makeTrajectoryPoint(target, duration)], t('p01.hardwareReplay'));
        if (!result.success || epoch !== motionEpoch) throw new Error(result.message || t('p01.failed'));
        // Each endpoint's gripper command completes before advancing to the next arm segment.
        const grip = await publishGripper(waypoints[index].joints.gripper);
        if (!grip || grip.success !== true || epoch !== motionEpoch) throw new Error(t('p01.failed'));
      }
      window.reBotSim.setTeachingStatus(t('p01.completed'));
    } catch (error) {
      window.reBotSim.setTeachingStatus(error.message);
      writeLog(error.message, 'error');
      if (epoch === motionEpoch && client.connected) await stopRobot();
    } finally {
      teachingHardwareBusy = false;
    }
  }

  function markSimulationDriverDetected(reason) {
    if (simulationDriverDetected) return;
   simulationDriverDetected = true;
    writeLog(t('log.simDriverDetected', {reason}), 'info');
 }

  function hasActionServer(actionName) {
    return listedActionServers.has(actionName);
  }

  function makeTrajectoryPoint(positions, seconds) {
    return {
      positions,
      velocities: JOINT_NAMES.map(() => 0),
      accelerations: [],
      effort: [],
      time_from_start: secondsToRosTime(seconds)
    };
  }

  function getCurrentRosPositions() {
    const source = latestJointPositions || (window.reBotSim && window.reBotSim.getAngles ? window.reBotSim.getAngles() : {});
    return JOINT_NAMES.map((name) => Number(source[name] || 0));
  }

  function readPose() {
    const tcpPose = window.reBotSim && typeof window.reBotSim.getTcpPose === 'function'
      ? window.reBotSim.getTcpPose()
      : null;
    return {
      position: {
        x: Number(els.poseX.value) || 0,
        y: Number(els.poseY.value) || 0,
        z: Number(els.poseZ.value) || 0
      },
      // The UI exposes position-only targets. Preserve the current TCP
      // orientation so the controller's full-pose IK is not forced to solve
      // an arbitrary identity orientation that is unreachable for link6.
      orientation: tcpPose && tcpPose.orientation
        ? tcpPose.orientation
        : { x: 0, y: 0, z: 0, w: 1 }
    };
  }

  function controlAllowed(interactive, options) {
    if ((window.reBotTaskState && window.reBotTaskState.owner) || window.reBotAIActive || window.reBotPhysicsActive) {
      if (interactive) setMessage(t('p23.taskOwnsControl'));
      return false;
    }
    if (!client.connected) {
      if (interactive) setStatus('closed', t('msg.rosNotConnected'));
      return false;
    }
    if (!els.control.checked) {
      if (interactive) setMessage(t('msg.controlLockClosed'));
      return false;
    }
    if (!(options && options.allowDisabled) && (stopping || stopFailed || !latestJointStateAt
        || performance.now() - latestJointStateAt > 2500
        || !latestJointPositions || !JOINT_NAMES.every((name) => Number.isFinite(latestJointPositions[name])))) {
      if (interactive) setMessage(t('msg.feedbackStale'));
      return false;
    }
    const allowLowLevel = Boolean(options && options.allowLowLevel);
    if (!(options && options.allowDisabled) && (!latestArmEnabled || (!allowLowLevel && !latestControlLoopActive))) {
      if (interactive) setMessage(t('msg.armNotReady'));
      return false;
    }
    return true;
  }

  function canConnectWebSocketUrl(url) {
    if (window.location.protocol === 'https:' && /^ws:\/\//i.test(url)) {
      const message = t('msg.httpsWsBlocked');
      setStatus('error', message);
      writeLog(message, 'error');
      return false;
    }
    return true;
  }

  async function guardedOptionalService(serviceName, call, optimisticMessage, allowWithoutControl, options) {
   if (listedServices.size && !listedServices.has(serviceName)) {
      const message = t('msg.serviceNotFound', {name: serviceName});
     updateGravityStatus(false, t('st.serviceUnavailable'));
      setMessage(message);
      if (!(options && (options.auto || options.silent))) writeLog(message, 'warn');
      return null;
    }
    return guardedCall(call, optimisticMessage, allowWithoutControl, {
      keepConnectionStatus: true,
      silent: Boolean(options && options.silent)
    });
  }

  async function disconnectRos() {
    if (safeDisconnectBusy) return;
    invalidateMotion();
    client.cancelActions();

    if (!els.safeDisconnect || !els.safeDisconnect.checked || !client.connected) {
      client.disconnect();
      return;
    }

    safeDisconnectBusy = true;
    els.disconnect.disabled = true;
    try {
      setMessage(t('msg.disconnectHome'));
      writeLog(t('log.disconnectHomeStart'), 'info');
      const homeResult = await client.safeHome();
      if (homeResult && homeResult.success === false) {
        throw new Error(homeResult.message || t('msg.safeHomeFail'));
      }
      writeLog(t('log.disconnectHomeDone'), 'ok');

      setMessage(t('msg.disconnectHold'));
      writeLog(t('log.disconnectHold'), 'ok');
      client.disconnect();
   } catch (error) {
      const message = t('msg.disconnectGuardFail', {err: error && error.message ? error.message : error});
     setMessage(message);
      writeLog(message, 'error');
    } finally {
      safeDisconnectBusy = false;
      els.disconnect.disabled = false;
    }
  }

  async function guardedCall(call, optimisticMessage, allowWithoutControl, options) {
    if (!client.connected) {
      setStatus('closed', t('msg.rosNotConnected'));
      return null;
    }
    const allowDisabled = options && options.allowDisabled;
    const allowLowLevel = options && options.allowLowLevel;
    if (!allowWithoutControl && !controlAllowed(true, { allowDisabled, allowLowLevel })) {
      return null;
    }
    try {
      if (!(options && options.silent)) {
        setMessage(optimisticMessage);
        writeLog(optimisticMessage, 'info');
      }
      const result = await call();
      const message = formatServiceResult(result);
      if (!(options && options.silent)) {
        setMessage(message);
        writeLog(message, serviceResultFailed(result) ? 'error' : 'ok');
      }
      return result;
    } catch (error) {
      const message = error && error.message ? error.message : t('log.rosCallFail');
      if (options && options.silent) return null;
      if (options && options.keepConnectionStatus && client.connected) {
        setMessage(message);
      } else {
        setStatus('error', message);
      }
      writeLog(message, 'error');
      return null;
    }
  }

  function formatServiceResult(result) {
    if (!result) return t('log.rosCallDone');
    if (typeof result.accepted === 'boolean') return result.accepted ? t('msg.goalAccepted') : t('msg.goalRejected');
   if (typeof result.message === 'string' && result.message) return result.message;
    if (typeof result.reached_position === 'number') return t('msg.gripperReached', {mm: Math.round(result.reached_position * 1000)});
    if (Array.isArray(result.q_solution)) return t('log.ikResult', {result: result.success ? t('log.ikSuccess') : t('log.ikFail'), q: result.q_solution.map((v) => Number(v).toFixed(3)).join(', ')});
   if (typeof result.success === 'boolean') return result.success ? t('log.rosCallSuccess') : t('log.rosCallFail');
    return t('log.rosCallDone');
  }

  function serviceResultFailed(result) {
    if (!result || typeof result !== 'object') return false;
    if (result.accepted === false || result.success === false) return true;
    const errorCode = Number(result.error_code);
    return Number.isFinite(errorCode) && errorCode !== 0;
  }

  function updateDiagnostics() {
    updateCameraStatusFromTopic();
    const age = latestJointStateAt ? (performance.now() - latestJointStateAt) / 1000 : Infinity;
    const stale = client.connected && age > 2.5;
    if (els.control) {
      els.control.disabled = !client.connected || stale;
      if (stale && els.control.checked) {
        els.control.checked = false;
        stopRobot();
      }
    }
    if (feedbackSource) feedbackSource.textContent = client.connected
      ? t('p01.feedback', { age: Number.isFinite(age) ? age.toFixed(1) : '--' }) : t('p01.localPreview');
    if (stale) setMessage(t('msg.feedbackStale'));
  }

  function markTopicDiag(el, topic) {
    const last = client.getLastMessageAt(topic);
    if (!client.connected) {
      markDiag(el, false, '--');
      return;
    }
    if (!last) {
      markDiag(el, null, listedTopics.has(topic) ? (topic === REQUIRED_TOPICS.armStatus ? t('st.diagFound') : t('st.diagFoundWait')) : t('st.diagWait'));
      return;
    }
    const age = (Date.now() - last) / 1000;
    const liveLimit = topic === REQUIRED_TOPICS.armStatus ? 90 : topic === REQUIRED_TOPICS.cameraImage ? 3.0 : 2.5;
    markDiag(el, age < liveLimit, `${age.toFixed(1)}s`);
  }

  function markDiag(el, ok, text) {
    if (!el) return;
    const box = el.closest('.diag-item');
    if (box) {
      box.classList.toggle('ok', ok === true);
      box.classList.toggle('warn', ok === null);
      box.classList.toggle('bad', ok === false);
    }
    el.textContent = text;
  }

  function normalizeJointName(name) {
    const text = String(name || '').toLowerCase();
    if (text.endsWith('gripper_joint') || text.endsWith('/gripper_joint')) return 'gripper_joint';
    if (text.endsWith('right_joint') || text.endsWith('/right_joint')) return 'right_joint';
    const match = text.match(/joint[_-]?([1-6])$/) || text.match(/j([1-6])$/);
    return match ? `joint${match[1]}` : null;
  }

  function handleCameraImage(msg, topic) {
    if (topic && CAMERA_IMAGE_TOPICS.includes(topic)) {
      activeCameraTopic = topic;
      if (els.cameraTopic) els.cameraTopic.textContent = topic;
    }
    if (topic && topic.includes('/mujoco/')) markSimulationDriverDetected(t('reason.mujocoCamera'));
    if (!els.cameraCanvas || !msg) return;
    const width = Number(msg.width) || 0;
    const height = Number(msg.height) || 0;
    if (!Number.isSafeInteger(width) || !Number.isSafeInteger(height) || width <= 0 || height <= 0) {
      setCameraStatus(t('st.cameraError'), 'error');
      return;
    }

    if (width > 4096 || height > 4096) { setCameraStatus(t('st.cameraDataError'), 'error'); return; }
    const bytes = rosImageBytes(msg.data);
    if (!bytes) {
      setCameraStatus(t('st.cameraDataError'), 'error');
      return;
    }

    const encoding = String(msg.encoding || 'rgb8').toLowerCase();
    const channels = encoding === 'rgba8' || encoding === 'bgra8' ? 4 : 3;
    const supported = encoding === 'rgb8' || encoding === 'bgr8' || encoding === 'rgba8' || encoding === 'bgra8';
    if (!supported) {
      setCameraStatus(encoding || t('st.encodingUnsupported'), 'warn');
      return;
    }

    const step = Number(msg.step) || width * channels;
    if (!Number.isSafeInteger(step) || step < width * channels || bytes.length < step * height) {
      setCameraStatus(t('st.cameraDataError'), 'error'); return;
    }
    if (els.cameraCanvas.width !== width) els.cameraCanvas.width = width;
    if (els.cameraCanvas.height !== height) els.cameraCanvas.height = height;
    const ctx = els.cameraCanvas.getContext('2d');
    const frame = ctx.createImageData(width, height);
    const dst = frame.data;
    const bgr = encoding === 'bgr8' || encoding === 'bgra8';

    for (let y = 0; y < height; y += 1) {
      const row = y * step;
      for (let x = 0; x < width; x += 1) {
        const src = row + x * channels;
        const out = (y * width + x) * 4;
        dst[out] = bgr ? bytes[src + 2] : bytes[src];
        dst[out + 1] = bytes[src + 1];
        dst[out + 2] = bgr ? bytes[src] : bytes[src + 2];
        dst[out + 3] = channels === 4 ? bytes[src + 3] : 255;
      }
    }

    ctx.putImageData(frame, 0, 0);
    setCameraStatus(`${width}x${height}`, 'online');
    updateDiagnostics();
  }

  function rosImageBytes(data) {
    if (!data) return null;
    if (Array.isArray(data)) return data;
    if (data instanceof Uint8Array) return data;
    if (typeof data === 'string') {
      try {
        const binary = window.atob(data);
        const bytes = new Uint8Array(binary.length);
        for (let i = 0; i < binary.length; i += 1) bytes[i] = binary.charCodeAt(i);
        return bytes;
      } catch (error) {
        return null;
      }
    }
    if (data.buffer instanceof ArrayBuffer) return new Uint8Array(data.buffer);
    return null;
  }

  function updateCameraStatusFromTopic() {
    if (!els.cameraStatus) return;
    if (!client.connected) {
      setCameraStatus(t('st.cameraOffline'), 'error');
      return;
    }
    const last = client.getLastMessageAt(activeCameraTopic);
    if (!last) {
      setCameraStatus(CAMERA_IMAGE_TOPICS.some((topic) => listedTopics.has(topic)) ? t('st.cameraWaitFrame') : t('st.cameraWaitTopic'), 'warn');
      return;
    }
    const age = (Date.now() - last) / 1000;
    if (age > 3.0) {
      setCameraStatus(`${age.toFixed(1)}s`, 'warn');
    }
  }

  function setCameraStatus(text, state) {
    if (!els.cameraStatus) return;
    els.cameraStatus.textContent = text;
    els.cameraStatus.classList.toggle('online', state === 'online');
    els.cameraStatus.classList.toggle('warn', state === 'warn');
    els.cameraStatus.classList.toggle('error', state === 'error');
  }

  function handleVisionDetections(msg) {
    markSimulationDriverDetected(t('reason.simVision'));
    if (!els.visionStatus && !els.visionTarget) return;
    let payload = null;
    try {
      payload = JSON.parse(msg && msg.data ? msg.data : '{}');
    } catch (error) {
      if (els.visionStatus) els.visionStatus.textContent = t('st.cameraDataError');
      return;
    }

    latestVisionPayload = payload;
    latestVisionAt = performance.now();
    const count = Number(payload.count) || 0;
   if (els.visionStatus) {
      els.visionStatus.textContent = count ? t('fb.visionCount', {count, color: payload.target_color || '--'}) : t('st.visionNone');
   }
    updateSelectedVisionTarget();
  }

  function handleMujocoObjectStates(msg) {
    let payload = null;
    try {
      payload = JSON.parse(msg && msg.data ? msg.data : '{}');
    } catch (error) {
      return;
    }
    markSimulationDriverDetected(t('reason.mujocoObject'));
    if (window.reBotSim && typeof window.reBotSim.syncMujocoObjectStates === 'function') {
      window.reBotSim.syncMujocoObjectStates(payload.objects || []);
    }
  }

  function handleSimAnimationEvent(msg) {
    let payload = null;
    try {
      payload = JSON.parse(msg && msg.data ? msg.data : '{}');
    } catch (error) {
      writeLog(t('log.mcpAnimParseFail'), 'warn');
      return;
    }

    const event = String(payload.event || payload.action || '').toLowerCase();
    if (event === 'attach_object') {
      const target = payload.target && typeof payload.target === 'object'
        ? payload.target
        : { color: payload.color };
      attachSimCarriedObject(target);
    } else if (event === 'release_object') {
      releaseSimCarriedObject();
    }
  }

  function updateSelectedVisionTarget() {
    const mode = els.visionColor ? String(els.visionColor.value || 'auto') : 'auto';
    const target = mode === 'auto'
      ? (chooseVisionTarget(autoVisionTargetColor) || chooseRandomVisionTarget())
      : chooseVisionTarget(mode);
    selectedVisionTarget = target;
    renderVisionTarget(target);
  }

  function renderVisionTarget(target) {
    if (!els.visionTarget) return;
    if (!target) {
      els.visionTarget.textContent = '--';
      return;
    }
    const approachZ = getVisionApproachZ(target);
   const graspPlan = estimateVisionGraspPlan(target);
    els.visionTarget.textContent = t('fb.visionTarget', {color: target.color, x: Number(target.x).toFixed(3), y: Number(target.y).toFixed(3), z: approachZ.toFixed(3), mm: Math.round(graspPlan.physicalGap * 1000), yaw: Math.round(graspPlan.yawRad * 180 / Math.PI)});
 }

  function chooseRandomVisionTarget() {
    const detections = latestVisionPayload && Array.isArray(latestVisionPayload.detections)
      ? latestVisionPayload.detections
      : [];
    const colors = [...new Set(
      detections
        .filter((item) => item && item.color)
        .map((item) => String(item.color))
    )];
    if (!colors.length) return null;
    const alternatives = colors.filter((color) => color !== autoVisionTargetColor);
    const pool = alternatives.length ? alternatives : colors;
    autoVisionTargetColor = pool[Math.floor(Math.random() * pool.length)];
    return chooseVisionTarget(autoVisionTargetColor);
  }

  function chooseVisionTarget(preferredColor) {
    const detections = latestVisionPayload && Array.isArray(latestVisionPayload.detections)
      ? latestVisionPayload.detections
      : [];
    if (!detections.length) return null;
    const color = preferredColor || (els.visionColor ? String(els.visionColor.value || 'auto') : 'auto');
    if (color && color !== 'auto') {
      return chooseMujocoAssociatedDetection(
        color,
        detections.filter((item) => item && item.color === color)
      );
    }
    const payloadTarget = latestVisionPayload.target;
    if (payloadTarget && payloadTarget.color) {
      const associated = chooseMujocoAssociatedDetection(
        String(payloadTarget.color),
        detections.filter((item) => item && item.color === payloadTarget.color)
      );
      if (associated) return associated;
    }
    return detections[0] || null;
  }

  function chooseMujocoAssociatedDetection(color, candidates) {
    if (!Array.isArray(candidates) || !candidates.length) return null;
    if (!window.reBotSim || typeof window.reBotSim.getSceneCollisionMap !== 'function') {
      return candidates[0];
    }
    const map = window.reBotSim.getSceneCollisionMap();
    const expected = map && map.source === 'mujoco_object_states'
      && map.objects && map.objects[color] && map.objects[color].position;
    if (!expected) return candidates[0];

    const ranked = candidates
      .map((item) => ({
        item,
        distance: Math.hypot(Number(item.x) - expected.x, Number(item.y) - expected.y)
      }))
      .filter((entry) => Number.isFinite(entry.distance))
      .sort((a, b) => a.distance - b.distance);
    return ranked.length && ranked[0].distance <= 0.055 ? ranked[0].item : null;
  }

  async function waitForFreshVisionTarget(preferredColor, timeoutMs) {
    const start = performance.now();
    const initialVisionAt = latestVisionAt;
    let target = chooseVisionTarget(preferredColor);
    while (performance.now() - start < timeoutMs) {
      const fresh = latestVisionAt > initialVisionAt || performance.now() - latestVisionAt < 450;
      target = chooseVisionTarget(preferredColor) || target;
      if (target && fresh) return cloneVisionTarget(target);
      await sleep(80);
    }
    return target ? cloneVisionTarget(target) : null;
  }

  function fillPoseFromVisionTarget() {
    const target = selectedVisionTarget || chooseVisionTarget();
    const pose = poseFromVisionTarget(getVisionApproachZ(target), target);
    if (!pose) return;
    writePoseInputs(pose);
    if (client.connected) client.publishTargetPose(pose);
    setMessage(t('msg.visionFillDone'));
    writeLog(t('log.visionFill'), 'ok');
  }

  async function moveAboveVisionTarget() {
    if (!controlAllowed(true)) return;
    const mode = els.visionColor ? String(els.visionColor.value || 'auto') : 'auto';
    const target = mode === 'auto'
      ? (chooseRandomVisionTarget() || selectedVisionTarget || chooseVisionTarget())
      : (selectedVisionTarget || chooseVisionTarget(mode));
    const pose = poseFromVisionTarget(getVisionApproachZ(target), target);
    if (!pose) return;
    if (mode === 'auto') {
      selectedVisionTarget = target;
     renderVisionTarget(target);
      writeLog(t('log.autoTarget', {color: target.color}), 'info');
   }
    const previousTarget = lastVisionTarget && !sameVisionTarget(lastVisionTarget, target)
      ? lastVisionTarget
      : null;
    const duration = getPoseDuration();
    setVisionBusy(true, 'move');
    try {
      const route = buildVisionTransitRoute(previousTarget, target);
      for (const waypoint of route) {
        await runVisionMoveStep(waypoint.pose, Math.max(1.1, duration * 0.60), waypoint.label);
     }
      await runVisionMoveStep(pose, duration, t('msg.moveAboveTarget', {color: target.color}));
     lastVisionTarget = cloneVisionTarget(target);
    } catch (error) {
      const message = error && error.message ? error.message : t('msg.visionMoveAbort');
      setMessage(message);
      writeLog(message, 'warn');
    } finally {
      setVisionBusy(false);
    }
  }

  async function runMaterialPick() {
    if (visionSequenceBusy) return;
    const x = Number(els.materialX && els.materialX.value);
    const y = Number(els.materialY && els.materialY.value);
    const z = Number(els.materialZ && els.materialZ.value);
    const clearance = Number(els.materialApproach && els.materialApproach.value);
    const width = clamp(Number(els.materialWidth && els.materialWidth.value) / 1000, 0, OPEN_GRIPPER_M);
    if (![x, y, z, clearance, width].every(Number.isFinite) || z < 0 || clearance <= 0) {
      setMessage('物料坐标或夹爪宽度无效');
      return;
    }
    const target = { x, y, z, color: 'manual', width_m: width, height_m: width };
    const approach = { position: { x, y, z: z + clearance }, orientation: topDownOrientationWithYaw(0) };
    const grasp = { position: { x, y, z }, orientation: topDownOrientationWithYaw(0) };
    const lift = { position: { x, y, z: z + clearance }, orientation: topDownOrientationWithYaw(0) };
    const duration = getPoseDuration();
    const hardwareControl = client.connected && els.control && els.control.checked;
    visionSequenceBusy = true;
    try {
      releaseSimCarriedObject();
      if (!hardwareControl) {
        const moveLocal = async (pose, label) => {
          if (!window.reBotSim || typeof window.reBotSim.moveToTcp !== 'function') throw new Error('仿真模型未加载');
          window.reBotSim.moveToTcp(pose.position, label);
          await sleep(duration * 1000);
        };
        await moveLocal(approach, '仿真：移动到物料上方');
        await moveLocal(grasp, '仿真：下探到物料');
        window.reBotSim.setGripperWidth(width, { source: 'ui', animate: true, localOnly: true });
        await sleep(650);
        await moveLocal(lift, '仿真：抬升物料');
        setMessage('仿真坐标抓取完成');
        writeLog(`仿真坐标抓取完成: X=${x.toFixed(3)} Y=${y.toFixed(3)} Z=${z.toFixed(3)}`, 'ok');
        return;
      }
      await commandGripperAndWait(OPEN_GRIPPER_M, '夹爪打开', { requireReached: true, minWaitMs: 500 });
      await runVisionMoveStep(approach, duration, '移动到物料上方');
      await runVisionMoveStep(grasp, Math.max(0.8, duration * 0.55), '下探到物料');
      await commandGripperAndWait(width, '夹紧物料', { allowContactStop: true, minWaitMs: 500 });
      attachSimCarriedObject(target);
      await runVisionMoveStep(lift, Math.max(1.0, duration * 0.65), '抬升物料');
      setMessage('坐标抓取完成');
      writeLog(`坐标抓取完成: X=${x.toFixed(3)} Y=${y.toFixed(3)} Z=${z.toFixed(3)}`, 'ok');
    } catch (error) {
      const message = error && error.message ? error.message : '坐标抓取中止';
      setMessage(message);
      writeLog(message, 'warn');
    } finally {
      visionSequenceBusy = false;
    }
  }

  async function runVisionPickDemo() {
    if (visionSequenceBusy) return;
    if (!controlAllowed(true)) return;
    const preferredColor = els.visionColor ? String(els.visionColor.value || 'auto') : 'auto';
    let target = await waitForFreshVisionTarget(preferredColor, 700);
    if (preferredColor === 'auto') {
      target = chooseRandomVisionTarget() || target;
      if (target) {
        selectedVisionTarget = target;
       renderVisionTarget(target);
        writeLog(t('log.autoTarget', {color: target.color}), 'info');
     }
    }
    if (!target) {
      setMessage(t('msg.noVisionTarget'));
      return;
    }

    const previousTarget = lastVisionTarget && !sameVisionTarget(lastVisionTarget, target)
      ? lastVisionTarget
      : null;
    let plan = buildVisionPickPlan(target);
   if (!plan) return;
   writeLog(
      t('log.visionGraspPose', {color: target.color, mm: Math.round(plan.graspPlan.physicalGap * 1000), yaw: Math.round(plan.graspPlan.yawRad * 180 / Math.PI), lift: plan.firstLiftPose.position.z.toFixed(3), transit: plan.transitPose.position.z.toFixed(3)}),
     'info'
    );

    const duration = getPoseDuration();
    let lastPose = null;
    const runIfNeeded = async (pose, moveDuration, label) => {
     if (lastPose && poseDistance(lastPose, pose) < VISION_POSE_SKIP_M) {
        writeLog(t('log.visionSkipMove', {label}), 'info');
       return { success: true, skipped: true };
      }
      const result = await runVisionMoveStep(pose, moveDuration, label);
      lastPose = pose;
      return result;
    };

    setVisionBusy(true, 'pick');
    try {
      releaseSimCarriedObject();
      await commandGripperAndWait(OPEN_GRIPPER_M, t('msg.pickOpenGripper'), {
        timeoutMs: 2600,
        minWaitMs: 850,
        tolerance: 0.006,
        requireReached: true,
        afterMs: 180
      });

      const route = buildVisionTransitRoute(previousTarget, target);
      for (const waypoint of route) {
        await runIfNeeded(waypoint.pose, Math.max(1.2, duration * 0.65), waypoint.label);
      }

      const refinedTarget = await waitForFreshVisionTarget(target.color, 420);
      if (refinedTarget && visionTargetShifted(refinedTarget, target, 0.008)) {
        target = refinedTarget;
        plan = buildVisionPickPlan(target);
       if (!plan) return;
        writeLog(t('log.visionRelocate', {color: target.color}), 'info');
       writeLog(
          t('log.visionGraspPose', {color: target.color, mm: Math.round(plan.graspPlan.physicalGap * 1000), yaw: Math.round(plan.graspPlan.yawRad * 180 / Math.PI), lift: plan.firstLiftPose.position.z.toFixed(3), transit: plan.transitPose.position.z.toFixed(3)}),
         'info'
        );
      }

     writePoseInputs(plan.approachPose);
      await runIfNeeded(plan.approachPose, duration, t('msg.pickMoveAbove', {color: target.color}));

     const alignDuration = Math.max(1.0, duration * 0.55);
      await runIfNeeded(plan.verticalAlignPose, alignDuration, t('msg.pickAlign', {color: target.color}));

     const pregraspDuration = Math.max(0.85, duration * 0.45);
      await runIfNeeded(plan.pregraspPose, pregraspDuration, t('msg.pickPreDescend', {color: target.color}));

     const descendDuration = Math.max(1.1, duration * 0.65);
      await runIfNeeded(plan.graspPose, descendDuration, t('msg.pickDescend', {color: target.color}));

      await commandGripperAndWait(plan.graspPlan.command, t('msg.pickSqueeze', {color: target.color}), {
       timeoutMs: 2100,
        minWaitMs: 850,
        tolerance: 0.006,
        allowContactStop: true,
        afterMs: 220
      });
      attachSimCarriedObject(target);

      const firstLiftDuration = Math.max(1.25, duration * 0.75);
     if (String(target.color || '') === 'blue') {
        await runVisionMoveStep(plan.firstLiftPose, firstLiftDuration, t('msg.pickBlueLift', {z: plan.firstLiftPose.position.z.toFixed(3)}));
       lastPose = plan.firstLiftPose;
        await runVisionMoveStep(plan.transitPose, Math.max(1.45, duration * 0.70), t('msg.pickBlueTransit', {z: plan.transitPose.position.z.toFixed(3)}));
       lastPose = plan.transitPose;
     } else {
        await runIfNeeded(plan.firstLiftPose, firstLiftDuration, t('msg.pickLift', {color: target.color}));
      }

     const liftDuration = Math.max(1.8, duration * 0.85);
      await runIfNeeded(plan.approachPose, liftDuration, t('msg.pickRaise', {color: target.color}));

     const finalTransitDuration = Math.max(1.1, duration * 0.60);
      await runIfNeeded(plan.transitPose, finalTransitDuration, t('msg.pickTransit', {color: target.color}));

     lastVisionTarget = cloneVisionTarget(target);
      setMessage(t('msg.graspDemoDone', {mm: Math.round(plan.graspPlan.physicalGap * 1000)}));
      writeLog(t('log.graspDone', {mm: Math.round(plan.graspPlan.command * 1000)}), 'ok');
   } catch (error) {
      const message = error && error.message ? error.message : t('msg.visionPickAbort');
      setMessage(message);
      writeLog(message, 'warn');
    } finally {
      setVisionBusy(false);
    }
  }

  async function runVisionPlaceDemo() {
    if (visionSequenceBusy) return;
    if (!controlAllowed(true)) return;

    const simCarriedColor = window.reBotSim && typeof window.reBotSim.getCarriedObject === 'function'
      ? window.reBotSim.getCarriedObject()
      : '';
    const target = heldVisionTarget
      || (simCarriedColor && lastVisionTarget && String(lastVisionTarget.color) === String(simCarriedColor)
        ? lastVisionTarget
        : null);
    if (!target) {
      setMessage(t('msg.noHeldObject'));
      writeLog(t('log.placeIgnored'), 'warn');
      return;
    }

    const plan = buildVisionPickPlan(target);
    if (!plan) return;
    const duration = getPoseDuration();
    setVisionBusy(true, 'place');
    try {
      await runVisionMoveStep(
       plan.approachPose,
       Math.max(1.1, duration * 0.65),
        t('msg.placeMoveAbove', {color: target.color})
     );
      await runVisionMoveStep(
       plan.graspPose,
       Math.max(1.1, duration * 0.65),
        t('msg.placeDescend', {color: target.color})
     );
      await commandGripperAndWait(OPEN_GRIPPER_M, t('msg.placeOpen', {color: target.color}), {
       timeoutMs: 2600,
        minWaitMs: 850,
        tolerance: 0.006,
        requireReached: true,
        afterMs: 220
      });
      releaseSimCarriedObject();
      await runVisionMoveStep(
       plan.approachPose,
       Math.max(1.5, duration * 0.8),
        t('msg.placeLift', {color: target.color})
     );
     lastVisionTarget = cloneVisionTarget(target);
      setMessage(t('msg.placeDone', {color: target.color}));
      writeLog(t('log.placeDoneLog', {color: target.color}), 'ok');
   } catch (error) {
      const message = error && error.message ? error.message : t('msg.visionPlaceAbort');
      setMessage(message);
      writeLog(message, 'warn');
    } finally {
      setVisionBusy(false);
    }
  }

  async function runVisionMoveStep(pose, duration, label) {
    writePoseInputs(pose);
    client.publishTargetPose(pose);
    const result = await sendVisionMoveGoal(pose, duration, label);
   if (!movementSucceeded(result)) {
      throw new Error(t('msg.stepFailed', {label}));
   }
    if (!(result && result.localPlayback)) {
      await sleep(duration * 1000 + 300);
    }
    return result;
  }

  function movementSucceeded(result) {
    if (!result) return false;
    if (result.success === false || result.accepted === false) return false;
    return true;
  }

  function buildVisionPickPlan(target) {
    const approachZ = getVisionApproachZ(target);
    const graspZ = getVisionGraspZ(target);
    const transitPose = poseFromVisionTarget(getVisionTransitZ(target), target);
    const approachPose = poseFromVisionTarget(approachZ, target);
    const verticalAlignZ = Math.min(
      approachZ,
      Math.max(graspZ + VISION_VERTICAL_ALIGN_CLEARANCE_M, VISION_MIN_VERTICAL_ALIGN_Z_M)
    );
    const pregraspZ = Math.min(
      verticalAlignZ,
      Math.max(graspZ + VISION_PREGRASP_CLEARANCE_M, graspZ)
    );
    const verticalAlignPose = poseFromVisionTarget(verticalAlignZ, target);
    const pregraspPose = poseFromVisionTarget(pregraspZ, target);
    const graspPose = poseFromVisionTarget(graspZ, target);
    const firstLiftZ = Math.min(
      approachZ,
      Math.max(graspZ + VISION_FIRST_LIFT_CLEARANCE_M, getVisionFirstLiftMinZ(target))
    );
    const firstLiftPose = poseFromVisionTarget(firstLiftZ, target);
    if (!transitPose || !approachPose || !verticalAlignPose || !pregraspPose || !graspPose || !firstLiftPose) return null;
    return {
      target,
      approachZ,
      graspZ,
      transitPose,
      approachPose,
      verticalAlignPose,
      pregraspPose,
      graspPose,
      firstLiftPose,
      graspPlan: estimateVisionGraspPlan(target)
    };
  }

  function buildVisionTransitRoute(previousTarget, target) {
    const route = [];
    if (previousTarget) {
      appendVisionRoutePose(
        route,
       poseFromVisionTarget(getVisionTransitZ(previousTarget), previousTarget),
        t('msg.avoidLift', {color: previousTarget.color})
     );
    }
    appendVisionRoutePose(
      route,
     poseFromVisionTarget(getVisionTransitZ(target), target),
      t('msg.avoidMove', {color: target.color})
   );
    return route;
  }

  function appendVisionRoutePose(route, pose, label) {
    if (!pose) return;
    const last = route.length ? route[route.length - 1].pose : null;
    if (last && poseDistance(last, pose) < 0.025) return;
    route.push({ pose, label });
  }

  function poseDistance(left, right) {
    const values = [
      left && left.position && left.position.x,
      left && left.position && left.position.y,
      left && left.position && left.position.z,
      right && right.position && right.position.x,
      right && right.position && right.position.y,
      right && right.position && right.position.z
    ].map(Number);
    if (!values.every(Number.isFinite)) return Infinity;
    return Math.hypot(values[0] - values[3], values[1] - values[4], values[2] - values[5]);
  }

  function poseFromVisionTarget(zOverride, targetOverride) {
    const target = targetOverride || selectedVisionTarget || chooseVisionTarget();
    if (!target) {
      setMessage(t('msg.noVisionTarget'));
      return null;
    }
    const x = Number(target.x);
    const y = Number(target.y);
    const z = Number(zOverride);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      setMessage(t('msg.visionCoordError'));
      return null;
    }
    selectedVisionTarget = target;
    const graspPlan = estimateVisionGraspPlan(target);
    return {
      position: { x, y, z },
      orientation: topDownOrientationWithYaw(graspPlan.yawRad)
    };
  }

  async function sendVisionMoveGoal(pose, duration, optimisticMessage) {
    return moveToPoseViaIkTrajectory(pose, duration, optimisticMessage);
  }

 async function moveToPoseViaIkTrajectory(pose, duration, optimisticMessage) {
    const epoch = motionEpoch;
    if (!controlAllowed(true) || teachingHardwareBusy) return { success: false };
    const ik = await guardedCall(
      () => client.solveMoveToPoseIK(pose), t('msg.ikSolving', {label: optimisticMessage}),
      false, { keepConnectionStatus: true }
    );
    if (epoch !== motionEpoch) return { success: false, message: t('p01.cancelled') };
    const defs = window.reBotSim.getJointDefs().filter((def) => def.name !== 'gripper');
    if (epoch !== motionEpoch || !ik || ik.success !== true || !Array.isArray(ik.q_solution)
        || ik.q_solution.length !== JOINT_NAMES.length
        || !ik.q_solution.every((value, index) => Number.isFinite(value) && value >= defs[index].min && value <= defs[index].max)) {
      const message = ik && ik.message || t('msg.ikNoSolution');
      setMessage(message);
      setExecution('p01.failed');
      return { success: false, message };
    }
    return sendTrajectory(buildSmoothJointMovePoints(getCurrentRosPositions(), ik.q_solution, duration), optimisticMessage);
  }

  function buildSmoothJointMovePoints(start, goal, duration) {
    const seconds = controlPolicy.computeJointTrajectoryDuration(
      start,
      goal,
      getVlim(),
      clamp(Number(duration) || 2, 0.4, 8)
    );
    // The controller already performs cubic Hermite interpolation. Sending
    // dozens of zero-velocity waypoints made the arm decelerate at every
    // 33 ms waypoint and prevented IK moves from tracking the final target.
    return [
      makeTrajectoryPoint(start, 0.05),
      makeTrajectoryPoint(goal, seconds)
    ];
  }

  function writePoseInputs(pose) {
    if (els.poseX) els.poseX.value = Number(pose.position.x).toFixed(3);
    if (els.poseY) els.poseY.value = Number(pose.position.y).toFixed(3);
    if (els.poseZ) els.poseZ.value = Number(pose.position.z).toFixed(3);
  }

  function getVisionApproachZ(target) {
    const transitZ = getVisionTransitZ(target);
    const detected = target && Number.isFinite(Number(target.z)) ? Number(target.z) : transitZ;
    const requested = Number(els.visionApproachZ && els.visionApproachZ.value);
    const value = Number.isFinite(requested) ? requested : detected;
    return clamp(Math.max(value, transitZ), 0.08, 0.42);
  }

  function getVisionTransitZ(target) {
    const color = target && target.color ? String(target.color) : '';
    const value = VISION_TRANSIT_Z_BY_COLOR_M[color];
    return Number.isFinite(value) ? value : VISION_TRANSIT_Z_M;
  }

  function getVisionGraspZ(target) {
    const safe = estimateVisionGraspZ(target);
    const requested = Number(els.visionGraspZ && els.visionGraspZ.value);
    const value = Number.isFinite(requested) ? Math.max(requested, safe) : safe;
    return clamp(value, 0.06, 0.25);
  }

  function estimateVisionGraspZ(target) {
    const color = target && target.color ? String(target.color) : '';
    const safeByColor = {
      red: 0.137,
      blue: 0.132,
      yellow: 0.164
    };
    if (Number.isFinite(safeByColor[color])) return safeByColor[color];
    const fallback = target && Number.isFinite(Number(target.z)) ? Math.max(Number(target.z) - 0.035, 0.13) : 0.14;
    return fallback;
  }

  function getVisionFirstLiftMinZ(target) {
    const color = target && target.color ? String(target.color) : '';
    const value = VISION_FIRST_LIFT_MIN_BY_COLOR_M[color];
    return Number.isFinite(value) ? value : VISION_FIRST_LIFT_MIN_M;
  }

  function sameVisionTarget(left, right) {
    if (!left || !right) return false;
    if (String(left.color || '') !== String(right.color || '')) return false;
    const dx = Number(left.x) - Number(right.x);
    const dy = Number(left.y) - Number(right.y);
    return Number.isFinite(dx) && Number.isFinite(dy) && Math.hypot(dx, dy) < 0.035;
  }

  function visionTargetShifted(left, right, threshold) {
    if (!left || !right) return false;
    if (String(left.color || '') !== String(right.color || '')) return false;
    const dx = Number(left.x) - Number(right.x);
    const dy = Number(left.y) - Number(right.y);
    const dz = Number(left.z || 0) - Number(right.z || 0);
    if (![dx, dy, dz].every(Number.isFinite)) return false;
    return Math.hypot(dx, dy, dz) > threshold;
  }

  function cloneVisionTarget(target) {
    if (!target || typeof target !== 'object') return null;
    return { ...target };
  }

  function attachSimCarriedObject(target) {
    const color = target && target.color ? String(target.color) : '';
    if (!color || color === 'manual') {
      if (window.reBotSim && typeof window.reBotSim.attachObjectAtPosition === 'function') {
        window.reBotSim.attachObjectAtPosition(target, 0.10);
      }
      return;
    }
    heldVisionTarget = cloneVisionTarget(target);
    if (!window.reBotSim || typeof window.reBotSim.attachObject !== 'function') return;
   if (window.reBotSim.attachObject(color)) {
      writeLog(t('log.simAttach', {color}), 'ok');
   }
  }

  function releaseSimCarriedObject() {
    heldVisionTarget = null;
    if (!window.reBotSim || typeof window.reBotSim.releaseObject !== 'function') return;
    if (window.reBotSim.releaseObject({ settleOnTable: true })) {
      writeLog(t('log.simRelease'), 'info');
    }
  }

  function estimateVisionGraspPlan(target) {
    const fallbackByColor = {
      red: 0.05,
      yellow: 0.044,
      blue: 0.044
    };
    const width = Number(target && target.width_m);
    const height = Number(target && target.height_m);
    let crossSection = Number(target && target.shortest_m);
    let yawRad = 0;

    const color = target && target.color ? String(target.color) : '';
    if (Number.isFinite(width) && Number.isFinite(height) && width > 0 && height > 0) {
      const candidates = [
        { crossSection: height, yawRad: 0 },
        { crossSection: width, yawRad: -Math.PI / 2 }
      ].sort((left, right) => {
        const leftFits = left.crossSection <= GRIPPER_EFFECTIVE_GAP_M;
        const rightFits = right.crossSection <= GRIPPER_EFFECTIVE_GAP_M;
        if (leftFits !== rightFits) return leftFits ? -1 : 1;
        return left.crossSection - right.crossSection;
      });
      crossSection = candidates[0].crossSection;
      yawRad = candidates[0].yawRad;
    } else {
      if (!Number.isFinite(crossSection) || crossSection <= 0) {
        crossSection = fallbackByColor[color] || 0.05;
      }
      const reportedYaw = Number(target && target.grasp_yaw_rad);
      if (Number.isFinite(reportedYaw)) {
        yawRad = reportedYaw;
      }
    }

    const nominal = fallbackByColor[color];
    if (Number.isFinite(nominal)) {
      crossSection = Math.max(crossSection, nominal);
    }

    const physicalGap = clamp(
      crossSection - GRASP_SQUEEZE_M,
      MIN_OBJECT_GRASP_M,
      GRIPPER_EFFECTIVE_GAP_M
    );
    return {
      command: physicalGapToGripperCommand(physicalGap),
      physicalGap,
      yawRad
    };
  }

  function physicalGapToGripperCommand(physicalGap) {
    const travel = Math.max(GRIPPER_VISUAL_TRAVEL_M, 0.001);
    return clamp(((physicalGap - GRIPPER_BASE_GAP_M) / travel) * OPEN_GRIPPER_M, CLOSE_GRIPPER_M, OPEN_GRIPPER_M);
  }

  function topDownOrientationWithYaw(yawRad) {
    const yaw = Number.isFinite(yawRad) ? yawRad : 0;
    const sy = Math.sin(yaw / 2);
    const cy = Math.cos(yaw / 2);
    const s90 = Math.SQRT1_2;
    return {
      x: -sy * s90,
      y: cy * s90,
      z: sy * s90,
      w: cy * s90
    };
  }

  function getPoseDuration() {
    return clamp(Number(els.poseDuration && els.poseDuration.value) || 2, 0.4, 8);
  }

  function setVisionBusy(busy, operation) {
    visionSequenceBusy = busy;
    lastVisionOp = busy ? operation : null;
    if (els.visionPickDemo) {
      els.visionPickDemo.disabled = busy;
      els.visionPickDemo.textContent = busy && operation === 'pick' ? t('btn.pickBusy') : t('camera.pick');
    }
    if (els.visionPlaceDemo) {
      els.visionPlaceDemo.disabled = busy;
      els.visionPlaceDemo.textContent = busy && operation === 'place' ? t('btn.placeBusy') : t('camera.place');
    }
    if (els.visionMoveAbove) els.visionMoveAbove.disabled = busy;
    if (els.visionFillPose) els.visionFillPose.disabled = busy;
  }

  function updateFeedbackError(feedback) {
    if (!els.feedbackError || !window.reBotSim || !feedback || !Object.keys(feedback).length) return;
    const simAngles = typeof window.reBotSim.getAngles === 'function' ? window.reBotSim.getAngles() : {};
    let maxError = 0;
    let sumSq = 0;
    let count = 0;
    let worstJoint = '';

    Object.entries(feedback).forEach(([name, value]) => {
      const target = simTargetAngles.has(name) ? simTargetAngles.get(name) : simAngles[name];
      if (typeof target !== 'number') return;
      const error = Math.abs(target - value);
      if (error > maxError) {
        maxError = error;
        worstJoint = name;
      }
      sumSq += error * error;
      count += 1;
    });

    if (!count) return;
   const rms = Math.sqrt(sumSq / count);
    els.feedbackError.textContent = t('fb.errorMax', {max: (maxError * 180 / Math.PI).toFixed(2), joint: worstJoint || '', rms: (rms * 180 / Math.PI).toFixed(2)});
   els.feedbackError.style.color = maxError < 0.035 ? 'var(--green)' : (maxError < 0.12 ? 'var(--amber)' : 'var(--red)');
  }

  function updateGravityStatus(active, detail, source) {
    const nextActive = Boolean(active);
    const nextSource = source || 'system';
    // ArmStatus arrives faster than the detailed status service. Once the
    // service has supplied its lock-target text, do not let the short machine
    // state overwrite it on every ArmStatus message and make the UI flicker.
    if (
      nextSource === 'arm' &&
      gravityStatusSource === 'service' &&
      nextActive === gravityCompensationActive
    ) return;

    gravityCompensationActive = nextActive;
    gravityStatusSource = nextSource;
    if (!els.gravityStatus) return;
    els.gravityStatus.textContent = nextActive ? t('st.running') : t('st.notRunning');
    if (detail && detail !== 'GRAVITY_COMP') {
      els.gravityStatus.textContent += ` / ${detail}`;
    }
    els.gravityStatus.style.color = nextActive ? 'var(--green)' : 'var(--amber)';
  }

  function maybeSendGripper(position) {
    syncSimGripper(position);
    if (!client.connected) {
      setMessage(t('msg.gripperSimOnly'));
      return;
    }
    if (!controlAllowed(false, { allowLowLevel: true })) {
      setMessage(t('msg.controlLockClosed'));
      return;
    }
    publishGripper(position);
  }

  function sendGripper(position, options) {
    if (teachingHardwareBusy) return;
    syncSimGripper(position);
    if (
      options &&
      options.requireControl &&
      !controlAllowed(true, { skipConfirm: true, allowLowLevel: true })
    ) return;
    if (!client.connected) {
      setStatus('closed', t('msg.rosNotConnected'));
      return;
    }
    publishGripper(position);
  }

  async function commandGripperAndWait(position, label, options) {
    const settings = {
      timeoutMs: 1800,
      minWaitMs: 500,
      tolerance: 0.006,
      settleMs: 260,
      afterMs: 0,
      allowContactStop: false,
      requireReached: false,
      ...(options || {})
    };
    const serviceResult = await publishGripper(position);
    if (!serviceResult || serviceResult.success === false) {
      throw new Error(`${label}夹爪命令未执行`);
    }
    setMessage(label);

    if (serviceResult.success === true) {
      if (settings.afterMs > 0) await sleep(settings.afterMs);
      const reachedWidth = gripperMotorPositionToWidth(serviceResult.reached_position);
      const feedback = Number.isFinite(reachedWidth)
        ? t('fb.gripperSrcFb', {src: 'gripper/set', mm: Math.round(reachedWidth * 1000)})
        : '';
      writeLog(t('log.gripperDone', {label, fb: feedback}), 'ok');
      return;
    }

    const start = performance.now();
    const initialFeedbackAt = latestGripperAt;
    const initialJointFeedback = gripperJointFeedback();
    let lastPosition = readGripperFeedbackPosition(position);
    let stableSince = start;
    let sawFreshFeedback = false;
    let reached = false;
    let current = lastPosition;
    let source = latestGripperAt > 0 ? 'gripper/state' : '';

    while (performance.now() - start < settings.timeoutMs) {
      await sleep(80);
      const now = performance.now();

      const jointFeedback = gripperJointFeedback();
      const hasFreshGripperState = latestGripperAt > initialFeedbackAt && now - latestGripperAt < 700;
      const hasFreshJointState = jointFeedback.fresh && jointFeedback.stamp !== initialJointFeedback.stamp;
      const hasFreshFeedback = hasFreshJointState || hasFreshGripperState;
      if (!hasFreshFeedback) {
        if (!settings.requireReached && now - start > Math.max(settings.minWaitMs, 900)) break;
        continue;
      }

      sawFreshFeedback = true;
      current = hasFreshJointState ? jointFeedback.widthCommand : Number(latestGripperPosition);
      source = hasFreshJointState ? 'joint_states/gripper_joint' : 'gripper/state';
      const velocity = Number(latestGripperVelocity);
      const closeEnough = Number.isFinite(current) && Math.abs(current - position) <= settings.tolerance;
      const barelyMoving = Number.isFinite(velocity)
        ? Math.abs(velocity) < 0.0025
        : Number.isFinite(current) && Number.isFinite(lastPosition) && Math.abs(current - lastPosition) < 0.0015;
      reached = reached || closeEnough;

      if (barelyMoving) {
        if (now - stableSince >= settings.settleMs && now - start >= settings.minWaitMs) {
          if (closeEnough || (!settings.requireReached && settings.allowContactStop)) break;
        }
      } else {
        stableSince = now;
      }

      if (closeEnough && now - start >= settings.minWaitMs) break;
      lastPosition = current;
    }

    if (settings.afterMs > 0) await sleep(settings.afterMs);
   if (settings.requireReached && !reached) {
      const message = t('msg.gripperNotReached', {label});
     setMessage(message);
      writeLog(message, 'warn');
      throw new Error(message);
    }
   const feedback = sawFreshFeedback && Number.isFinite(Number(current))
      ? t('fb.gripperSrcFb', {src: source, mm: Math.round(current * 1000)})
     : '';
    writeLog(t('log.gripperDone', {label, fb: feedback}), 'ok');
 }

  function readGripperFeedbackPosition(commandPosition) {
    const jointFeedback = gripperJointFeedback();
    if (jointFeedback.fresh && Number.isFinite(jointFeedback.widthCommand)) {
      return jointFeedback.widthCommand;
    }
    if (Number.isFinite(Number(latestGripperPosition))) {
      return Number(latestGripperPosition);
    }
    return Number(commandPosition);
  }

  function gripperJointFeedback() {
    const source = latestJointPositions || {};
    const left = Number(source.gripper_joint);
    if (!Number.isFinite(left)) {
      return { fresh: false, widthCommand: NaN, stamp: 0 };
    }
    return {
      fresh: true,
      widthCommand: fingerOpeningToGripperCommand(left),
      stamp: latestJointStateAt || 0
    };
  }

  function fingerOpeningToGripperCommand(opening) {
    return clamp(Number(opening) * 2, CLOSE_GRIPPER_M, OPEN_GRIPPER_M);
  }

  function gripperWidthToMotorPosition(width) {
    if (!gripperMapping) return NaN;
    return gripperMapping.close + clamp(width, 0, gripperMapping.width)
      / gripperMapping.width * (gripperMapping.open - gripperMapping.close);
  }

  function gripperMotorPositionToWidth(position) {
    if (!gripperMapping || !Number.isFinite(position)) return NaN;
    return clamp((position - gripperMapping.close) / (gripperMapping.open - gripperMapping.close), 0, 1)
      * gripperMapping.width;
  }

  async function publishGripper(position) {
    syncSimGripper(position);
    if (!controlAllowed(true, { allowLowLevel: true }) || !gripperMapping || !Number.isFinite(position)
        || position < 0 || position > gripperMapping.width) {
      setMessage(t('p01.mappingUnavailable'));
      return { success: false };
    }
    const epoch = motionEpoch;
    if (gripperCommandBusy) {
      const message = '已有夹爪命令正在执行，请等待完成';
      setMessage(message);
      writeLog(message, 'warn');
      return null;
    }
    simTargetAngles.set('gripper', position);
    mirrorHoldUntil.set('gripper', performance.now() + 1200);
    gripperCommandBusy = true;
    try {
      const result = await guardedCall(
        () => client.setGripper(gripperWidthToMotorPosition(position), 0),
        t('msg.gripperCmdPublished', {mm: Math.round(position * 1000), fb: ''}),
        false,
        { keepConnectionStatus: true }
      );
      if (epoch !== motionEpoch) return { success: false };
      const feedback = typeof latestGripperPosition === 'number'
        ? t('fb.gripperFb', {mm: Math.round(latestGripperPosition * 1000)})
        : '';
      writeLog(t('log.gripperCmd', {mm: Math.round(position * 1000), topic: '/' + NS + '/gripper/set'}), result && result.success === true ? 'ok' : 'error');
      if (feedback) setMessage(t('msg.gripperCmdPublished', {mm: Math.round(position * 1000), fb: feedback}));
      return result;
    } finally {
      gripperCommandBusy = false;
    }
  }

  function syncSimGripper(position) {
    if (!window.reBotSim || typeof window.reBotSim.setGripperWidth !== 'function') return;
    window.reBotSim.setGripperWidth(position, {
      source: 'ui-preview',
      animate: true,
      localOnly: true,
      emit: false
    });
  }

  function getVlim() {
    return clamp(Number(els.vlim.value) || 0.3, 0.05, 1.5);
  }

  function getTrajectoryDuration() {
    return clamp(Number(els.trajectoryDuration.value) || 2, 1, 30);
  }

  function secondsToRosTime(seconds) {
    const ns = Math.round(seconds * 1e9);
    return { sec: Math.floor(ns / 1e9), nanosec: ns % 1e9 };
  }

  function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
  }

  function sleep(ms) {
    return new Promise((resolve) => window.setTimeout(resolve, ms));
  }

  function waitForSimApi(callback) {
    if (window.reBotSim && typeof window.reBotSim.onCommand === 'function') {
      callback(window.reBotSim);
      return;
    }
    window.setTimeout(() => waitForSimApi(callback), 50);
  }

  function setStatus(state, message) {
    lastStatusState = state;
    els.status.className = 'mini-pill';
    if (state === 'open') {
      els.status.classList.add('online');
      els.status.textContent = t('st.online');
    } else if (state === 'connecting') {
      els.status.classList.add('warn');
      els.status.textContent = t('st.connecting');
    } else if (state === 'error') {
      els.status.classList.add('error');
      els.status.textContent = t('st.error');
    } else {
      els.status.textContent = t('st.offline');
    }
    setMessage(message);
  }

  function setMessage(message) {
    if (!els.message) return;
    const next = message || '';
    if (els.message.textContent !== next) els.message.textContent = next;
  }

  function writeLog(message, level) {
    if (!els.log || !message) return;
    const line = document.createElement('div');
    line.className = `ros-log-line ${level || 'info'}`;
    const now = new Date();
    line.innerHTML = `<time>${now.toLocaleTimeString()}</time><span></span>`;
    line.querySelector('span').textContent = String(message);
    els.log.prepend(line);
    while (els.log.children.length > 80) els.log.lastElementChild.remove();
  }

  if (window.rebotI18n) {
    window.rebotI18n.onLangChange(() => {
      // Re-render ROS connection status pill
      if (lastStatusState !== null && els.status) {
        if (lastStatusState === 'open') {
          els.status.textContent = t('st.online');
        } else if (lastStatusState === 'connecting') {
          els.status.textContent = t('st.connecting');
        } else if (lastStatusState === 'error') {
          els.status.textContent = t('st.error');
        } else {
          els.status.textContent = t('st.offline');
        }
      }
      // Re-render gravity compensation status
      if (els.gravityStatus) {
        els.gravityStatus.textContent = gravityCompensationActive ? t('st.running') : t('st.notRunning');
        els.gravityStatus.style.color = gravityCompensationActive ? 'var(--green)' : 'var(--amber)';
      }
      // Re-render vision pick/place demo buttons
      if (els.visionPickDemo) {
        els.visionPickDemo.textContent = visionSequenceBusy && lastVisionOp === 'pick' ? t('btn.pickBusy') : t('camera.pick');
      }
      if (els.visionPlaceDemo) {
        els.visionPlaceDemo.textContent = visionSequenceBusy && lastVisionOp === 'place' ? t('btn.placeBusy') : t('camera.place');
      }
    });
  }
})();
