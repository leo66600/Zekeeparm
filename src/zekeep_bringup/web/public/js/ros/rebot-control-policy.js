(function (root, factory) {
  const policy = factory();
  if (typeof module === 'object' && module.exports) module.exports = policy;
  if (root) root.ReBotControlPolicy = policy;
})(typeof window !== 'undefined' ? window : globalThis, function () {
  function canExecuteRealMotion(state) {
    return Boolean(
      state && state.connected && state.controlEnabled && state.confirmed
    );
  }

  function hasSimulationEvidence(topics) {
    const names = new Set(Array.isArray(topics) ? topics : []);
    return names.has('/zekeep/mujoco/object_states')
      || names.has('/zekeep/sim/animation_event');
  }

  function shouldUseLowLevelTrajectory(state) {
    return Boolean(state && state.simulationDriverDetected);
  }

  function selectTrajectoryTransport(state) {
    if (state && state.actionAvailable) return 'action';
    if (state && state.simulationDriverDetected) return 'low-level';
    // The real controller always exposes the action, but rosbridge's action
    // discovery can lag or be unavailable.  Prefer the safe controller API
    // over raw arm topics, which are rejected by hybrid hardware control.
    return 'action';
  }

  function buildCoordinatedJointTarget(joints, jointNames, currentPositions) {
    const source = joints && typeof joints === 'object' ? joints : {};
    const names = Array.isArray(jointNames) ? jointNames : [];
    const current = Array.isArray(currentPositions) ? currentPositions : [];
    return names.map((name, index) => {
      const target = Number(source[name]);
      return Number.isFinite(target) ? target : Number(current[index] || 0);
    });
  }

  function computeJointTrajectoryDuration(
    currentPositions,
    targetPositions,
    maxVelocity,
    requestedDuration
  ) {
    const current = Array.isArray(currentPositions) ? currentPositions : [];
    const target = Array.isArray(targetPositions) ? targetPositions : [];
    const count = Math.min(current.length, target.length);
    let maxDisplacement = 0;
    for (let index = 0; index < count; index += 1) {
      const displacement = Math.abs(Number(target[index]) - Number(current[index]));
      if (Number.isFinite(displacement)) maxDisplacement = Math.max(maxDisplacement, displacement);
    }
    const velocity = Math.max(0.05, Number(maxVelocity) || 0.3);
    // A zero-endpoint-velocity cubic Hermite segment reaches 1.5*d/T.
    const velocityBoundedDuration = 1.5 * maxDisplacement / velocity;
    return Math.max(1, Number(requestedDuration) || 0, velocityBoundedDuration);
  }

  function shouldMirrorFeedback(state) {
    return Boolean(state && state.connected && state.mirrorEnabled);
  }

  function shouldUseHardwareTargets(state) {
    return Boolean(state && state.connected && state.controlEnabled);
  }

  function canStartNewRecording(state) {
    return Boolean(
      state && (!state.hasExistingPath || state.overwriteConfirmed)
    );
  }

  const MAX_WAYPOINT_BYTES = 4 * 1024 * 1024;
  const MAX_WAYPOINTS = 10000;

  function validateTeachingPayload(payload, jointDefs) {
    const fail = () => { throw new Error('Invalid zekeep_ros_waypoints_v1 trajectory'); };
    if (!payload || payload.format !== 'zekeep_ros_waypoints_v1'
        || payload.frame_id !== 'base_link'
        || (payload.gripper_unit !== undefined && payload.gripper_unit !== 'm')) fail();
    const names = payload.joint_names;
    if (!Array.isArray(names) || names.length !== jointDefs.length
        || new Set(names).size !== names.length
        || names.some((name) => !jointDefs.some((def) => def.name === name))) fail();
    const points = payload.waypoints;
    if (!Array.isArray(points) || !points.length || points.length > MAX_WAYPOINTS
        || (payload.count !== undefined && payload.count !== points.length)) fail();
    let previous = -1;
    return points.map((point) => {
      const time = point && point.time_from_start;
      if (!time || !Number.isSafeInteger(time.sec) || time.sec < 0
          || !Number.isSafeInteger(time.nanosec) || time.nanosec < 0 || time.nanosec >= 1e9) fail();
      const stamp = time.sec * 1e9 + time.nanosec;
      if (stamp <= previous || stamp > 1800 * 1e9) fail();
      previous = stamp;
      if (!Array.isArray(point.positions) || point.positions.length !== names.length) fail();
      const joints = {};
      names.forEach((name, index) => {
        const def = jointDefs.find((item) => item.name === name);
        const value = point.positions[index];
        if (typeof value !== 'number' || !Number.isFinite(value) || value < def.min || value > def.max) fail();
        joints[name] = value;
      });
      const tcp = point.tcp_ros;
      if (tcp !== undefined && (!tcp || !['x', 'y', 'z'].every((axis) => typeof tcp[axis] === 'number' && Number.isFinite(tcp[axis])))) fail();
      return { t: stamp / 1e6, joints, ...(tcp ? { tcp_ros: { x: tcp.x, y: tcp.y, z: tcp.z } } : {}) };
    });
  }

  function parseTeachingJson(text, jointDefs) {
    if (typeof text !== 'string' || new TextEncoder().encode(text).length > MAX_WAYPOINT_BYTES) {
      throw new Error('Waypoint JSON exceeds 4 MiB');
    }
    return validateTeachingPayload(JSON.parse(text), jointDefs);
  }

  return {
    MAX_WAYPOINT_BYTES,
    MAX_WAYPOINTS,
    parseTeachingJson,
    validateTeachingPayload,
    canExecuteRealMotion,
    canStartNewRecording,
    buildCoordinatedJointTarget,
    computeJointTrajectoryDuration,
    hasSimulationEvidence,
    selectTrajectoryTransport,
    shouldMirrorFeedback,
    shouldUseHardwareTargets,
    shouldUseLowLevelTrajectory
  };
});
