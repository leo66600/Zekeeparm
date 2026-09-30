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
    const velocity = Math.max(0.05, Number(maxVelocity) || 0.2);
    // A zero-endpoint-velocity cubic Hermite segment reaches 1.5*d/T.
    const velocityBoundedDuration = 1.5 * maxDisplacement / velocity;
    return Math.min(30, Math.max(1, Number(requestedDuration) || 0, velocityBoundedDuration));
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

  return {
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
