(function () {
const t = window.rebotI18n ? window.rebotI18n.t : (k) => k;
 const controlPolicy = window.ReBotControlPolicy;
 const DEG = Math.PI / 180;
  const NOMINAL_REACH = 0.65;
  const GRIPPER_COMMAND_MAX = 0.07;
  const GRIPPER_ANIMATION_MS = 520;
  const FAKE_GRASP_LOCAL_OFFSET = new THREE.Vector3(-0.05, 0, -0.02);
  const TABLE_CENTER_X = 0.42;
  const TABLE_WIDTH = 0.60;
  const TABLE_DEPTH = 0.52;
  const TABLE_SURFACE_Y = 0.03;
  const MUJOCO_OBJECT_COLORS = Object.freeze({
    red_cube: 'red',
    blue_block: 'blue',
    yellow_cylinder: 'yellow'
  });
  const ROS_TO_THREE_FRAME = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(1, 0, 0),
    -Math.PI / 2
  );
  const THREE_TO_ROS_FRAME = ROS_TO_THREE_FRAME.clone().invert();

 const jointDefs = [
    { name: 'joint1', label: 'joint.j1', min: -2.58, max: 2.58, home: 0, maxVelocity: 1.5 },
    { name: 'joint2', label: 'joint.j2', min: 0, max: 3.7, home: 0, maxVelocity: 1.5 },
    { name: 'joint3', label: 'joint.j3', min: 0, max: 3.7, home: 0, maxVelocity: 1.5 },
    { name: 'joint4', label: 'joint.j4', min: -1.57, max: 1.57, home: 0, maxVelocity: 1.5 },
    { name: 'joint5', label: 'joint.j5', min: -1.57, max: 1.57, home: 0, maxVelocity: 1.5 },
    { name: 'joint6', label: 'joint.j6', min: -1.57, max: 1.57, home: 0, maxVelocity: 1.5 },
    { name: 'gripper', label: 'joint.gripper', min: 0, max: GRIPPER_COMMAND_MAX, home: 0, unit: 'm' }
 ];

 const presets = {
    zero: { label: 'preset.zero', angles: [0, 0, 0, 0, 0, 0, 0] },
    ready: { label: 'preset.ready', angles: [2.37, 39.98, 46.60, -43.28, -1.14, -1.52, 0] }
 };

  let scene;  let camera;
  let renderer;
  let sceneResizeObserver;
  let controls;
  let robot;
  let robotFrame;
  let ghostRobot;
  let envelopeGroup;
  let workspacePlanarReach = NOMINAL_REACH;
  let workspaceVerticalReach = NOMINAL_REACH;
  let targetGhost;
  let tcpMarker;
  let dragErrorLine;
  let animation = null;
  let currentAngles = {};
  let targetAngles = {};
  const pendingTargetJoints = new Set();
  let moveStartAngles = {};
  let moveStart = 0;
  let moveDuration = 900;
  let gripperMotion = null;
  let carriedObject = null;
  let mujocoObjectFeedbackAt = 0;
  const taskObjects = new Map();
  let dragMode = false;
  let draggingTcp = false;
  let dragPlane = null;
  let dragTarget = new THREE.Vector3();
  let dragLastTime = 0;
  let dragPointerId = null;
  let dragTargetClamped = false;
  let dragSettling = false;
  let dragSettleStart = 0;
  let dragSettleLastTime = 0;
  const DRAG_SETTLE_TIMEOUT_MS = 1400;
  const DRAG_SETTLE_TARGET_ERROR = 0.002;
  let teachingRecording = false;
  let teachingStart = 0;
  let teachingLastSample = 0;
  let teachingWaypoints = [];
  let teachingPlayback = null;
  const TEACH_SAMPLE_INTERVAL_MS = 90;
  const TEACH_MIN_TCP_STEP = 0.004;
  const commandListeners = new Set();
  const axisLabelSprites = [];

  const els = {
    host: document.getElementById('scene-host'),
    loading: document.getElementById('loading-mask'),
    loadingText: document.getElementById('loading-text'),
    status: document.getElementById('load-status'),
    tcp: document.getElementById('tcp-position'),
    reach: document.getElementById('reach-state'),
    dragMarker: document.getElementById('drag-marker'),
    dragHud: document.getElementById('drag-hud'),
    dragStatus: document.getElementById('drag-status'),
    teachRecord: document.getElementById('teach-record'),
    teachReplay: document.getElementById('teach-replay'),
    teachExport: document.getElementById('teach-export'),
    teachClear: document.getElementById('teach-clear'),
    teachStatus: document.getElementById('teach-status'),
    teachExportText: document.getElementById('teach-export-text'),
    joints: document.getElementById('joint-controls'),
    presets: document.getElementById('preset-buttons'),
    presetExecute: document.getElementById('preset-execute'),
    presetStatus: document.getElementById('preset-status'),
    planTrajectory: document.getElementById('plan-trajectory'),
    toggleDrag: document.getElementById('toggle-drag')
  };

  let selectedPresetKey = '';
  let selectedPresetAngles = null;

  jointDefs.forEach((joint) => {
    currentAngles[joint.name] = joint.home;
    targetAngles[joint.name] = joint.home;
  });

  init();

  function init() {
    buildControls();
    setupScene();
    setupEvents();
    updateTeachingStatus();
    loadRobot();
    animate();
  }

  function buildControls() {
    Object.entries(presets).forEach(([key, preset]) => {
      const button = document.createElement('button');
     button.type = 'button';
      button.textContent = t(preset.label);
     button.dataset.preset = key;
     button.setAttribute('aria-pressed', 'false');
     button.addEventListener('click', () => selectPreset(key));
      els.presets.appendChild(button);
    });

    jointDefs.forEach((joint) => {
      const wrap = document.createElement('div');
      wrap.className = 'joint-control';

      const head = document.createElement('div');
     head.className = 'joint-head';
      const valueKey = joint.unit === 'm' ? 'joint.gripSuffix' : 'joint.radSuffix';
      head.innerHTML = `<strong>${t(joint.label)}</strong><span class="joint-value" id="${joint.name}-value">${t(valueKey, { val: '0.000' })}</span>`;

      const range = document.createElement('input');
      range.type = 'range';
      range.id = joint.name;
      const number = document.createElement('input');
      number.type = 'number';
      number.id = `${joint.name}-input`;
      number.className = 'joint-number';
      number.setAttribute('aria-label', `${t(joint.label)} target`);
      const error = document.createElement('span');
      error.id = `${joint.name}-input-error`;
      error.className = 'joint-input-error';
      error.setAttribute('aria-live', 'polite');
      if (joint.unit === 'm') {
        range.min = (joint.min * 1000).toFixed(0);
        range.max = (joint.max * 1000).toFixed(0);
        range.step = '1';
        range.value = (joint.home * 1000).toFixed(0);
        number.min = range.min;
        number.max = range.max;
        number.step = range.step;
        number.value = range.value;
      } else {
        range.min = joint.min.toFixed(3);
        range.max = joint.max.toFixed(3);
        range.step = '0.001';
        range.value = joint.home.toFixed(3);
        number.min = range.min;
        number.max = range.max;
        number.step = range.step;
        number.value = range.value;
      }
      number.dataset.lastValid = number.value;

      const displayToJointValue = (displayValue) => (
        joint.unit === 'm' ? Number(displayValue) / 1000 : Number(displayValue)
      );
      const formatDisplayValue = (value) => (
        joint.unit === 'm' ? (value * 1000).toFixed(0) : value.toFixed(3)
      );
      const formatLimitValue = (value) => (
        joint.unit === 'm' ? (value * 1000).toFixed(0) : value.toFixed(3)
      );
      const clearInputError = () => {
        number.classList.remove('invalid');
        number.setCustomValidity('');
        error.textContent = '';
      };
      const showInputError = (message) => {
        number.classList.add('invalid');
        number.setCustomValidity(message || '');
        error.textContent = message || '';
      };
      const rejectOutOfRangeInput = (value) => {
        if (value >= joint.min && value <= joint.max) return false;
        const unit = joint.unit === 'm' ? 'mm' : 'rad';
        showInputError(t('joint.inputInvalid', {
          min: formatLimitValue(joint.min),
          max: formatLimitValue(joint.max),
          unit
        }));
        number.value = number.dataset.lastValid || formatDisplayValue(currentAngles[joint.name] ?? joint.home);
        return true;
      };
      const previewJointValue = (displayValue, source) => {
        stopPath();
        if (source === 'number-input' && ['', '-', '.', '-.'].includes(displayValue)) {
          showInputError(t('joint.inputInvalidEmpty'));
          return false;
        }
        const value = displayToJointValue(displayValue);
        if (!Number.isFinite(value)) {
          showInputError(t('joint.inputInvalidEmpty'));
          return false;
        }
        if (source === 'number-input' && rejectOutOfRangeInput(value)) return false;
        clearInputError();
        if (source === 'number-input') number.dataset.lastValid = number.value;
        if (hardwareControlActive()) {
          updateTargetJoint(joint.name, value);
        } else {
          setJoint(joint.name, value, true, { source });
          syncGhostToRobot();
        }
        return true;
      };
      const commitJointValue = () => {
        if (number.classList.contains('invalid')) return;
        if (hardwareControlActive()) commitTargetJoint(joint.name);
      };

      range.addEventListener('input', () => {
        previewJointValue(range.value, 'slider');
      });
      range.addEventListener('change', commitJointValue);
      number.addEventListener('input', () => {
        previewJointValue(number.value, 'number-input');
      });
      number.addEventListener('change', commitJointValue);
      number.addEventListener('keydown', (event) => {
        if (event.key === 'Enter') {
          number.blur();
        }
      });

      const controlRow = document.createElement('div');
      controlRow.className = 'joint-input-row';
      controlRow.appendChild(range);
      controlRow.appendChild(number);

      wrap.appendChild(head);
      wrap.appendChild(controlRow);
      wrap.appendChild(error);
      els.joints.appendChild(wrap);
      updateJointLabel(joint.name);
    });
  }

  function setupScene() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x070a08);
    scene.fog = new THREE.Fog(0x070a08, 1.8, 5.2);

    camera = new THREE.PerspectiveCamera(48, getAspect(), 0.01, 20);
    resetCamera();

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(els.host.clientWidth, els.host.clientHeight);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.12;
    els.host.appendChild(renderer.domElement);

    controls = createOrbit(camera, renderer.domElement, new THREE.Vector3(0.18, 0.2, 0));

    robotFrame = new THREE.Group();
    robotFrame.rotation.x = -Math.PI / 2;
    scene.add(robotFrame);

    setupLights();
    createDirectionAxes();
    envelopeGroup = createEnvelope();
    scene.add(envelopeGroup);

    tcpMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.012, 24, 16),
      new THREE.MeshStandardMaterial({ color: 0x33d6b0, emissive: 0x0a4d3d, emissiveIntensity: 0.9 })
    );
    tcpMarker.visible = false;
    scene.add(tcpMarker);

    targetGhost = new THREE.Mesh(
      new THREE.SphereGeometry(0.018, 28, 18),
      new THREE.MeshBasicMaterial({ color: 0xf2a541, transparent: true, opacity: 0.85 })
    );
    targetGhost.visible = false;
    targetGhost.userData.active = false;
    scene.add(targetGhost);

    dragErrorLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineBasicMaterial({ color: 0xff6b5f, transparent: true, opacity: 0.82 })
    );
    dragErrorLine.visible = false;
    scene.add(dragErrorLine);
  }

  function setupLights() {
    scene.add(new THREE.HemisphereLight(0xfff8e8, 0x141613, 1.08));

    const key = new THREE.DirectionalLight(0xfff3dc, 1.95);
    key.position.set(1.4, 2.2, 1.2);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.camera.near = 0.1;
    key.shadow.camera.far = 6;
    key.shadow.camera.left = -1.4;
    key.shadow.camera.right = 1.4;
    key.shadow.camera.top = 1.4;
    key.shadow.camera.bottom = -1.4;
    scene.add(key);

    const side = new THREE.DirectionalLight(0xdde8e2, 0.38);
    side.position.set(-1, 0.6, -1.2);
    scene.add(side);

    const rim = new THREE.DirectionalLight(0xd8fff0, 0.72);
    rim.position.set(-0.8, 1.5, 1.8);
    scene.add(rim);
  }

  function createWorkbench() {
    const grid = new THREE.GridHelper(2.4, 48, 0x4d716a, 0x2c3a35);
    grid.position.y = 0;
    scene.add(grid);

    const table = new THREE.Group();
    const tableTexture = createTableTexture();
    const tableMat = new THREE.MeshStandardMaterial({
      color: 0xc9c5ae,
      map: tableTexture,
      roughness: 0.68,
      metalness: 0.04
    });
    const top = new THREE.Mesh(new THREE.BoxGeometry(TABLE_WIDTH, 0.03, TABLE_DEPTH), tableMat);
    top.position.set(TABLE_CENTER_X, TABLE_SURFACE_Y - 0.015, 0);
    top.castShadow = true;
    top.receiveShadow = true;
    top.userData.collisionKind = 'table';
    table.add(top);

    const edgeBand = new THREE.Mesh(
      new THREE.BoxGeometry(TABLE_WIDTH + 0.008, 0.012, TABLE_DEPTH + 0.008),
      new THREE.MeshStandardMaterial({ color: 0x454a43, roughness: 0.5, metalness: 0.16 })
    );
    edgeBand.position.set(TABLE_CENTER_X, 0.006, 0);
    edgeBand.castShadow = true;
    edgeBand.receiveShadow = true;
    table.add(edgeBand);

    const topOutline = new THREE.LineSegments(
      new THREE.EdgesGeometry(top.geometry, 28),
      new THREE.LineBasicMaterial({ color: 0xeee9d2, transparent: true, opacity: 0.34 })
    );
    topOutline.position.copy(top.position);
    table.add(topOutline);

    scene.add(table);
    scene.add(createTaskSpace());
  }

  function createTableTexture() {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 256;
    const ctx = canvas.getContext('2d');
    const image = ctx.createImageData(canvas.width, canvas.height);
    let seed = 601;
    for (let y = 0; y < canvas.height; y += 1) {
      for (let x = 0; x < canvas.width; x += 1) {
        seed = (seed * 1664525 + 1013904223) >>> 0;
        const grain = ((seed >>> 24) / 255 - 0.5) * 12;
        const streak = Math.sin(y * 0.23) * 2.4 + Math.sin(y * 0.057) * 2.0;
        const index = (y * canvas.width + x) * 4;
        image.data[index] = clamp(205 + grain + streak, 0, 255);
        image.data[index + 1] = clamp(201 + grain + streak, 0, 255);
        image.data[index + 2] = clamp(179 + grain + streak, 0, 255);
        image.data[index + 3] = 255;
      }
    }
    ctx.putImageData(image, 0, 0);
    const texture = new THREE.CanvasTexture(canvas);
    texture.wrapS = THREE.RepeatWrapping;
    texture.wrapT = THREE.RepeatWrapping;
    texture.repeat.set(3.2, 2.6);
    texture.anisotropy = Math.min(8, renderer.capabilities.getMaxAnisotropy());
    if ('colorSpace' in texture) texture.colorSpace = THREE.SRGBColorSpace;
    else if (THREE.sRGBEncoding !== undefined) texture.encoding = THREE.sRGBEncoding;
    return texture;
  }

  function createDirectionAxes() {
    const origin = new THREE.Vector3(0, 0.006, 0);
    addArrow(origin, new THREE.Vector3(1, 0, 0), 0xef5a4d, t('app.axisX'));
    addArrow(origin, new THREE.Vector3(0, 0, -1), 0x77c96b, t('app.axisY'));
    addArrow(origin, new THREE.Vector3(0, 1, 0), 0x5fa8ff, t('app.axisZ'));
  }

  function addArrow(origin, dir, color, label) {
    const arrow = new THREE.ArrowHelper(dir, origin, 0.18, color, 0.035, 0.012);
    scene.add(arrow);

    const sprite = makeTextSprite(label, color);
    sprite.position.copy(origin).add(dir.clone().multiplyScalar(0.23));
    sprite.position.y += dir.y === 0 ? 0.018 : 0;
    sprite.userData.autoHideAt = performance.now() + 3000;
    sprite.userData.fadeDuration = 900;
    axisLabelSprites.push(sprite);
    scene.add(sprite);
  }

  function createEnvelope() {
    const group = new THREE.Group();
    const mainMat = new THREE.LineBasicMaterial({ color: 0x33d6b0, transparent: true, opacity: 0.32 });
    const guideMat = new THREE.LineBasicMaterial({ color: 0x33d6b0, transparent: true, opacity: 0.18 });
    const radius = workspacePlanarReach;
    const heightLimit = workspaceVerticalReach;

    [0, 0.25, 0.5, 0.75, 0.95].forEach((ratio) => {
      const height = heightLimit * ratio;
      const ringRadius = radius * Math.sqrt(Math.max(0, 1 - ratio * ratio));
      group.add(makeCircleLine(ringRadius, height, height === 0 ? mainMat : guideMat));
    });

    for (let i = 0; i < 12; i += 1) {
      group.add(makeVerticalArc(radius, heightLimit, (i / 12) * Math.PI * 2, i % 3 === 0 ? mainMat : guideMat));
    }
    return group;
  }

  function makeCircleLine(radius, y, mat) {
    const points = [];
    for (let i = 0; i <= 128; i++) {
      const a = (i / 128) * Math.PI * 2;
      points.push(new THREE.Vector3(Math.cos(a) * radius, y, Math.sin(a) * radius));
    }
    return new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), mat);
  }

  function makeVerticalArc(radius, heightLimit, yaw, mat) {
    const points = [];
    for (let i = 0; i <= 72; i++) {
      const a = (i / 72) * Math.PI / 2;
      const r = Math.cos(a) * radius;
      const y = Math.sin(a) * heightLimit;
      points.push(new THREE.Vector3(Math.cos(yaw) * r, y, Math.sin(yaw) * r));
    }
    return new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), mat);
  }

  function createTaskSpace() {
    const group = new THREE.Group();
    addZone(group, 'sim.pickZone', 0.42, -0.13, 0x3f9f56, 0.57, 0.24);
    addZone(group, 'sim.placeZone', 0.42, 0.13, 0x777368, 0.57, 0.24);

    const objects = [
      {
        key: 'red',
        label: 'sim.redBlock',
        color: 0xd52b21,
        material: { roughness: 0.25, metalness: 0.03, clearcoat: 0.52, clearcoatRoughness: 0.18 },
        position: [0.34, -0.13, 0.055],
        tableY: 0.055,
        geometry: new THREE.BoxGeometry(0.05, 0.05, 0.05)
      },
      {
        key: 'blue',
        label: 'sim.blueBlock',
        color: 0x008fc5,
        material: { roughness: 0.38, metalness: 0.08, clearcoat: 0.28, clearcoatRoughness: 0.3 },
        position: [0.50, 0.11, 0.048],
        tableY: 0.048,
        geometry: new THREE.BoxGeometry(0.09, 0.036, 0.044)
      },
      {
        key: 'yellow',
        label: 'sim.cylinder',
        color: 0xeeb900,
        material: { roughness: 0.3, metalness: 0.12, clearcoat: 0.42, clearcoatRoughness: 0.22 },
        position: [0.44, -0.02, 0.065],
        tableY: 0.065,
        geometry: new THREE.CylinderGeometry(0.022, 0.022, 0.07, 24)
      }
    ];

    objects.forEach((object) => {
      const item = new THREE.Mesh(
        object.geometry,
        new THREE.MeshPhysicalMaterial({
          color: object.color,
          emissive: object.color,
          emissiveIntensity: 0.035,
          ...object.material
        })
      );
      const [rosX, rosY, rosZ] = object.position;
      item.position.set(rosX, rosZ, -rosY);
      item.castShadow = true;
      item.receiveShadow = true;
      item.userData.clickTarget = true;
      item.userData.targetKind = 'object';
      item.userData.targetLabel = object.label;
      item.userData.targetColor = object.key;
      item.geometry.computeBoundingBox();
      const size = new THREE.Vector3();
      item.geometry.boundingBox.getSize(size);
      item.userData.halfSize = size.multiplyScalar(0.5);
      item.userData.tableY = TABLE_SURFACE_Y + item.userData.halfSize.y;
      item.userData.collisionKind = 'task-object';
      item.userData.restPosition = item.position.clone();
      taskObjects.set(object.key, item);
      group.add(item);

      const outline = new THREE.LineSegments(
        new THREE.EdgesGeometry(object.geometry, 32),
        new THREE.LineBasicMaterial({ color: 0x17201d, transparent: true, opacity: 0.42 })
      );
      outline.renderOrder = 2;
      item.add(outline);
    });
    return group;
  }

  function addZone(group, label, rosX, rosY, color, width, depth) {
    const zone = new THREE.Mesh(
      new THREE.BoxGeometry(width || 0.18, 0.004, depth || 0.18),
      new THREE.MeshStandardMaterial({ color, roughness: 0.76, metalness: 0.025 })
    );
    // Keep the decorative zone almost flush with the table collision surface.
    // Raising it by its full thickness makes correctly resting objects appear
    // to penetrate the zone by roughly 4 mm.
    zone.position.set(rosX, TABLE_SURFACE_Y - 0.002 + 0.0002, -rosY);
    zone.receiveShadow = true;
    zone.userData.clickTarget = true;
    zone.userData.targetKind = 'zone';
    zone.userData.targetLabel = label;
    group.add(zone);

    const border = new THREE.LineSegments(
      new THREE.EdgesGeometry(zone.geometry),
      new THREE.LineBasicMaterial({ color: 0xd8d5c3, transparent: true, opacity: 0.52 })
    );
    border.position.copy(zone.position);
    group.add(border);

  }

  function loadRobot() {
    if (typeof URDFLoader === 'undefined') {
      failLoad('URDFLoader is not loaded.');
      return;
    }

    const manager = new THREE.LoadingManager();
    const modelVersion = Date.now().toString(36);
    manager.setURLModifier((assetUrl) => {
      const resolved = new URL(assetUrl, window.location.href);
      if (resolved.origin === window.location.origin && resolved.pathname.startsWith('/api/')) {
        resolved.searchParams.set('modelVersion', modelVersion);
        return resolved.href;
      }
      return assetUrl;
    });
    manager.onProgress = (url, loaded, total) => {
      els.loadingText.textContent = `Loading model ${Math.round((loaded / Math.max(total, 1)) * 100)}%`;
    };
    manager.onLoad = () => {
      if (!robot) return;
      finishRobotLoad();
    };

    const loader = new URDFLoader(manager);
    loader.parseCollision = true;
    loader.packages = {
      zekeep_bringup: `${window.location.origin}/api`
    };

    loader.load('/api/urdf', (loadedRobot) => {
      robot = loadedRobot;
      robotFrame.add(robot);
    }, undefined, (error) => {
      failLoad(`URDF load failed: ${error && error.message ? error.message : error}`);
    });
  }

  async function finishRobotLoad() {
    styleRobot(robot, false);
    createGhostRobot();
    jointDefs.forEach((joint) => {
      setJoint(joint.name, joint.home, false, { source: 'ros', emit: false });
    });
    estimateWorkspaceEnvelope();
    rebuildEnvelope();
    syncGhostToRobot();
    updateReadyState();
  }

  function estimateWorkspaceEnvelope() {
    if (!robot) return;

    const savedAngles = { ...currentAngles };
    let maxPlanar = NOMINAL_REACH;
    let maxVertical = NOMINAL_REACH;
    const movableJoints = jointDefs.filter((joint) => joint.name !== 'gripper');

    for (let i = 0; i < 960; i += 1) {
      const sample = { ...savedAngles };
      movableJoints.forEach((joint, index) => {
        const t = seededUnit(i + 1, index + 3);
        sample[joint.name] = joint.min + (joint.max - joint.min) * t;
      });
      applyRobotAngles(robot, sample);
      robot.updateMatrixWorld(true);

      const pos = getTcpPosition(robot);
      if (!pos) continue;
      maxPlanar = Math.max(maxPlanar, Math.sqrt(pos.x * pos.x + pos.z * pos.z));
      maxVertical = Math.max(maxVertical, Math.max(0, pos.y));
    }

    applyRobotAngles(robot, savedAngles);
    robot.updateMatrixWorld(true);
    workspacePlanarReach = clamp(Math.ceil(maxPlanar * 100) / 100, NOMINAL_REACH, 1.2);
    workspaceVerticalReach = clamp(Math.ceil(maxVertical * 100) / 100, NOMINAL_REACH, 1.2);
  }

  function seededUnit(a, b) {
    const x = Math.sin(a * 12.9898 + b * 78.233) * 43758.5453;
    return x - Math.floor(x);
  }

  function rebuildEnvelope() {
    if (!scene) return;
    const wasVisible = envelopeGroup ? envelopeGroup.visible : true;
    if (envelopeGroup) scene.remove(envelopeGroup);
    envelopeGroup = createEnvelope();
    const toggle = document.getElementById('toggle-envelope');
    envelopeGroup.visible = toggle ? toggle.checked && wasVisible : wasVisible;
    scene.add(envelopeGroup);
  }

  function styleRobot(root, ghost) {
    root.traverse((child) => {
      if (!child.isMesh) return;
      if (child.isURDFCollider || child.parent?.isURDFCollider) {
        child.visible = false;
        child.userData.collisionOnly = true;
        return;
      }
      child.castShadow = !ghost;
      child.receiveShadow = !ghost;

      if (ghost) {
        child.material = new THREE.MeshStandardMaterial({
          color: 0x33d6b0,
          roughness: 0.28,
          metalness: 0.05,
          transparent: true,
          opacity: 0.22,
          side: THREE.DoubleSide
        });
        return;
      }

    });
    Object.values(root.colliders || {}).forEach((collider) => {
      collider.visible = false;
    });
  }

  function createGhostRobot() {
    if (!robot) return;
    ghostRobot = robot.clone(true);
    styleRobot(ghostRobot, true);
    ghostRobot.visible = document.getElementById('toggle-ghost').checked;
    robotFrame.add(ghostRobot);
  }

  function updateReadyState() {
    els.status.classList.add('ready');
    els.status.lastChild.textContent = ' Ready';
    els.loading.classList.add('hidden');
  }

  function failLoad(message) {
    els.status.lastChild.textContent = ' Load failed';
    els.loadingText.textContent = message;
  }

  function setupEvents() {
    window.addEventListener('resize', resize);
    if (window.ResizeObserver) {
      sceneResizeObserver = new ResizeObserver(() => resize());
      sceneResizeObserver.observe(els.host);
    }
    document.getElementById('reset-camera').addEventListener('click', resetCamera);
    document.getElementById('play-path')?.addEventListener('click', playPath);
    document.getElementById('stop-path')?.addEventListener('click', () => {
      stopPath();
      teachingPlayback = null;
      updateTeachingStatus();
    });
    if (els.planTrajectory) els.planTrajectory.addEventListener('click', generateTrajectory);
    if (els.presetExecute) els.presetExecute.addEventListener('click', executeSelectedPreset);
    if (els.toggleDrag) els.toggleDrag.addEventListener('click', toggleDragMode);
    if (els.teachRecord) els.teachRecord.addEventListener('click', toggleTeachingRecord);
    if (els.teachReplay) els.teachReplay.addEventListener('click', replayTeaching);
    if (els.teachExport) els.teachExport.addEventListener('click', exportTeachingWaypoints);
    if (els.teachClear) els.teachClear.addEventListener('click', clearTeaching);
    if (els.dragMarker) {
      els.dragMarker.addEventListener('pointerdown', startTcpDrag);
    }
    window.addEventListener('pointermove', moveTcpDrag);
    window.addEventListener('pointerup', endTcpDrag);
    window.addEventListener('pointercancel', endTcpDrag);
    document.getElementById('open-gripper')?.addEventListener('click', () => setGripperWidth(GRIPPER_COMMAND_MAX));
    document.getElementById('close-gripper')?.addEventListener('click', () => setGripperWidth(0));

    document.getElementById('toggle-envelope').addEventListener('change', (event) => {
      envelopeGroup.visible = event.target.checked;
    });
    document.getElementById('toggle-ghost').addEventListener('change', (event) => {
      if (ghostRobot) ghostRobot.visible = event.target.checked;
      if (targetGhost) targetGhost.visible = false;
    });
  }

  function applyPreset(key, immediate) {
    const preset = presets[key];
    if (!preset) return;
    const next = presetAngles(preset);
    if (hardwareControlActive()) {
      jointDefs.forEach((joint) => pendingTargetJoints.add(joint.name));
      targetAngles = { ...next };
      updateGhostTarget(targetAngles);
      emitJointBatch(next, 'preset', t(preset.label));
      return;
    }
    moveToAngles(next, immediate ? 1 : 850, {
      source: immediate ? 'init' : 'preset',
      label: t(preset.label),
      emitBatch: !immediate
    });
  }

  function presetAngles(preset) {
    const next = {};
    jointDefs.forEach((joint, index) => {
      const raw = preset.angles[index] || 0;
      next[joint.name] = clamp(joint.unit === 'm' ? raw / 1000 : raw * DEG, joint.min, joint.max);
    });
    return next;
  }

  function selectPreset(key) {
    const preset = presets[key];
    if (!preset) return;
    stopPath();
    selectedPresetKey = key;
    selectedPresetAngles = presetAngles(preset);
    moveToAngles(selectedPresetAngles, 850, {
      source: 'preset-preview',
      label: t(preset.label),
      emitBatch: false,
      localOnly: true
    });
    Array.from(els.presets.children).forEach((button) => {
      const selected = button.dataset.preset === key;
      button.classList.toggle('primary-btn', selected);
      button.setAttribute('aria-pressed', String(selected));
    });
    if (els.presetStatus) els.presetStatus.textContent = t('preset.selected', {name: t(preset.label)});
  }

  function executeSelectedPreset() {
    if (!selectedPresetKey || !selectedPresetAngles) {
      if (els.presetStatus) els.presetStatus.textContent = t('preset.noneSelected');
      return;
    }
    const preset = presets[selectedPresetKey];
    const label = t(preset.label);
    if (hardwareControlActive()) {
      jointDefs.forEach((joint) => pendingTargetJoints.add(joint.name));
      targetAngles = { ...selectedPresetAngles };
      updateGhostTarget(targetAngles);
      emitJointBatch(selectedPresetAngles, 'preset-execute', label);
      if (els.presetStatus) els.presetStatus.textContent = t('preset.sent', {name: label});
      return;
    }
    moveToAngles(selectedPresetAngles, 850, {
      source: 'preset-execute',
      label,
      emitBatch: false,
      localOnly: true
    });
    if (els.presetStatus) els.presetStatus.textContent = t('preset.simulated', {name: label});
  }

  function setJoint(name, rad, fromUser, options) {
    const def = jointDefs.find((item) => item.name === name);
    if (!def) return;
    const value = clamp(rad, def.min, def.max);
    currentAngles[name] = value;
    const source = options && options.source ? options.source : (fromUser ? 'user' : 'sim');
    if (source === 'ros') {
      if (pendingTargetJoints.has(name) && Math.abs((targetAngles[name] ?? value) - value) < 0.02) {
        pendingTargetJoints.delete(name);
      }
      if (!pendingTargetJoints.has(name)) targetAngles[name] = value;
    }

    if (name === 'gripper') {
      const fingerTravel = (value / GRIPPER_COMMAND_MAX) * 0.035;
      setUrdfJointValue(robot, 'gripper_joint', fingerTravel);
      setUrdfJointValue(robot, 'right_joint', fingerTravel);
      setUrdfJointValue(ghostRobot, 'gripper_joint', fingerTravel);
      setUrdfJointValue(ghostRobot, 'right_joint', fingerTravel);
    }

    const joint = getJoint(robot, name);
    if (joint) {
      if (typeof joint.setJointValue === 'function') {
        joint.setJointValue(value);
      } else if (typeof joint.setAngle === 'function') {
        joint.setAngle(value);
      }
    }

    const displayValue = def.unit === 'm' ? (value * 1000).toFixed(0) : value.toFixed(3);
    const slider = document.getElementById(name);
    if (slider && (!fromUser || source === 'number-input')) slider.value = displayValue;
    const number = document.getElementById(`${name}-input`);
    if (number) {
      number.dataset.lastValid = displayValue;
      if (!fromUser || source !== 'number-input') number.value = displayValue;
      number.classList.remove('invalid');
      number.setCustomValidity('');
      const error = document.getElementById(`${name}-input-error`);
      if (error) error.textContent = '';
    }
    updateJointLabel(name);

    if (source !== 'ros' && !(options && options.emit === false)) {
      emitCommand({ type: 'joint', name, value, source, stamp: performance.now() });
    }
    if (source === 'ros') updateGhostTarget(targetAngles);
  }

  function setUrdfJointValue(root, name, value) {
    const joint = getJoint(root, name);
    if (!joint) return;
    if (typeof joint.setJointValue === 'function') {
      joint.setJointValue(value);
    } else if (typeof joint.setAngle === 'function') {
      joint.setAngle(value);
    }
  }

  function setGhostJoint(name, rad) {
    if (name === 'gripper') {
      const fingerTravel = (clamp(rad, 0, GRIPPER_COMMAND_MAX) / GRIPPER_COMMAND_MAX) * 0.035;
      setUrdfJointValue(ghostRobot, 'gripper_joint', fingerTravel);
      setUrdfJointValue(ghostRobot, 'right_joint', fingerTravel);
      return;
    }
    const joint = getJoint(ghostRobot, name);
    if (!joint) return;
    if (typeof joint.setJointValue === 'function') {
      joint.setJointValue(rad);
    } else if (typeof joint.setAngle === 'function') {
      joint.setAngle(rad);
    }
  }

  function getJoint(root, name) {
    if (!root) return null;
    if (root.joints && root.joints[name]) return root.joints[name];
    return root.getObjectByName(name);
  }

  function moveToAngles(nextAngles, duration, options) {
    teachingPlayback = null;
    if (hardwareControlActive() && !(options && options.localOnly)) {
      targetAngles = { ...targetAngles, ...nextAngles };
      jointDefs.forEach((joint) => pendingTargetJoints.add(joint.name));
      updateGhostTarget(targetAngles);
      if (options && options.emitBatch) {
        emitJointBatch(nextAngles, options.source || 'trajectory-target', options.label || '');
      }
      return;
    }
    moveStartAngles = { ...currentAngles };
    targetAngles = { ...nextAngles };
    moveStart = performance.now();
    moveDuration = Math.max(duration || 850, 1);
    updateGhostTarget(nextAngles);
    if (options && options.emitBatch) {
      emitJointBatch(nextAngles, options.source || 'trajectory-target', options.label || '');
    }
  }

  function updateMotion(now) {
   if (!moveStart) return;
    const u = clamp((now - moveStart) / moveDuration, 0, 1);
    const eased = u < 0.5 ? 4 * u * u * u : 1 - Math.pow(-2 * u + 2, 3) / 2;
    jointDefs.forEach((joint) => {
      const start = moveStartAngles[joint.name] ?? currentAngles[joint.name];
      const end = targetAngles[joint.name] ?? start;
      setJoint(joint.name, start + (end - start) * eased, false, { source: 'trajectory', emit: false });
    });
    if (u >= 1) moveStart = 0;
  }

  function updateGhostTarget(angles) {
    if (!ghostRobot) return;
    jointDefs.forEach((joint) => setGhostJoint(joint.name, angles[joint.name] ?? 0));
    ghostRobot.updateMatrixWorld(true);

    const pos = getTcpPosition(ghostRobot);
    if (pos) {
      targetGhost.position.copy(pos);
      targetGhost.userData.active = true;
      targetGhost.visible = false;
    }
  }

  function syncGhostToRobot() {
    if (!ghostRobot) return;
    jointDefs.forEach((joint) => setGhostJoint(joint.name, currentAngles[joint.name] ?? 0));
    ghostRobot.updateMatrixWorld(true);
  }

  function generateTrajectory() {
    if (!robot) return;
    const hardwareControl = hardwareControlActive();
    stopPath();
    draggingTcp = false;
    dragSettling = false;
    const destination = { ...currentAngles };
    const readyPreset = presets.ready;
    const readyAngles = {};

    jointDefs.forEach((joint, index) => {
      const raw = readyPreset.angles[index] || 0;
      readyAngles[joint.name] = clamp(joint.unit === 'm' ? raw / 1000 : raw * DEG, joint.min, joint.max);
      setJoint(joint.name, readyAngles[joint.name], false, { source: 'ros' });
    });

    syncGhostToRobot();
    updateGhostTarget(destination);
    moveToAngles(destination, 1200, {
      source: 'plan-current',
      label: t('adv.plan'),
      emitBatch: hardwareControl,
      localOnly: !hardwareControl
    });
    if (hardwareControl) {
      setDragStatus('实机轨迹已发送');
    } else {
      setDragStatus(t('sim.generatedReady'));
    }
  }

  function toggleDragMode() {
    dragMode = !dragMode;
    draggingTcp = false;
    dragSettling = false;
    dragLastTime = 0;

    if (els.toggleDrag) {
      els.toggleDrag.textContent = dragMode ? t('sim.exitDrag') : t('adv.drag');
      els.toggleDrag.classList.toggle('active', dragMode);
    }
    if (els.dragMarker) {
      els.dragMarker.classList.toggle('active', dragMode);
      els.dragMarker.classList.remove('dragging');
      if (!dragMode) els.dragMarker.style.display = 'none';
    }
    if (els.dragHud) {
      els.dragHud.classList.toggle('active', dragMode);
    }

    const pos = getTcpPosition(robot);
    if (pos) {
      dragTarget.copy(pos);
      showTargetGhost(pos);
    }
    updateDragErrorLine();
    setDragStatus(dragMode ? t('sim.dragGreen') : t('app.dragDisabled'));
    updateDragMarker();
  }

  function startTcpDrag(event) {
    if (!dragMode || !robot) return;
    event.preventDefault();
    event.stopPropagation();
    draggingTcp = true;
    dragSettling = false;
    dragTargetClamped = false;
    stopPath();
    moveStart = 0;
    dragPointerId = event.pointerId;
    dragLastTime = performance.now();
    dragPlane = createDragPlane();
    recordTeachingWaypoint(true);
    if (els.dragMarker) {
      els.dragMarker.classList.add('dragging');
      els.dragMarker.setPointerCapture(event.pointerId);
    }
    moveTcpDrag(event);
  }

  function moveTcpDrag(event) {
    if (!draggingTcp || !dragPlane || !robot) return;
    dragSettling = false;
    const hit = screenToDragPlane(event.clientX, event.clientY, dragPlane);
    if (!hit) return;

    const boundedTarget = clampToWorkspaceEnvelope(hit);
    dragTarget.copy(boundedTarget.point);
    dragTargetClamped = boundedTarget.clamped;
    showTargetGhost(dragTarget, dragTargetClamped);
    emitTcpTarget(dragTarget, 'drag', t('sim.tcpDrag'), dragTargetClamped);

    const now = performance.now();
    const dt = Math.min(0.05, Math.max(0.012, (now - dragLastTime) / 1000 || 0.016));
    dragLastTime = now;

    let result = null;
    const substeps = Math.max(1, Math.ceil(dt / 0.016));
    for (let i = 0; i < substeps; i += 1) {
      result = IKSolver.servoStep(dragTarget, dt / substeps);
    }

    syncGhostToRobot();
    recordTeachingWaypoint(false);
    updateDragMarker();
    updateDragErrorLine();
   if (result) {
      setDragStatus(`${dragTargetClamped ? t('sim.edgeSnap') : ''}${t('sim.errorMm', { mm: (result.error * 1000).toFixed(1) })}`);
   }
  }

  function endTcpDrag(event) {
    if (!draggingTcp) return;
    draggingTcp = false;
    dragPlane = null;
    dragPointerId = null;
    const releasedTcp = getTcpPosition(robot);
    if (releasedTcp && releasedTcp.distanceTo(dragTarget) > DRAG_SETTLE_TARGET_ERROR) {
      dragSettling = true;
      dragSettleStart = performance.now();
     dragSettleLastTime = dragSettleStart;
      setDragStatus(t('sim.converging', { mm: (releasedTcp.distanceTo(dragTarget) * 1000).toFixed(1) }));
    }
    if (els.dragMarker) {
      els.dragMarker.classList.remove('dragging');
      if (event && els.dragMarker.hasPointerCapture(event.pointerId)) {
        els.dragMarker.releasePointerCapture(event.pointerId);
      }
    }
    recordTeachingWaypoint(true);
    if (dragSettling) return;
    dragTargetClamped = false;
    const tcp = getTcpPosition(robot);
   if (tcp) {
      setDragStatus(t('sim.doneMm', { mm: (tcp.distanceTo(dragTarget) * 1000).toFixed(1) }));
    }
  }

  function updateDragMarker() {
    if (!dragMode || !els.dragMarker || !camera || !robot) return;
    const pos = (draggingTcp || dragSettling) ? dragTarget : getTcpPosition(robot);
    if (!pos) return;

    const hostRect = els.host.getBoundingClientRect();
    const viewportRect = document.getElementById('viewport').getBoundingClientRect();
    const projected = pos.clone().project(camera);
    const x = hostRect.left - viewportRect.left + ((projected.x + 1) / 2) * hostRect.width;
    const y = hostRect.top - viewportRect.top + ((1 - projected.y) / 2) * hostRect.height;

    els.dragMarker.style.left = `${x}px`;
    els.dragMarker.style.top = `${y}px`;
    els.dragMarker.style.display = projected.z < 1 ? 'block' : 'none';
  }

  function updateDragSettling(now) {
    if (!dragMode || !dragSettling || draggingTcp || !robot) return;

    const dt = Math.min(0.05, Math.max(0.012, (now - dragSettleLastTime) / 1000 || 0.016));
    dragSettleLastTime = now;

    let result = null;
    const substeps = Math.max(1, Math.ceil(dt / 0.016));
    for (let i = 0; i < substeps; i += 1) {
      result = IKSolver.servoStep(dragTarget, dt / substeps, { source: 'drag-settle' });
    }

    syncGhostToRobot();
    recordTeachingWaypoint(false);
    showTargetGhost(dragTarget, dragTargetClamped);
    emitTcpTarget(dragTarget, 'drag-settle', t('sim.tcpConverge'), dragTargetClamped);
    updateDragErrorLine();

    const elapsed = now - dragSettleStart;
    const fallbackTcp = result ? null : getTcpPosition(robot);
    const error = result ? result.error : (fallbackTcp ? fallbackTcp.distanceTo(dragTarget) : 0);
    if ((result && result.reached) || error <= DRAG_SETTLE_TARGET_ERROR) {
      dragSettling = false;
      dragTargetClamped = false;
     updateDragErrorLine();
      setDragStatus(t('sim.doneMm', { mm: (error * 1000).toFixed(1) }));
    } else if (elapsed >= DRAG_SETTLE_TIMEOUT_MS) {
      dragSettling = false;
      dragTargetClamped = false;
      updateDragErrorLine();
      setDragStatus(t('sim.bestEffortMm', { mm: (error * 1000).toFixed(1) }));
    } else {
      setDragStatus(t('sim.converging', { mm: (error * 1000).toFixed(1) }));
    }
  }

  function createDragPlane() {
    const tcp = getTcpPosition(robot) || new THREE.Vector3();
    const normal = new THREE.Vector3();
    camera.getWorldDirection(normal);
    return new THREE.Plane().setFromNormalAndCoplanarPoint(normal, tcp);
  }

  function screenToDragPlane(clientX, clientY, plane) {
    const rect = els.host.getBoundingClientRect();
    const mouse = new THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1
    );
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(mouse, camera);
    const hit = new THREE.Vector3();
    return raycaster.ray.intersectPlane(plane, hit) ? hit : null;
  }

  function clampToWorkspaceEnvelope(pos) {
    const point = pos.clone();
    let clamped = false;
    const radius = Math.max(workspacePlanarReach, 0.05);
    const heightLimit = Math.max(workspaceVerticalReach, 0.05);

    if (point.y < 0) {
      point.y = 0;
      clamped = true;
    } else if (point.y > heightLimit) {
      point.y = heightLimit;
      clamped = true;
    }

    const verticalRatio = clamp(point.y / heightLimit, 0, 1);
    const planarLimit = Math.max(0.03, radius * Math.sqrt(Math.max(0, 1 - verticalRatio * verticalRatio)));
    const planar = Math.sqrt(point.x * point.x + point.z * point.z);
    if (planar > planarLimit) {
      const scale = planarLimit / planar;
      point.x *= scale;
      point.z *= scale;
      clamped = true;
    }

    return { point, clamped };
  }

  function updateDragErrorLine() {
    if (!dragErrorLine || !robot) return;
    const active = dragMode && (draggingTcp || dragSettling || dragTargetClamped);
    const tcp = active ? getTcpPosition(robot) : null;
    if (!active || !tcp) {
      dragErrorLine.visible = false;
      return;
    }

    const error = tcp.distanceTo(dragTarget);
    if (error < 0.001) {
      dragErrorLine.visible = false;
      return;
    }

    dragErrorLine.geometry.setFromPoints([tcp, dragTarget]);
    dragErrorLine.material.opacity = clamp(error / 0.08, 0.28, 0.9);
    dragErrorLine.visible = true;
  }

  function showTargetGhost(pos, clamped) {
    if (!targetGhost || !pos) return;
    targetGhost.position.copy(pos);
    targetGhost.userData.active = true;
    targetGhost.userData.clamped = !!clamped;
    if (targetGhost.material && targetGhost.material.color) {
      targetGhost.material.color.set(clamped ? 0xff6b5f : 0xf2a541);
      targetGhost.material.opacity = clamped ? 0.95 : 0.85;
    }
    targetGhost.visible = false;
  }

  function emitTcpTarget(pos, source, label, clamped) {
    if (!pos) return;
    emitCommand({
      type: 'tcp-target',
      target_ros: threeToRos(pos),
      source: source || 'target',
      label: label || 'TCP target',
      clamped: !!clamped,
      stamp: performance.now()
    });
  }

  function setDragStatus(text) {
    if (els.dragStatus) els.dragStatus.textContent = text;
  }

  function toggleTeachingRecord() {
    if (teachingRecording) {
      teachingRecording = false;
      recordTeachingWaypoint(true);
    } else {
      const hasExistingPath = teachingWaypoints.length > 0;
      const overwriteConfirmed = !hasExistingPath || window.confirm(
        '已有示教路径，开始新录制将覆盖旧路径。是否覆盖？'
      );
      if (!controlPolicy.canStartNewRecording({ hasExistingPath, overwriteConfirmed })) {
        updateTeachingStatus('已取消新录制，旧路径保持不变');
        return;
      }
      teachingWaypoints = [];
      teachingStart = performance.now();
      teachingLastSample = 0;
      teachingPlayback = null;
      teachingRecording = true;
      if (!dragMode) toggleDragMode();
      recordTeachingWaypoint(true);
      if (els.teachExportText) els.teachExportText.value = '';
    }
    updateTeachingStatus();
  }

  function recordTeachingWaypoint(force) {
    if (!teachingRecording || !robot) return;
    const now = performance.now();
    if (!force && now - teachingLastSample < TEACH_SAMPLE_INTERVAL_MS) return;

    const tcp = getTcpPosition(robot);
    if (!tcp) return;
    const last = teachingWaypoints[teachingWaypoints.length - 1];
    if (!force && last && last.tcp && new THREE.Vector3(last.tcp.x, last.tcp.y, last.tcp.z).distanceTo(tcp) < TEACH_MIN_TCP_STEP) {
      return;
    }

    teachingLastSample = now;
    const ros = threeToRos(tcp);
    teachingWaypoints.push({
      t: Math.max(0, now - teachingStart),
      joints: { ...currentAngles },
      tcp: { x: tcp.x, y: tcp.y, z: tcp.z },
      tcp_ros: { x: ros.x, y: ros.y, z: ros.z }
    });
    updateTeachingStatus();
  }

  function replayTeaching() {
    if (!teachingWaypoints.length || !robot) {
      updateTeachingStatus(t('sim.noReplay'));
      return;
    }
    teachingRecording = false;
    stopPath();
    moveStart = 0;
    teachingPlayback = {
      points: teachingWaypoints.map((point) => ({ ...point, joints: { ...point.joints } })),
      index: 0,
      segmentStart: performance.now(),
      segmentDuration: 260,
      startAngles: { ...currentAngles }
    };
    updateTeachingStatus(t('sim.replaying'));
  }

  function updateTeachingPlayback(now) {
    if (!teachingPlayback) return;
    const point = teachingPlayback.points[teachingPlayback.index];
    if (!point) {
      teachingPlayback = null;
      stageTeachingReplayForHardware();
      updateTeachingStatus(t('sim.replayDone'));
      return;
    }

    const u = clamp((now - teachingPlayback.segmentStart) / teachingPlayback.segmentDuration, 0, 1);
    const eased = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
    jointDefs.forEach((joint) => {
      const start = teachingPlayback.startAngles[joint.name] ?? currentAngles[joint.name] ?? 0;
      const end = point.joints[joint.name] ?? start;
      setJoint(joint.name, start + (end - start) * eased, false, {
        source: 'teach-replay',
        emit: false
      });
    });

    if (u < 1) return;

    teachingPlayback.index += 1;
    if (teachingPlayback.index >= teachingPlayback.points.length) {
      teachingPlayback = null;
      syncGhostToRobot();
      stageTeachingReplayForHardware();
      updateTeachingStatus(t('sim.replayDone'));
      return;
    }

    const prev = teachingPlayback.points[teachingPlayback.index - 1];
    const next = teachingPlayback.points[teachingPlayback.index];
    teachingPlayback.startAngles = { ...currentAngles };
    teachingPlayback.segmentStart = now;
    teachingPlayback.segmentDuration = clamp(next.t - prev.t, 80, 900);
  }

  function stageTeachingReplayForHardware() {
    if (!teachingWaypoints.length) return;
    emitCommand({
      type: 'teaching-replay-complete',
      label: '示教轨迹',
      waypoints: teachingWaypoints.map((point) => ({
        ...point,
        joints: { ...point.joints },
        tcp_ros: { ...point.tcp_ros }
      })),
      stamp: performance.now()
    });
  }

  function exportTeachingWaypoints() {
    if (!teachingWaypoints.length) {
      updateTeachingStatus(t('sim.noExport'));
      return;
    }
    const jointNames = jointDefs.map((joint) => joint.name);
    const payload = {
      format: 'zekeep_ros_waypoints_v1',
      frame_id: 'base_link',
      joint_names: jointNames,
      count: teachingWaypoints.length,
      waypoints: teachingWaypoints.map((point) => ({
        time_from_start: {
          sec: Math.floor(point.t / 1000),
          nanosec: Math.round((point.t % 1000) * 1e6)
        },
        positions: jointNames.map((name) => point.joints[name] ?? 0),
        tcp_ros: point.tcp_ros
      }))
    };
    const text = JSON.stringify(payload, null, 2);
    if (els.teachExportText) {
      els.teachExportText.value = text;
      els.teachExportText.focus();
      els.teachExportText.select();
    }
    if (navigator.clipboard && navigator.clipboard.writeText) {
    navigator.clipboard.writeText(text).catch(() => {});
    }
    updateTeachingStatus(t('sim.exported', { n: teachingWaypoints.length }));
  }

  function clearTeaching() {
    teachingRecording = false;
    teachingPlayback = null;
    teachingWaypoints = [];
    if (els.teachExportText) els.teachExportText.value = '';
    updateTeachingStatus();
  }

  function updateTeachingStatus(message) {
    if (els.teachRecord) {
      els.teachRecord.textContent = teachingRecording ? t('sim.stopRecord') : t('teach.record');
      els.teachRecord.classList.toggle('active', teachingRecording);
    }
    if (!els.teachStatus) return;
    if (message) {
      els.teachStatus.textContent = message;
    } else if (teachingRecording) {
      els.teachStatus.textContent = t('sim.recording', { n: teachingWaypoints.length });
    } else if (teachingPlayback) {
      els.teachStatus.textContent = t('sim.replaying');
    } else if (teachingWaypoints.length) {
      const duration = teachingWaypoints[teachingWaypoints.length - 1].t / 1000;
      els.teachStatus.textContent = t('sim.recorded', { n: teachingWaypoints.length, sec: duration.toFixed(1) });
    } else {
      els.teachStatus.textContent = t('teach.status');
    }
  }

  function planTcpMoveTo(target, label, options) {
    stopPath();
    moveStart = 0;
    showTargetGhost(target);

    const start = { ...currentAngles };
    const solved = solveIKTarget(target, 240, 900);
    applyRobotAngles(robot, start);
    Object.entries(start).forEach(([name, value]) => {
      setJoint(name, value, false, { source: options && options.localOnly ? 'sim' : 'ros' });
    });

    if (!solved || !solved.angles) {
      setDragStatus(t('sim.unreachable', { label: t(label || 'sim.clickPoint') }));
      return;
    }

    emitTcpTarget(target, 'plan-target', t(label || 'sim.clickPoint'), false);
    moveToAngles({ ...currentAngles, ...solved.angles }, 900, options && options.localOnly ? { localOnly: true } : undefined);
    setDragStatus(t('sim.planTarget', { label: t(label || 'sim.clickPoint'), mm: (solved.error * 1000).toFixed(1) }));
  }

  function solveIKTarget(target, maxIter, timeoutMs) {
    const started = performance.now();
    let result = null;

    for (let i = 0; i < maxIter; i += 1) {
      result = IKSolver.servoStep(target, 0.016, { source: 'solver' });
      if (result && result.reached) break;
      if (performance.now() - started > timeoutMs) break;
    }

    return {
      angles: { ...currentAngles },
      error: result ? result.error : Infinity,
      reached: result ? result.reached : false
    };
  }

  function applyRobotAngles(root, angles) {
    if (!root) return;
    IKSolver.jointNames.forEach((name) => {
      const joint = getJoint(root, name);
      if (!joint) return;
      const value = angles[name] ?? 0;
      if (typeof joint.setJointValue === 'function') {
        joint.setJointValue(value);
      } else if (typeof joint.setAngle === 'function') {
        joint.setAngle(value);
      }
    });
    root.updateMatrixWorld(true);
  }

  const IKSolver = {
    jointNames: jointDefs.filter((joint) => joint.name !== 'gripper').map((joint) => joint.name),
    gain: 12,
    damping: 0.035,
    maxJointSpeed: 2.8,

    servoStep(target, dt, options) {
      if (!target || !robot || dt <= 0) return null;
      const current = getTcpPosition(robot);
      if (!current) return null;
      const error = new THREE.Vector3().subVectors(target, current);
      const errorNorm = error.length();
      if (errorNorm < 0.0015) return { error: errorNorm, reached: true };

      const stepError = error.multiplyScalar(Math.min(0.65, Math.max(0.08, this.gain * dt)));
      const jacobian = this.computeJacobian(currentAngles);
      const delta = this.solveDampedLeastSquares(jacobian, stepError);
      if (!delta) return { error: errorNorm, reached: false };

      this.jointNames.forEach((name, index) => {
        const def = jointDefs.find((joint) => joint.name === name);
        const limitedDelta = clamp(delta[index] || 0, -this.maxJointSpeed * dt, this.maxJointSpeed * dt);
        setJoint(name, clamp((currentAngles[name] || 0) + limitedDelta, def.min, def.max), false, { source: options && options.source ? options.source : 'drag' });
      });

      robot.updateMatrixWorld(true);
      const after = getTcpPosition(robot);
      const afterError = after ? after.distanceTo(target) : errorNorm;
      return { error: afterError, reached: afterError < 0.0015 };
    },

    computeJacobian(baseAngles) {
      const eps = 0.004;
      const saved = { ...baseAngles };
      const rows = [[], [], []];

      this.jointNames.forEach((name, index) => {
        const plus = { ...saved, [name]: (saved[name] || 0) + eps };
        const minus = { ...saved, [name]: (saved[name] || 0) - eps };

        applyRobotAngles(robot, plus);
        const plusPos = getTcpPosition(robot);
        applyRobotAngles(robot, minus);
        const minusPos = getTcpPosition(robot);

        rows[0][index] = plusPos && minusPos ? (plusPos.x - minusPos.x) / (2 * eps) : 0;
        rows[1][index] = plusPos && minusPos ? (plusPos.y - minusPos.y) / (2 * eps) : 0;
        rows[2][index] = plusPos && minusPos ? (plusPos.z - minusPos.z) / (2 * eps) : 0;
      });

      applyRobotAngles(robot, saved);
      return rows;
    },

    solveDampedLeastSquares(j, error) {
      const lambda2 = this.damping * this.damping;
      const a = [
        [
          dotRows(j[0], j[0]) + lambda2,
          dotRows(j[0], j[1]),
          dotRows(j[0], j[2])
        ],
        [
          dotRows(j[1], j[0]),
          dotRows(j[1], j[1]) + lambda2,
          dotRows(j[1], j[2])
        ],
        [
          dotRows(j[2], j[0]),
          dotRows(j[2], j[1]),
          dotRows(j[2], j[2]) + lambda2
        ]
      ];
      const y = solve3x3(a, [error.x, error.y, error.z]);
      if (!y) return null;
      return this.jointNames.map((name, index) => j[0][index] * y[0] + j[1][index] * y[1] + j[2][index] * y[2]);
    }
  };

  function dotRows(a, b) {
    return a.reduce((sum, value, index) => sum + value * (b[index] || 0), 0);
  }

  function solve3x3(a, b) {
    const det =
      a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
      a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
      a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if (Math.abs(det) < 1e-9) return null;

    const inv = [
      [
        (a[1][1] * a[2][2] - a[1][2] * a[2][1]) / det,
        (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / det,
        (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det
      ],
      [
        (a[1][2] * a[2][0] - a[1][0] * a[2][2]) / det,
        (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det,
        (a[0][2] * a[1][0] - a[0][0] * a[1][2]) / det
      ],
      [
        (a[1][0] * a[2][1] - a[1][1] * a[2][0]) / det,
        (a[0][1] * a[2][0] - a[0][0] * a[2][1]) / det,
        (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / det
      ]
    ];

    return [
      inv[0][0] * b[0] + inv[0][1] * b[1] + inv[0][2] * b[2],
      inv[1][0] * b[0] + inv[1][1] * b[1] + inv[1][2] * b[2],
      inv[2][0] * b[0] + inv[2][1] * b[1] + inv[2][2] * b[2]
    ];
  }

  function updateJointLabel(name) {
    const def = jointDefs.find((item) => item.name === name);
    const label = document.getElementById(`${name}-value`);
    if (!label || !def) return;
    if (def.unit === 'm') {
     const widthMm = currentAngles[name] * 1000;
      label.textContent = t('joint.gripSuffix', { val: widthMm.toFixed(0) });
      const readout = document.getElementById('gripper-width');
      if (readout) readout.textContent = t('joint.gripSuffix', { val: widthMm.toFixed(0) });
      return;
    }
    label.textContent = t('joint.radSuffix', { val: currentAngles[name].toFixed(3) });
  }

  function setGripperWidth(widthM) {
    const options = arguments[1] || {};
    const source = options.source || 'ui';
    const target = clamp(widthM, 0, GRIPPER_COMMAND_MAX);
    if (source === 'ros') {
      setJoint('gripper', target, false, { source: 'ros', emit: false });
      return;
    }
    if (hardwareControlActive() && !options.localOnly) {
      updateTargetJoint('gripper', target);
      commitTargetJoint('gripper');
      return;
    }
    stopPath();
    moveToAngles({ ...currentAngles, gripper: target }, 450);
  }

  function hardwareControlActive() {
    const control = document.getElementById('ros-control-enable');
    return controlPolicy.shouldUseHardwareTargets({
      connected: Boolean(window.reBotRos && window.reBotRos.connected),
      controlEnabled: Boolean(control && control.checked)
    });
  }

  function updateTargetJoint(name, value) {
    const def = jointDefs.find((joint) => joint.name === name);
    if (!def) return;
    const target = clamp(value, def.min, def.max);
    targetAngles[name] = target;
    pendingTargetJoints.add(name);
    setGhostJoint(name, target);
    updateGhostTarget(targetAngles);
  }

  function commitTargetJoint(name) {
    if (!pendingTargetJoints.has(name)) return;
    const value = targetAngles[name];
    pendingTargetJoints.delete(name);
    emitCommand({ type: 'joint', name, value, source: 'slider-release', stamp: performance.now() });
  }

  function emitCommand(command) {
    commandListeners.forEach((listener) => {
      try {
        listener({ ...command });
      } catch (error) {
        console.warn('Command listener failed:', error);
      }
    });
  }

  function emitJointBatch(angles, source, label) {
    const joints = {};
    jointDefs.forEach((joint) => {
      if (typeof angles[joint.name] === 'number') {
        joints[joint.name] = angles[joint.name];
      }
    });
    if (!Object.keys(joints).length) return;
    emitCommand({
      type: 'joint-batch',
      joints,
      source,
      label,
      stamp: performance.now()
    });
  }

  function playPath() {
    const sequence = ['ready', 'left', 'inspect', 'forward', 'right', 'ready'];
    animation = { sequence, index: 0, nextAt: 0 };
  }

  function stopPath() {
    animation = null;
  }

  function updatePath(now) {
    if (!animation || moveStart) return;
    if (now < animation.nextAt) return;
    applyPreset(animation.sequence[animation.index]);
    animation.index = (animation.index + 1) % animation.sequence.length;
    animation.nextAt = now + 1350;
  }

  function updateTcpHud() {
    const pos = getTcpPosition(robot);
    if (!pos) return;
    tcpMarker.position.copy(pos);
    tcpMarker.visible = false;

    const ros = threeToRos(pos);
    const planar = Math.sqrt(ros.x * ros.x + ros.y * ros.y);
    const spatial = Math.sqrt(ros.x * ros.x + ros.y * ros.y + ros.z * ros.z);
    els.tcp.textContent = `X ${mm(ros.x)} / Y ${mm(ros.y)} / Z ${mm(ros.z)}`;
    els.reach.textContent = t('sim.reachText', { planar: Math.round(planar * 1000), workspace: Math.round(workspacePlanarReach * 1000), spatial: Math.round(spatial * 1000) });
    els.reach.style.color = planar <= workspacePlanarReach ? '#d7fff4' : '#ffd1c9';
  }

  function getTcpPosition(root) {
    if (!root) return null;
    // The Six-axis hardware configuration uses link6 as its end-effector
    // frame. Keep optional end_link/grasp_tcp fallbacks for other URDFs.
    const link = root.getObjectByName('end_link')
      || root.getObjectByName('link6')
      || root.getObjectByName('grasp_tcp')
      || root;
    link.updateMatrixWorld(true);
    const pos = new THREE.Vector3();
    link.getWorldPosition(pos);
    return pos;
  }

  function getEndLink(root) {
    if (!root) return null;
    return root.getObjectByName('end_link')
      || root.getObjectByName('link6')
      || root.getObjectByName('grasp_tcp')
      || root;
  }

  function getTcpPose() {
    if (!robot) return null;
    const link = getEndLink(robot);
    if (!link) return null;
    link.updateMatrixWorld(true);
    const position = new THREE.Vector3();
    const orientation = new THREE.Quaternion();
    link.getWorldPosition(position);
    link.getWorldQuaternion(orientation);
    // robotFrame applies ROS_TO_THREE_FRAME to the URDF. Convert the TCP
    // orientation back to the ROS base_link frame before sending a Pose.
    const rosOrientation = THREE_TO_ROS_FRAME.clone().multiply(orientation).normalize();
    return {
      position: threeToRos(position),
      orientation: {
        x: rosOrientation.x,
        y: rosOrientation.y,
        z: rosOrientation.z,
        w: rosOrientation.w
      }
    };
  }

  function getFakeGraspPosition(root) {
    const link = getEndLink(root);
    if (!link) return null;
    link.updateMatrixWorld(true);
    return link.localToWorld(FAKE_GRASP_LOCAL_OFFSET.clone());
  }

  function updateCarriedObject() {
    if (!carriedObject || !carriedObject.mesh || !robot) return;
    if (hasFreshMujocoObjectFeedback()) return;
    const grip = getFakeGraspPosition(robot) || getTcpPosition(robot);
    if (!grip) return;
    carriedObject.mesh.position.lerp(grip, 0.55);
    enforceTableCollision(carriedObject.mesh);
  }

  function hasFreshMujocoObjectFeedback() {
    return mujocoObjectFeedbackAt > 0 && performance.now() - mujocoObjectFeedbackAt < 500;
  }

  function getObjectHalfSize(mesh) {
    if (mesh && mesh.userData && mesh.userData.halfSize) {
      return mesh.userData.halfSize;
    }
    if (!mesh || !mesh.geometry) return new THREE.Vector3();
    if (!mesh.geometry.boundingBox) mesh.geometry.computeBoundingBox();
    const size = new THREE.Vector3();
    mesh.geometry.boundingBox.getSize(size);
    mesh.userData.halfSize = size.multiplyScalar(0.5);
    return mesh.userData.halfSize;
  }

  function isOverTable(mesh) {
    const half = getObjectHalfSize(mesh);
    const minX = TABLE_CENTER_X - TABLE_WIDTH / 2;
    const maxX = TABLE_CENTER_X + TABLE_WIDTH / 2;
    const minZ = -TABLE_DEPTH / 2;
    const maxZ = TABLE_DEPTH / 2;
    return mesh.position.x + half.x >= minX
      && mesh.position.x - half.x <= maxX
      && mesh.position.z + half.z >= minZ
      && mesh.position.z - half.z <= maxZ;
  }

  function enforceTableCollision(mesh) {
    if (!mesh || !isOverTable(mesh)) return;
    const minCenterY = TABLE_SURFACE_Y + getObjectHalfSize(mesh).y;
    if (mesh.position.y < minCenterY) mesh.position.y = minCenterY;
  }

  function clampObjectToTable(mesh) {
    const half = getObjectHalfSize(mesh);
    mesh.position.x = clamp(
      mesh.position.x,
      TABLE_CENTER_X - TABLE_WIDTH / 2 + half.x,
      TABLE_CENTER_X + TABLE_WIDTH / 2 - half.x
    );
    mesh.position.z = clamp(
      mesh.position.z,
      -TABLE_DEPTH / 2 + half.z,
      TABLE_DEPTH / 2 - half.z
    );
    mesh.position.y = TABLE_SURFACE_Y + half.y;
  }

  function resolveTaskObjectCollisions(mesh) {
    if (!mesh) return;
    const half = getObjectHalfSize(mesh);
    for (let pass = 0; pass < 4; pass += 1) {
      let moved = false;
      taskObjects.forEach((other) => {
        if (!other || other === mesh || (carriedObject && carriedObject.mesh === other)) return;
        const otherHalf = getObjectHalfSize(other);
        const dx = mesh.position.x - other.position.x;
        const dz = mesh.position.z - other.position.z;
        const overlapX = half.x + otherHalf.x - Math.abs(dx);
        const overlapZ = half.z + otherHalf.z - Math.abs(dz);
        if (overlapX <= 0 || overlapZ <= 0) return;
        if (overlapX < overlapZ) {
          mesh.position.x += (dx < 0 ? -1 : 1) * (overlapX + 0.003);
        } else {
          mesh.position.z += (dz < 0 ? -1 : 1) * (overlapZ + 0.003);
        }
        clampObjectToTable(mesh);
        moved = true;
      });
      if (!moved) break;
    }
  }

  function settleTaskObject(mesh) {
    if (!mesh) return;
    clampObjectToTable(mesh);
    resolveTaskObjectCollisions(mesh);
    mesh.userData.tableY = mesh.position.y;
    mesh.userData.restPosition = mesh.position.clone();
  }

  function applyMujocoObjectStates(objects) {
    if (!Array.isArray(objects)) return;
    let updated = false;
    objects.forEach((state) => {
      const color = MUJOCO_OBJECT_COLORS[String(state && state.name || '')];
      const mesh = color ? taskObjects.get(color) : null;
      const position = state && state.position;
      if (!mesh || !Array.isArray(position) || position.length < 3) return;
      const rosX = Number(position[0]);
      const rosY = Number(position[1]);
      const rosZ = Number(position[2]);
      if (![rosX, rosY, rosZ].every(Number.isFinite)) return;
      mesh.position.set(rosX, rosZ, -rosY);

      const quat = state.quat_wxyz;
      if (Array.isArray(quat) && quat.length >= 4) {
        const rosQuat = new THREE.Quaternion(
          Number(quat[1]),
          Number(quat[2]),
          Number(quat[3]),
          Number(quat[0])
        );
        if ([rosQuat.x, rosQuat.y, rosQuat.z, rosQuat.w].every(Number.isFinite)) {
          mesh.quaternion.copy(ROS_TO_THREE_FRAME)
            .multiply(rosQuat.normalize())
            .multiply(THREE_TO_ROS_FRAME);
        }
      }
      mesh.userData.restPosition = mesh.position.clone();
      updated = true;
    });
    if (updated) mujocoObjectFeedbackAt = performance.now();
  }

  function getSceneCollisionMap() {
    const objects = {};
    taskObjects.forEach((mesh, color) => {
      const position = threeToRos(mesh.position);
      const half = getObjectHalfSize(mesh);
      objects[color] = {
        position,
        size: { x: half.x * 2, y: half.z * 2, z: half.y * 2 },
        carried: Boolean(carriedObject && carriedObject.mesh === mesh)
      };
    });
    return {
      frame: 'base_link',
      mapping: 'three(x,y,z)=ros(x,z,-y)',
      source: hasFreshMujocoObjectFeedback() ? 'mujoco_object_states' : 'web_fallback',
      table: {
        center: { x: TABLE_CENTER_X, y: 0, z: TABLE_SURFACE_Y - 0.015 },
        size: { x: TABLE_WIDTH, y: TABLE_DEPTH, z: 0.03 },
        surface_z: TABLE_SURFACE_Y
      },
      objects
    };
  }

  function attachObject(color) {
    const key = String(color || '').toLowerCase();
    const mesh = taskObjects.get(key);
    if (!mesh) return false;
    if (carriedObject && carriedObject.mesh !== mesh) {
      releaseObject({ settleOnTable: true });
    }
    carriedObject = { color: key, mesh };
    mesh.userData.fakeCarried = true;
    updateCarriedObject();
    return true;
  }

  function attachObjectAtPosition(position, maxDistance) {
    if (!position || ![position.x, position.y, position.z].every(Number.isFinite)) return false;
    const limit = Number.isFinite(maxDistance) ? maxDistance : 0.09;
    let nearest = null;
    let nearestDistance = Infinity;
    taskObjects.forEach((mesh, color) => {
      if (!mesh || (carriedObject && carriedObject.mesh === mesh)) return;
      const ros = threeToRos(mesh.position);
      const distance = Math.hypot(ros.x - position.x, ros.y - position.y, ros.z - position.z);
      if (distance < nearestDistance) {
        nearest = color;
        nearestDistance = distance;
      }
    });
    return nearest !== null && nearestDistance <= limit ? attachObject(nearest) : false;
  }

  function releaseObject(options) {
    if (!carriedObject || !carriedObject.mesh) {
      carriedObject = null;
      return false;
    }
    const mesh = carriedObject.mesh;
    mesh.userData.fakeCarried = false;
    if (!options || options.settleOnTable !== false) {
      if (!hasFreshMujocoObjectFeedback()) settleTaskObject(mesh);
    }
    carriedObject = null;
    return true;
  }

  function threeToRos(v) {
    return { x: v.x, y: -v.z, z: v.y };
  }

  function mm(value) {
    return t('sim.mmShort', { val: Math.round(value * 1000) });
  }

  function animate(now) {
    requestAnimationFrame(animate);
    const frameNow = now || performance.now();
    updateMotion(frameNow);
    updateGripperMotion(frameNow);
    updatePath(frameNow);
    updateTeachingPlayback(frameNow);
    updateDragSettling(frameNow);
    updateAxisLabelVisibility(frameNow);
    if (robot) {
      robot.updateMatrixWorld(true);
      updateTcpHud();
      updateCarriedObject();
      updateDragMarker();
      updateDragErrorLine();
    }
    if (controls) controls.update();
    renderer.render(scene, camera);
  }

  function updateAxisLabelVisibility(now) {
    axisLabelSprites.forEach((sprite) => {
      const hideAt = sprite.userData.autoHideAt || 0;
      const fadeDuration = sprite.userData.fadeDuration || 900;
      if (now <= hideAt) return;

      const progress = clamp((now - hideAt) / fadeDuration, 0, 1);
      const opacity = 1 - progress;
      sprite.material.opacity = opacity;
      sprite.scale.set(0.16 * (1 + progress * 0.12), 0.05 * (1 + progress * 0.12), 1);
      sprite.visible = opacity > 0.02;
    });
  }

  function resize() {
    camera.aspect = getAspect();
    camera.updateProjectionMatrix();
    renderer.setSize(els.host.clientWidth, els.host.clientHeight);
  }

  function resetCamera() {
    if (!camera) return;
    camera.position.set(-0.72, 0.48, 0.74);
    camera.lookAt(0.18, 0.18, 0);
    if (controls) {
      controls.target.set(0.18, 0.18, 0);
      controls.sync();
    }
  }

  function getAspect() {
    return Math.max(1, els.host.clientWidth) / Math.max(1, els.host.clientHeight);
  }

  function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
  }

  function makeTextSprite(text, color) {
    const canvas = document.createElement('canvas');
    canvas.width = 512;
    canvas.height = 160;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = 'rgba(17, 18, 17, 0.72)';
    roundRect(ctx, 10, 24, 492, 92, 14);
    ctx.fill();
    ctx.font = '700 38px "Microsoft YaHei", Arial';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = `#${color.toString(16).padStart(6, '0')}`;
    ctx.fillText(text, 256, 70);

    const texture = new THREE.CanvasTexture(canvas);
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false }));
    sprite.scale.set(0.16, 0.05, 1);
    return sprite;
  }

  function roundRect(ctx, x, y, width, height, radius) {
    ctx.beginPath();
    ctx.moveTo(x + radius, y);
    ctx.lineTo(x + width - radius, y);
    ctx.quadraticCurveTo(x + width, y, x + width, y + radius);
    ctx.lineTo(x + width, y + height - radius);
    ctx.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
    ctx.lineTo(x + radius, y + height);
    ctx.quadraticCurveTo(x, y + height, x, y + height - radius);
    ctx.lineTo(x, y + radius);
    ctx.quadraticCurveTo(x, y, x + radius, y);
    ctx.closePath();
  }

  function setGripperWidth(widthM, options) {
    const target = clamp(widthM, 0, GRIPPER_COMMAND_MAX);
    const source = options && options.source ? options.source : 'gripper';
    if (source === 'ros') {
      setJoint('gripper', target, false, { source: 'ros', emit: false });
      return;
    }
    if (hardwareControlActive() && !(options && options.localOnly)) {
      updateTargetJoint('gripper', target);
      commitTargetJoint('gripper');
      return;
    }
    const immediate = options && options.immediate;
    const emit = !(options && options.emit === false);

    gripperMotion = null;
    if (immediate) {
      setJoint('gripper', target, false, { source, emit });
      syncGhostToRobot();
      return;
    }

    gripperMotion = {
      start: currentAngles.gripper || 0,
      target,
      startedAt: performance.now(),
      duration: options && options.duration ? Math.max(Number(options.duration), 1) : GRIPPER_ANIMATION_MS,
      source,
      emit
    };
  }

  function updateGripperMotion(now) {
    if (!gripperMotion) return;

    const u = clamp((now - gripperMotion.startedAt) / gripperMotion.duration, 0, 1);
    const eased = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
    const value = gripperMotion.start + (gripperMotion.target - gripperMotion.start) * eased;
    setJoint('gripper', value, false, {
      source: gripperMotion.source,
      emit: gripperMotion.emit && u >= 1
    });
    syncGhostToRobot();

    if (u >= 1) gripperMotion = null;
  }

  window.reBotSim = {
    getAngles() {
      return { ...currentAngles };
    },
    getTcpPose() {
      return getTcpPose();
    },
    getJointDefs() {
      return jointDefs.map((joint) => ({ ...joint }));
    },
    getTeachingWaypoints() {
      return teachingWaypoints.map((point) => ({
        ...point,
        joints: { ...point.joints },
        tcp_ros: { ...point.tcp_ros }
      }));
    },
    setAngles(angles, options) {
      if (!angles || typeof angles !== 'object') return;
      const source = options && options.source ? options.source : 'api';
      if (source === 'ros' && (teachingPlayback || moveStart || animation || draggingTcp || dragSettling || gripperMotion)) return;
      if (source !== 'ros') {
        stopPath();
        teachingPlayback = null;
        moveStart = 0;
      }
      Object.entries(angles).forEach(([name, value]) => {
        setJoint(name, value, false, options || {});
      });
      if (source === 'ros') updateGhostTarget(targetAngles);
      else syncGhostToRobot();
    },
    setGripperWidth(widthM, options) {
      const source = options && options.source ? options.source : 'api';
      if (source === 'ros' && (teachingPlayback || moveStart || animation || draggingTcp || dragSettling || gripperMotion)) return;
      if (source !== 'ros') {
        stopPath();
        teachingPlayback = null;
        moveStart = 0;
      }
      if (options && options.animate) {
        setGripperWidth(widthM, options);
      } else {
        setJoint('gripper', clamp(widthM, 0, GRIPPER_COMMAND_MAX), false, options || {});
        if (source === 'ros') updateGhostTarget(targetAngles);
        else syncGhostToRobot();
      }
    },
    attachObject(color) {
      return attachObject(color);
    },
    attachObjectAtPosition(position, maxDistance) {
      return attachObjectAtPosition(position, maxDistance);
    },
    releaseObject(options) {
      return releaseObject(options || {});
    },
    getCarriedObject() {
      return carriedObject ? carriedObject.color : null;
    },
    syncMujocoObjectStates(objects) {
      applyMujocoObjectStates(objects);
    },
    getSceneCollisionMap() {
      return getSceneCollisionMap();
    },
    generateTrajectory,
    moveToTcp(position, label) {
      if (!position || ![position.x, position.y, position.z].every(Number.isFinite)) return;
      planTcpMoveTo(new THREE.Vector3(position.x, position.z, -position.y), label || 'tcp-target', { localOnly: true });
    },
    setDragMode(enabled) {
      if (Boolean(enabled) !== dragMode) toggleDragMode();
    },
    onCommand(listener) {
      if (typeof listener !== 'function') return () => {};
      commandListeners.add(listener);
     return () => commandListeners.delete(listener);
   }
 };

  if (window.rebotI18n) {
    window.rebotI18n.onLangChange(() => {
      Object.entries(presets).forEach(([key, preset], index) => {
        const btn = els.presets && els.presets.children[index];
        if (btn) btn.textContent = t(preset.label);
      });
      jointDefs.forEach((joint, index) => {
        const wrap = els.joints && els.joints.children[index];
        if (wrap) {
          const strong = wrap.querySelector('strong');
          if (strong) strong.textContent = t(joint.label);
        }
        updateJointLabel(joint.name);
      });
      updateTeachingStatus();
      if (els.toggleDrag) {
        els.toggleDrag.textContent = dragMode ? t('sim.exitDrag') : t('adv.drag');
        els.toggleDrag.classList.toggle('active', dragMode);
      }
      if (!draggingTcp && !dragSettling) {
        setDragStatus(dragMode ? t('sim.dragGreen') : t('app.dragDisabled'));
      }
      if (robot) updateTcpHud();
    });
  }

  function createOrbit(cam, dom, initialTarget) {
    let rotating = false;
    let panning = false;
    let lastX = 0;
    let lastY = 0;
    const target = initialTarget.clone();
    const spherical = new THREE.Spherical();
    const offset = new THREE.Vector3();

    function sync() {
      offset.copy(cam.position).sub(target);
      spherical.setFromVector3(offset);
    }
    sync();

    dom.addEventListener('pointerdown', (event) => {
      dom.setPointerCapture(event.pointerId);
      rotating = event.button === 0;
      panning = event.button === 2;
      lastX = event.clientX;
      lastY = event.clientY;
    });

    dom.addEventListener('pointermove', (event) => {
      const dx = event.clientX - lastX;
      const dy = event.clientY - lastY;
      lastX = event.clientX;
      lastY = event.clientY;

      if (rotating) {
        spherical.theta -= dx * 0.006;
        spherical.phi = clamp(spherical.phi - dy * 0.006, 0.12, Math.PI - 0.08);
      }

      if (panning) {
        const distance = cam.position.distanceTo(target);
        const right = new THREE.Vector3();
        const up = new THREE.Vector3(0, 1, 0);
        cam.getWorldDirection(right).cross(up).normalize();
        target.add(right.multiplyScalar(-dx * distance * 0.0015));
        target.y += dy * distance * 0.0015;
      }
    });

    dom.addEventListener('pointerup', (event) => {
      rotating = false;
      panning = false;
      if (dom.hasPointerCapture(event.pointerId)) dom.releasePointerCapture(event.pointerId);
    });

    dom.addEventListener('wheel', (event) => {
      event.preventDefault();
      spherical.radius = clamp(spherical.radius * (event.deltaY > 0 ? 1.08 : 0.92), 0.24, 4);
    }, { passive: false });

    dom.addEventListener('contextmenu', (event) => event.preventDefault());

    return {
      target,
      sync,
      update() {
        offset.setFromSpherical(spherical);
        cam.position.copy(target).add(offset);
        cam.lookAt(target);
      }
    };
  }
})();
