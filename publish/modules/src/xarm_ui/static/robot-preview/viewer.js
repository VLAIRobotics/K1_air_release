import * as THREE from './vendor/three/three.module.js';
import { OrbitControls } from './vendor/three/controls/OrbitControls.js';
import { STLLoader } from './vendor/three/loaders/STLLoader.js';
import { ColladaLoader } from './vendor/three/loaders/ColladaLoader.js';

const labels = { live: 'ROS反馈', partial: '反馈不完整', stale: '反馈过期', unknown: '未连接' };
const vector = (value, fallback) => value ? value.trim().split(/\s+/).map(Number) : fallback;
const direct = (node, tag) => [...node.children].filter(child => child.tagName === tag);

function origin(group, element) {
  if (!element) return;
  group.position.fromArray(vector(element.getAttribute('xyz'), [0, 0, 0]));
  const [r, p, y] = vector(element.getAttribute('rpy'), [0, 0, 0]);
  group.quaternion.setFromEuler(new THREE.Euler(r, p, y, 'ZYX'));
}

async function modelFromURDF(xml, scene, fetchAsset) {
  const doc = new DOMParser().parseFromString(xml, 'application/xml');
  if (doc.querySelector('parsererror')) throw new Error('模型XML无效');
  const robot = doc.documentElement;
  if (robot.tagName !== 'robot') throw new Error('模型格式无效');
  const links = new Map(), joints = new Map(), meshCache = new Map(), materialColors = new Map();
  for (const element of direct(robot, 'material')) {
    const rgba = element.querySelector('color')?.getAttribute('rgba');
    if (rgba) materialColors.set(element.getAttribute('name'), vector(rgba, []));
  }
  const loadMesh = url => {
    if (!url.startsWith('/api/robot-preview/meshes/')) throw new Error('模型资源地址无效');
    if (!meshCache.has(url)) {
      meshCache.set(url, (async () => {
        const response = await fetchAsset(url);
        if (!response.ok) throw new Error('模型网格加载失败');
        if (url.toLowerCase().endsWith('.stl')) {
          const geometry = new STLLoader().parse(await response.arrayBuffer());
          geometry.computeVertexNormals();
          return new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: 0xc2cbd6, roughness: 0.62 }));
        }
        if (url.toLowerCase().endsWith('.dae')) {
          const base = url.slice(0, url.lastIndexOf('/') + 1);
          const model = new ColladaLoader().parse(await response.text(), base).scene;
          // URDF specifies the mesh frame. ColladaLoader's optional Z_UP -> Y_UP
          // root conversion is for free-standing scenes, not URDF mesh frames.
          model.quaternion.identity();
          return model;
        }
        throw new Error('模型网格格式不支持');
      })());
    }
    return meshCache.get(url);
  };

  const jobs = [];
  for (const element of direct(robot, 'link')) {
    const link = new THREE.Group();
    link.name = element.getAttribute('name');
    links.set(link.name, link);
    for (const visual of direct(element, 'visual')) {
      const frame = new THREE.Group();
      origin(frame, direct(visual, 'origin')[0]);
      link.add(frame);
      const geometry = direct(visual, 'geometry')[0];
      const mesh = geometry?.querySelector('mesh');
      const rgba = vector(visual.querySelector('material > color')?.getAttribute('rgba'),
        materialColors.get(visual.querySelector('material')?.getAttribute('name')) || [0.72, 0.78, 0.85, 1]);
      const colorMaterial = new THREE.MeshStandardMaterial({
        color: new THREE.Color(...rgba.slice(0, 3)), roughness: 0.62,
        transparent: rgba[3] < 1, opacity: rgba[3], side: THREE.DoubleSide,
      });
      if (mesh) {
        frame.scale.fromArray(vector(mesh.getAttribute('scale'), [1, 1, 1]));
        jobs.push(loadMesh(mesh.getAttribute('filename')).then(source => {
          const object = source.clone(true);
          object.traverse(child => {
            if (child.isMesh) {
              if (visual.querySelector('material')) child.material = colorMaterial;
              child.castShadow = false;
            }
          });
          frame.add(object);
        }));
      } else if (geometry) {
        const box = geometry.querySelector('box'), cylinder = geometry.querySelector('cylinder'), sphere = geometry.querySelector('sphere');
        let shape;
        if (box) shape = new THREE.BoxGeometry(...vector(box.getAttribute('size'), [1, 1, 1]));
        if (sphere) shape = new THREE.SphereGeometry(Number(sphere.getAttribute('radius')), 20, 12);
        if (cylinder) {
          const radius = Number(cylinder.getAttribute('radius'));
          shape = new THREE.CylinderGeometry(radius, radius, Number(cylinder.getAttribute('length')), 20);
          shape.rotateX(Math.PI / 2);
        }
        if (shape) frame.add(new THREE.Mesh(shape, colorMaterial));
      }
    }
  }
  const children = new Set();
  for (const element of direct(robot, 'joint')) {
    const name = element.getAttribute('name'), type = element.getAttribute('type');
    const parent = links.get(element.querySelector('parent')?.getAttribute('link'));
    const childName = element.querySelector('child')?.getAttribute('link');
    const child = links.get(childName);
    if (!parent || !child) throw new Error('模型关节链不完整');
    const frame = new THREE.Group(), motion = new THREE.Group();
    origin(frame, direct(element, 'origin')[0]);
    frame.add(motion); motion.add(child); parent.add(frame);
    const axis = new THREE.Vector3().fromArray(vector(element.querySelector('axis')?.getAttribute('xyz'), [1, 0, 0])).normalize();
    const mimic = element.querySelector('mimic');
    joints.set(name, { motion, type, axis, value: 0,
      mimic: mimic ? { joint: mimic.getAttribute('joint'), multiplier: Number(mimic.getAttribute('multiplier') ?? 1), offset: Number(mimic.getAttribute('offset') ?? 0) } : null });
    children.add(childName);
  }
  const group = new THREE.Group(); group.name = robot.getAttribute('name') || 'robot';
  for (const [name, link] of links) if (!children.has(name)) group.add(link);
  scene.add(group);
  await Promise.all(jobs);
  group.updateMatrixWorld(true);
  let meshCount = 0;
  group.traverse(child => { if (child.isMesh) meshCount++; });
  if (!meshCount) throw new Error('模型没有可显示的网格');
  return {
    group, joints, meshCount,
    update(positions) {
      for (const [name, joint] of joints) {
        const value = positions[name];
        if (!joint.mimic && Number.isFinite(value)) joint.value = value;
      }
      const apply = joint => {
        if (joint.type === 'revolute' || joint.type === 'continuous') joint.motion.quaternion.setFromAxisAngle(joint.axis, joint.value);
        if (joint.type === 'prismatic') joint.motion.position.copy(joint.axis).multiplyScalar(joint.value);
      };
      for (const joint of joints.values()) if (!joint.mimic) apply(joint);
      for (const joint of joints.values()) if (joint.mimic) {
        const master = joints.get(joint.mimic.joint);
        if (master) { joint.value = master.value * joint.mimic.multiplier + joint.mimic.offset; apply(joint); }
      }
    },
  };
}

export function mountRobotPreview(container, options = {}) {
  if (!container) throw new Error('找不到模型面板');
  container.innerHTML = `
    <section class="x1-robot-viewer">
      <div class="x1-preview-heading"><strong data-model-title>机器人实时姿态</strong><button type="button" aria-label="复位模型视角">复位视角</button></div>
      <div class="x1-preview-viewport"><div class="x1-preview-message">正在加载模型…</div></div>
      <div class="x1-preview-states"><span data-side="left">左臂：未连接</span><span data-side="right">右臂：未连接</span></div>
      <div class="x1-preview-note">旋转：拖动 · 缩放：滚轮 · 仅显示收到的 ROS 反馈</div>
    </section>`;
  if (!document.getElementById('x1-robot-viewer-style')) {
    const style = document.createElement('style'); style.id = 'x1-robot-viewer-style';
    style.textContent = `
      .x1-robot-viewer{background:linear-gradient(145deg,#f9fbff,#edf2f8);border:1px solid #d9e2ed;border-radius:16px;overflow:hidden;color:#344660;width:100%;box-sizing:border-box;box-shadow:0 6px 20px #20324c09;font-family:inherit}
      .x1-preview-heading{display:flex;align-items:center;justify-content:space-between;padding:12px 14px 3px;font-size:14px;gap:8px}
      .x1-preview-heading button{border:1px solid #d5dfea;border-radius:7px;padding:4px 7px;background:white;color:#536780;font:inherit;font-size:11px;cursor:pointer}
      .x1-preview-viewport{height:255px;position:relative;touch-action:none}
      .x1-preview-viewport canvas{display:block;width:100%;height:100%;outline:none}
      .x1-preview-message{position:absolute;left:12px;right:12px;top:10px;font-size:12px;color:#65768c;pointer-events:none;z-index:1}
      .x1-preview-states{display:flex;gap:6px;padding:0 12px;flex-wrap:wrap}
      .x1-preview-states span{font-size:11px;padding:4px 7px;background:#e3e9f0;border-radius:6px;line-height:1.5}
      .x1-preview-states span[data-state=live]{background:#d9f2e6;color:#16714b}.x1-preview-states span[data-state=partial],.x1-preview-states span[data-state=stale]{background:#fff0d5;color:#996816}
      .x1-preview-note{font-size:10px;color:#7b8ba0;padding:7px 12px 10px;line-height:1.5}
      @media(max-width:600px){.x1-preview-viewport{height:230px}}
    `;
    document.head.append(style);
  }
  const viewport = container.querySelector('.x1-preview-viewport');
  const message = container.querySelector('.x1-preview-message');
  let renderer, controls, observer, robot, animation, pollTimer, destroyed = false;
  const pending = new Set();
  const api = { destroy() {
    destroyed = true; clearTimeout(pollTimer); cancelAnimationFrame(animation);
    pending.forEach(controller => controller.abort()); observer?.disconnect(); controls?.dispose();
    robot?.group.traverse(object => {
      object.geometry?.dispose();
      const materials = Array.isArray(object.material) ? object.material : [object.material];
      materials.filter(Boolean).forEach(material => { material.map?.dispose(); material.dispose(); });
    });
    renderer?.dispose(); renderer?.forceContextLoss(); container.replaceChildren();
  }, ready: null, model: null, lastState: null, receivedAt: null, connectionLost: false };
  const fetchAsset = async path => {
    const abort = new AbortController(); pending.add(abort);
    const timer = setTimeout(() => abort.abort(), 15000);
    try {
      const response = await fetch(path, { signal: abort.signal });
      // Consume the body before releasing the timeout and return a buffered Response.
      const body = await response.arrayBuffer();
      return new Response(body, { status: response.status, headers: response.headers });
    } finally { clearTimeout(timer); pending.delete(abort); }
  };
  const fetchData = async (path, json = true) => {
    const abort = new AbortController(); pending.add(abort);
    const timer = setTimeout(() => abort.abort(), 5000);
    try {
      const response = await fetch(path, { signal: abort.signal, cache: 'no-store' });
      if (!response.ok) throw new Error('预览请求失败');
      return json ? await response.json() : await response.text();
    } finally { clearTimeout(timer); pending.delete(abort); }
  };
  const markStates = (state, unavailable = false) => {
    if (state?.model) container.querySelector('[data-model-title]').textContent = `${state.model} 实时姿态`;
    const prefix = state?.joint_prefix || 'xarm_';
    for (const side of ['left', 'right']) {
      const badge = container.querySelector(`[data-side=${side}]`);
      const arm = state?.arms?.[side];
      const elapsed = api.receivedAt === null ? 0 : performance.now() - api.receivedAt;
      const fresh = state ? Array.from({ length: 7 }, (_, i) => state.joints?.[`${prefix}${side}_joint${i + 1}`])
        .filter(joint => joint && joint.age_ms + elapsed <= (state.stale_after_ms || 1000)).length : 0;
      const status = unavailable ? 'stale' : fresh === 7 ? 'live' : fresh ? 'partial' : arm?.received_joints ? 'stale' : 'unknown';
      badge.dataset.state = status;
      const handJoint = state?.joints?.[`${prefix}${side}_finger_joint1`];
      const hand = handJoint ? (handJoint.age_ms + elapsed <= (state.stale_after_ms || 1000) ? 'live' : 'stale') : 'unknown';
      const handLabel = hand === 'live' ? '' : hand === 'stale' ? ' · 夹爪过期' : ' · 夹爪无反馈';
      badge.textContent = `${side === 'left' ? '左臂' : '右臂'}：${unavailable ? '连接中断' : labels[status]}${handLabel}`;
      badge.title = arm?.sources?.join('\n') || '启用遥操作 ROS2 节点，或连接发布 /joint_states 的 ROS 控制系统';
    }
  };
  const poll = async () => {
    // Server ages describe its snapshot, not when this response reaches us.
    // Conservatively include the entire request/response time so a delayed
    // snapshot cannot become "live" again merely by arriving at the browser.
    const requestedAt = performance.now();
    try {
      const state = await fetchData(options.stateUrl || '/api/robot-preview/state');
      if (destroyed) return;
      if (!state.success) throw new Error('反馈不可用');
      api.lastState = state;
      api.receivedAt = requestedAt;
      api.connectionLost = false;
      robot?.update(state.positions || {});
      markStates(state);
    } catch (error) {
      if (!destroyed) { api.connectionLost = true; markStates(api.lastState, true); }
    } finally {
      if (!destroyed) pollTimer = setTimeout(poll, 100);
    }
  };
  api.ready = (async () => {
    const scene = new THREE.Scene(); scene.background = new THREE.Color(0xf0f4f9);
    const camera = new THREE.PerspectiveCamera(35, 1, 0.01, 50); camera.up.set(0, 0, 1);
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 0.9;
    viewport.append(renderer.domElement);
    controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = true; controls.enablePan = true;
    scene.add(new THREE.HemisphereLight(0xffffff, 0x93a0b1, 1.1));
    const light = new THREE.DirectionalLight(0xffffff, 1.4); light.position.set(2, -3, 4); scene.add(light);
    const grid = new THREE.GridHelper(2, 16, 0xc2cedd, 0xe0e7ef); grid.rotation.x = Math.PI / 2; scene.add(grid);
    const resize = () => {
      const width = viewport.clientWidth || 360, height = viewport.clientHeight || 255;
      renderer.setSize(width, height, false); camera.aspect = width / height; camera.updateProjectionMatrix();
    };
    observer = new ResizeObserver(resize); observer.observe(viewport); resize();
    const animate = () => {
      if (destroyed) return;
      animation = requestAnimationFrame(animate); controls.update(); renderer.render(scene, camera);
      if (api.lastState) markStates(api.lastState, api.connectionLost);
    };
    animate();
    const xml = await fetchData(options.modelUrl || '/api/robot-preview/model', false);
    if (destroyed) return api;
    robot = await modelFromURDF(xml, scene, fetchAsset); api.model = robot;
    if (destroyed) return api;
    const resetView = () => {
      robot.group.updateMatrixWorld(true);
      const bounds = new THREE.Box3().setFromObject(robot.group);
      const center = bounds.getCenter(new THREE.Vector3()), size = bounds.getSize(new THREE.Vector3());
      const direction = new THREE.Vector3(1.9, -2.5, 1.05).normalize();
      const right = new THREE.Vector3().crossVectors(direction.clone().negate(), camera.up).normalize();
      const up = new THREE.Vector3().crossVectors(right, direction.clone().negate()).normalize();
      const tanVertical = Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2);
      const tanHorizontal = tanVertical * camera.aspect;
      // Fit all projected box corners, including perspective depth, into 85%
      // of either viewport dimension instead of multiplying a diagonal radius.
      let distance = 0.1;
      for (const x of [bounds.min.x, bounds.max.x]) for (const y of [bounds.min.y, bounds.max.y]) for (const z of [bounds.min.z, bounds.max.z]) {
        const point = new THREE.Vector3(x, y, z).sub(center);
        const depth = point.dot(direction);
        distance = Math.max(distance, depth + Math.abs(point.dot(up)) / (tanVertical * 0.85),
          depth + Math.abs(point.dot(right)) / (tanHorizontal * 0.85));
      }
      const extent = Math.max(size.length(), 0.5);
      controls.target.copy(center); camera.position.copy(center).addScaledVector(direction, distance);
      controls.minDistance = extent * 0.15; controls.maxDistance = extent * 7;
      camera.near = 0.01; camera.far = Math.max(distance + extent * 10, 10); camera.updateProjectionMatrix(); controls.update();
    };
    resetView(); container.querySelector('button').addEventListener('click', resetView);
    message.textContent = '无反馈时保持最后姿态';
    message.dataset.loaded = 'true';
    poll();
    return api;
  })().catch(error => {
    if (!destroyed) message.textContent = error.message?.includes('WebGL') ? '浏览器未提供可用的 3D 显示环境' : '模型加载失败，请检查模型资源';
    throw error;
  });
  return api;
}

function autoMount() {
  const root = document.getElementById('robot-preview-root');
  if (root && !root.dataset.mounted) {
    root.dataset.mounted = 'true';
    const preview = mountRobotPreview(root);
    window.x1RobotPreview = preview;
    preview.ready.catch(error => console.warn('Robot model preview:', error.message));
    window.addEventListener('pagehide', () => preview.destroy(), { once: true });
  }
}
if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', autoMount, { once: true });
else autoMount();
