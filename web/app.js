// Survey drone explorer: orbit, guided tour, exploded view, cutaway / x-ray skins and capped section cuts.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { DRACOLoader } from 'three/addons/loaders/DRACOLoader.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

const MODEL = window.DRONE_MODEL_URL || './drone.glb';
const DRACO_PATH = 'https://cdn.jsdelivr.net/npm/three@0.170.0/examples/jsm/libs/draco/gltf/';
const $ = (s) => document.querySelector(s);
const clamp = THREE.MathUtils.clamp;
const smooth = (a, b, x) => { const t = clamp((x - a) / (b - a), 0, 1); return t * t * (3 - 2 * t); };
const easeIO = (t) => (t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2);
const approach = (cur, target, rate, dt) => cur + (target - cur) * (1 - Math.exp(-rate * dt));

// ------------------------------------------------------------------ renderer / scene
const stage = $('#stage');
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, stencil: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 0.95;
renderer.localClippingEnabled = true;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
stage.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.6;

// a hidden tab / pane can report a 0 x 0 window: keep the aspect finite or the camera turns NaN
const viewAspect = () => (innerWidth > 0 && innerHeight > 0 ? innerWidth / innerHeight : 1.6);
const camera = new THREE.PerspectiveCamera(32, viewAspect(), 0.02, 16000);
camera.position.set(-3.4, 1.9, 3.6);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 0.2;
controls.maxDistance = 10;
controls.autoRotateSpeed = 0.1;     // one turn in ten minutes
controls.target.set(0, 0, 0);

scene.add(new THREE.HemisphereLight(0xf2f6ff, 0x3b4656, 0.55));
const key = new THREE.DirectionalLight(0xffffff, 1.7);
key.position.set(2.5, 6, -3);
key.castShadow = true;
key.shadow.mapSize.set(2048, 2048);
Object.assign(key.shadow.camera, { left: -2.4, right: 2.4, top: 2.4, bottom: -2.4, near: 1, far: 16 });
key.shadow.bias = -0.0004;
key.shadow.normalBias = 0.02;
key.shadow.radius = 4;
scene.add(key);
const fill = new THREE.DirectionalLight(0xdfe8ff, 0.6);
fill.position.set(-4, 2, 4);
scene.add(fill);

// ground 100 m below the drone: one wide disc shaded like a soft, drifting cloud deck seen from above, under a fixed
// 20 m / 100 m grid. Noise octaves and grid lines both flatten out with distance instead of shimmering. It is almost white under the drone and eases out to nothing at the
// rim. The fade runs on the viewing angle: in metres it would pile up into a thin line at the horizon, in angle it
// reads evenly on screen
const GROUND_Y = -100, GROUND_R = 6000, GROUND_FADE = 150;
const ground = new THREE.Mesh(new THREE.PlaneGeometry(GROUND_R * 2, GROUND_R * 2), new THREE.ShaderMaterial({
  transparent: true, depthWrite: false,
  uniforms: { uTime: { value: 0 }, uColor: { value: new THREE.Color() }, uShade: { value: new THREE.Color() }, uLine: { value: new THREE.Color() }, uOpacity: { value: 1 }, uFade: { value: new THREE.Vector2(Math.atan(-GROUND_Y / GROUND_FADE), Math.atan(-GROUND_Y / GROUND_R)) }, uHeight: { value: -GROUND_Y } },
  vertexShader: `
    varying vec2 vP;
    void main() {
      vec4 w = modelMatrix * vec4(position, 1.0);
      vP = w.xz;
      gl_Position = projectionMatrix * viewMatrix * w;
    }`,
  fragmentShader: `
    uniform vec3 uColor, uShade, uLine;
    uniform float uTime, uOpacity, uHeight;
    uniform vec2 uFade;
    varying vec2 vP;
    float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
    float noise(vec2 p) {
      vec2 i = floor(p), f = fract(p);
      f = f * f * (3.0 - 2.0 * f);
      return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
    }
    float gridLine(float cell) {
      vec2 q = vP / cell, w = fwidth(q);
      vec2 g = abs(fract(q - 0.5) - 0.5) / max(w, vec2(1e-4));
      return (1.0 - min(min(g.x, g.y), 1.0)) * (1.0 - smoothstep(0.12, 0.45, max(w.x, w.y)));
    }
    float clouds(vec2 p, float px) {
      float v = 0.0, a = 0.5, sum = 0.0;
      for (int i = 0; i < 6; i++) {
        v += a * mix(noise(p), 0.5, smoothstep(0.2, 0.6, px));    // an octave finer than a pixel settles to its mean
        sum += a;
        p = mat2(1.6, 1.2, -1.2, 1.6) * p + 17.0;
        px *= 2.0;
        a *= 0.5;
      }
      return v / sum;
    }
    void main() {
      vec2 q = vP / 80.0;
      vec2 w = fwidth(q);
      float px = max(w.x, w.y);
      q += vec2(0.012, 0.005) * uTime;                                   // drifting at about 1 m/s
      // warped, so the billows do not line up; the warp moves a little slower, so they change shape as they drift
      q += 0.9 * vec2(noise(q * 0.7 + 3.0 - uTime * 0.004), noise(q * 0.7 + 9.0 + uTime * 0.003));
      float puff = smoothstep(0.28, 0.56, clouds(q, px));
      float angle = atan(uHeight, length(vP));
      float lines = max(gridLine(20.0) * 0.45, gridLine(100.0));
      gl_FragColor = vec4(mix(mix(uShade, uColor, puff), uLine, lines), smoothstep(uFade.y, uFade.x, angle) * mix(0.93, 1.0, puff) * uOpacity);
      #include <colorspace_fragment>
    }`,
}));
ground.rotation.x = -Math.PI / 2;
ground.position.y = GROUND_Y;
ground.renderOrder = -2;
scene.add(ground);

function radialTexture(size, stops) {
  const c = document.createElement('canvas');
  c.width = c.height = size;
  const g = c.getContext('2d'), r = size / 2, grad = g.createRadialGradient(r, r, 0, r, r, r);
  for (const [at, alpha] of stops) grad.addColorStop(at, `rgba(255,255,255,${alpha})`);
  g.fillStyle = grad;
  g.fillRect(0, 0, size, size);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// night sky: stars on the upper hemisphere, fading out towards the horizon. Seeded, so it is the same on every visit.
const SKY_R = 7000;
const sky = new THREE.Group();
{
  let seed = 20261104;
  const rnd = () => { seed = (seed + 0x6D2B79F5) | 0; let t = Math.imul(seed ^ (seed >>> 15), 1 | seed); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  const dot = radialTexture(32, [[0, 1], [0.4, 0.8], [1, 0]]);
  for (const [count, size, tint] of [[3200, 2, 0.8], [1000, 3, 1], [240, 4.6, 1.3]]) {
    const pos = new Float32Array(count * 3), col = new Float32Array(count * 3);
    for (let i = 0; i < count; i++) {
      const y = rnd(), a = rnd() * Math.PI * 2, r = Math.sqrt(1 - y * y);
      pos.set([r * Math.cos(a) * SKY_R, y * SKY_R, r * Math.sin(a) * SKY_R], i * 3);
      const warm = rnd(), b = tint * (0.6 + 0.4 * rnd()) * smooth(0, 0.1, y);      // a few warm stars among the blue-white
      col.set(warm < 0.15 ? [b, b * 0.86, b * 0.7] : [b * 0.86, b * 0.92, b], i * 3);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    const pts = new THREE.Points(geo, new THREE.PointsMaterial({
      size: size * Math.min(window.devicePixelRatio, 2), sizeAttenuation: false, map: dot, vertexColors: true,
      transparent: true, depthWrite: false, toneMapped: false, blending: THREE.AdditiveBlending,
    }));
    pts.renderOrder = -4;
    pts.frustumCulled = false;
    sky.add(pts);
  }
}
// aurora borealis: curtains around the sky with sharp lower edges and vertical rays that fade upward, green low
// and violet high. Noise runs on the unit circle of the azimuth, so there is no seam; it drifts very slowly.
// Ribbons in a flat projection of the sky carry it overhead, crossing the zenith at different angles.
const aurora = new THREE.Mesh(new THREE.SphereGeometry(SKY_R * 1.05, 48, 24, 0, Math.PI * 2, 0, Math.PI / 2), new THREE.ShaderMaterial({
  side: THREE.BackSide, transparent: true, depthWrite: false, toneMapped: false,
  blending: THREE.CustomBlending, blendEquation: THREE.AddEquation, blendSrc: THREE.OneFactor, blendDst: THREE.OneFactor,
  uniforms: { uTime: { value: 0 } },
  vertexShader: `
    varying vec3 vD;
    void main() { vD = position; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
  fragmentShader: `
    uniform float uTime;
    varying vec3 vD;
    float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
    float noise(vec2 p) {
      vec2 i = floor(p), f = fract(p);
      f = f * f * (3.0 - 2.0 * f);
      return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
    }
    vec3 curtain(vec2 c, float alt, float s) {
      float t = uTime;
      float base = 0.018 + 0.05 * noise(c * 1.4 + s + t * 0.012) + 0.014 * noise(c * 6.0 + s * 3.0 - t * 0.03);
      float h = alt - base;
      float reach = 0.16 + 0.3 * noise(c * 2.2 + s + 9.0);
      float body = smoothstep(-0.008, 0.012, h) * exp(-max(h, 0.0) / reach * 2.4);
      float rays = 0.4 + 0.9 * pow(noise(c * 42.0 + vec2(s, t * 0.05)), 2.0) + 0.35 * noise(c * 15.0 - vec2(t * 0.03, s));
      float region = smoothstep(0.3, 0.68, noise(c * 1.2 + s * 1.7 + t * 0.008));
      vec3 col = mix(vec3(0.10, 1.0, 0.48), vec3(0.08, 0.72, 0.78), smoothstep(0.02, 0.14, h));
      col = mix(col, vec3(0.58, 0.20, 0.92), smoothstep(0.1, 0.34, h));
      return col * body * rays * region;
    }
    vec3 ribbon(vec2 p, float turn, float off, float s) {
      float t = uTime;
      p = mat2(cos(turn), -sin(turn), sin(turn), cos(turn)) * p;
      float h = p.y - off + 0.4 * sin(p.x * 0.9 + s + t * 0.01) + 0.5 * (noise(p * 1.1 + s + t * 0.01) - 0.5);
      float body = smoothstep(-0.03, 0.02, h) * exp(-max(h, 0.0) / (0.28 + 0.3 * noise(vec2(p.x * 0.8, s))) * 2.4);
      float rays = 0.4 + 0.9 * pow(noise(vec2(p.x * 16.0 + t * 0.03, s)), 2.0) + 0.35 * noise(vec2(p.x * 6.0 - t * 0.02, s + 5.0));
      float region = smoothstep(0.25, 0.6, noise(p * 0.8 + s * 1.7 + t * 0.008));
      vec3 col = mix(vec3(0.10, 1.0, 0.48), vec3(0.08, 0.72, 0.78), smoothstep(0.03, 0.22, h));
      col = mix(col, vec3(0.58, 0.20, 0.92), smoothstep(0.16, 0.55, h));
      return col * body * rays * region;
    }
    void main() {
      vec3 d = normalize(vD);
      vec2 c = d.xz / max(length(d.xz), 1e-4);
      vec3 col = curtain(c, d.y, 3.1) + 0.8 * curtain(c, d.y, 11.7) + 0.6 * curtain(c, d.y, 23.3);
      col *= smoothstep(0.0, 0.03, d.y) * (1.0 - smoothstep(0.55, 0.9, d.y));
      vec2 p = d.xz / (d.y + 0.3);
      col += (ribbon(p, 0.5, -0.5, 4.3) + 0.8 * ribbon(p, 2.1, 0.1, 13.9) + 0.7 * ribbon(p, 3.9, 0.6, 29.1)) * smoothstep(0.12, 0.4, d.y);
      col *= 0.12;
      gl_FragColor = vec4(col, max(col.r, max(col.g, col.b)));
    }`,
}));
aurora.renderOrder = -3;
aurora.frustumCulled = false;
sky.add(aurora);
scene.add(sky);

// ------------------------------------------------------------------ state
const S = {
  explode: 0, explodeT: 0, ctrl: 0, ctrlT: 0,
  shell: 'cutaway', skinOp: 1, cut: 0,
  section: 'off', secPos: 0.5, secPosT: 0.5, flip: false, caps: true, sweep: null,
  labels: innerWidth > 700, rotate: true, spin: false, spinRate: 0, wire: false, ground: true,
  hovered: null, selected: null, isolate: null, focus: null,
  hidden: new Set(),
};
const clipPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
const parts = [];
const byName = {};
let bounds = new THREE.Box3();
let totalTris = 0;
let glbBytes = 0;
let interiorFill = 0;   // share of the fuselage interior occupied by parts (measured in Blender, glTF extras)

// ------------------------------------------------------------------ loading
// Two levels of detail exported from the same Blender build: phones and low-memory devices start on Lite.
const LOD_URLS = window.DRONE_LODS || { hd: MODEL, lite: MODEL.replace(/drone\.glb$/, 'drone_lite.glb') };
const weakDevice = matchMedia('(max-width: 700px)').matches || (navigator.deviceMemory && navigator.deviceMemory < 4);
let lod = new URLSearchParams(location.search).get('lod') || (weakDevice ? 'lite' : 'hd');
let modelRoot = null, firstLoad = true;
const draco = new DRACOLoader().setDecoderPath(DRACO_PATH);
const loader = new GLTFLoader().setDRACOLoader(draco);

function loadModel(level) {
  lod = level;
  $('#tHD').classList.toggle('on', lod === 'hd');
  const embed = window.DRONE_EMBED;   // self-contained build: base64 GLBs inlined in the page
  if (embed && embed[lod]) {
    const bin = atob(embed[lod]);
    const buf = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) buf[i] = bin.charCodeAt(i);
    glbBytes = buf.byteLength;
    loader.parse(buf.buffer, '', onLoad, (err) => { $('#loadTxt').textContent = 'Failed to read model: ' + err.message; });
    return;
  }
  loader.load(LOD_URLS[lod], onLoad, (e) => {
    if (!firstLoad) return;
    if (e.total) $('#loadBar').style.width = (100 * e.loaded / e.total).toFixed(0) + '%';
    $('#loadTxt').textContent = Math.round(e.loaded / 1024) + ' KB';
  }, (err) => { $('#loadTxt').textContent = 'Failed to load model: ' + err.message; });
  fetch(LOD_URLS[lod], { method: 'HEAD' }).then((r) => (glbBytes = +r.headers.get('content-length') || glbBytes)).catch(() => {});
}

function disposeModel() {
  if (!modelRoot) return;
  scene.remove(modelRoot);
  modelRoot.traverse((o) => { if (o.isMesh) { o.geometry.dispose(); [].concat(o.material).forEach((m) => m.dispose()); } });
  capsGroup.children.slice().forEach((c) => { c.material.dispose(); capsGroup.remove(c); });
  parts.length = 0;
  for (const k in byName) delete byName[k];
  labelEls.length = 0;
  $('#labels').innerHTML = '';
  $('#leaders').innerHTML = '';
  $('#partList').innerHTML = '';
  totalTris = 0;
}

function onLoad(gltf) {
  const keep = { sel: S.selected && S.selected.name, iso: S.isolate && S.isolate.name, focus: S.focus && S.focus.name };
  disposeModel();
  const root = (modelRoot = gltf.scene);
  scene.add(root);
  root.updateMatrixWorld(true);
  root.traverse((o) => { if (o.userData && o.userData.label) collectPart(o); });
  const rootNode = root.getObjectByName('SurveyDrone');
  interiorFill = (rootNode && rootNode.userData.fill) || 0;
  parts.sort((a, b) => a.order - b.order);
  parts.forEach((p, i) => { p.index = i; p.delay = (i / parts.length) * 0.45; });
  bounds = new THREE.Box3().setFromObject(root);
  buildCaps();
  buildPartList();
  buildLabels();
  layoutRing();
  for (const n of S.hidden) byName[n] && byName[n].row.classList.add('hidden');
  S.selected = byName[keep.sel] || null;
  S.isolate = byName[keep.iso] || null;
  S.focus = byName[keep.focus] || null;
  if (firstLoad) {
    firstLoad = false;
    setShell('solid', true);
    setSection('off');
    $('#loader').style.opacity = 0;
    setTimeout(() => $('#loader').remove(), 600);
    if (matchMedia('(prefers-reduced-motion: reduce)').matches) {
      // no fly-in, orbit or autoplay: show the tour's first view and wait for the viewer
      S.rotate = false;
      $('#tRotate').classList.remove('on');
      startTour(0);
      stopTour();
      const s = TOUR[0];
      controls.target.set(...s.tgt);
      camera.position.set(...s.cam);
    } else {
      S.explode = 1.15;            // parts fly in and assemble on first load
      camera.position.set(-5.7, 1.3, 6.1);
      startTour(0);
    }
  } else {
    setSection(S.section);
  }
}

function ownMeshes(o) {
  if (o.isMesh) return [o];
  return o.children.filter((c) => c.isMesh && !(c.userData && c.userData.label));
}

function collectPart(o) {
  const u = o.userData;
  const meshes = ownMeshes(o);
  let best = null, bestN = -1, tris = 0;
  const mats = [];
  for (const m of meshes) {
    m.material = m.material.clone();
    m.material.userData.base = { color: m.material.color.clone(), metalness: m.material.metalness, roughness: m.material.roughness };
    m.castShadow = true;
    m.receiveShadow = !!u.skin;   // interior stays lit like a cutaway illustration
    m.renderOrder = 1000;
    m.userData.part = null;
    const n = m.geometry.index ? m.geometry.index.count / 3 : m.geometry.attributes.position.count / 3;
    tris += n;
    if (n > bestN) { bestN = n; best = m; }
    mats.push(m.material);
  }
  const e = u.explode || [0, 0, 0];
  const p = {
    node: o, name: o.name, label: u.label, group: u.group, desc: u.desc || '', order: u.order || 0,
    skin: !!u.skin, closed: u.closed !== false, tris: u.tris || tris, meshes, mats,
    base: o.position.clone(), explode: new THREE.Vector3(e[0], e[2], -e[1]),
    color: best ? best.material.color.clone() : new THREE.Color(0x888888),
    opacity: 1, centerLocal: new THREE.Vector3(), size: 0.1,
  };
  for (const m of meshes) m.userData.part = p;
  if (/^(Aileron|Flap|Elevator|Rudder)/.test(p.name)) p.hinge = findHinge(p);
  const box = new THREE.Box3();
  meshes.forEach((m) => box.expandByObject(m));
  if (!box.isEmpty()) {
    const c = box.getCenter(new THREE.Vector3());
    p.centerLocal.copy(o.worldToLocal(c));
    p.size = box.getSize(new THREE.Vector3()).length();
  }
  totalTris += p.tris;
  parts.push(p);
  byName[p.name] = p;
}

function partCenter(p, out = new THREE.Vector3()) { return p.node.localToWorld(out.copy(p.centerLocal)); }

// ------------------------------------------------------------------ section caps (stencil)
const capsGroup = new THREE.Group();
scene.add(capsGroup);
const capGeom = new THREE.PlaneGeometry(1, 1);

function stencilMat(side, op) {
  return new THREE.MeshBasicMaterial({
    side, clippingPlanes: [clipPlane], colorWrite: false, depthWrite: false, depthTest: false,
    stencilWrite: true, stencilFunc: THREE.AlwaysStencilFunc, stencilFail: op, stencilZFail: op, stencilZPass: op,
  });
}

function buildCaps() {
  const back = stencilMat(THREE.BackSide, THREE.IncrementWrapStencilOp);
  const front = stencilMat(THREE.FrontSide, THREE.DecrementWrapStencilOp);
  parts.forEach((p, i) => {
    p.stencil = [];
    if (!p.closed) return;
    const ro = 1 + i * 2;
    for (const m of p.meshes) {
      for (const mat of [back, front]) {
        const s = new THREE.Mesh(m.geometry, mat);
        s.renderOrder = ro;
        s.raycast = () => {};
        s.visible = false;
        s.matrixAutoUpdate = false;
        m.add(s);
        p.stencil.push(s);
      }
    }
    const col = p.color.clone();
    const capMat = new THREE.MeshStandardMaterial({
      color: col, metalness: 0.05, roughness: 0.8, side: THREE.DoubleSide,
      emissive: col.clone().multiplyScalar(0.18),
      stencilWrite: true, stencilRef: 0, stencilFunc: THREE.NotEqualStencilFunc,
      stencilFail: THREE.ReplaceStencilOp, stencilZFail: THREE.ReplaceStencilOp, stencilZPass: THREE.ReplaceStencilOp,
    });
    const cap = new THREE.Mesh(capGeom, capMat);
    cap.renderOrder = ro + 0.5;
    cap.onAfterRender = (r) => r.clearStencil();
    cap.raycast = () => {};
    cap.visible = false;
    capsGroup.add(cap);
    p.cap = cap;
  });
}

// section plane outline
const secFrame = new THREE.LineSegments(
  new THREE.EdgesGeometry(new THREE.PlaneGeometry(1, 1)),
  new THREE.LineBasicMaterial({ color: 0xe0362b, transparent: true, opacity: 0.7, depthTest: true }));
secFrame.visible = false;
scene.add(secFrame);
const secFill = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({
  color: 0xe0362b, transparent: true, opacity: 0.05, side: THREE.DoubleSide, depthWrite: false }));
secFill.raycast = () => {};
secFrame.add(secFill);

const AXES = {
  z: { n: new THREE.Vector3(0, 0, -1), i: 'z', name: 'Side (buttock line)' },
  x: { n: new THREE.Vector3(1, 0, 0), i: 'x', name: 'Station' },
  y: { n: new THREE.Vector3(0, -1, 0), i: 'y', name: 'Plan (waterline)' },
};

function setSection(axis) {
  S.section = axis;
  const on = axis !== 'off';
  for (const p of parts) for (const m of p.mats) {
    m.clippingPlanes = on ? [clipPlane] : [];
    m.side = on && !p.closed ? THREE.DoubleSide : THREE.FrontSide;
    m.needsUpdate = true;
  }
  document.querySelectorAll('#secSeg button').forEach((b) => b.classList.toggle('on', b.dataset.v === axis));
  if (on && !S.sweep) {
    const start = { z: 0.0, x: -0.3, y: -0.02 }[axis];
    S.secPos = S.secPosT = sectionNorm(axis, start);
  }
}

function sectionNorm(axis, metres) {
  const i = AXES[axis].i;
  return (metres - bounds.min[i]) / (bounds.max[i] - bounds.min[i]);
}

function sectionValue(t) {
  const a = AXES[S.section];
  if (!a) return 0;
  const lo = bounds.min[a.i], hi = bounds.max[a.i];
  return lo + (hi - lo) * t;
}

function updateSection() {
  const on = S.section !== 'off';
  secFrame.visible = on;
  const showCaps = on && S.caps;
  if (!on) {
    for (const p of parts) { if (p.cap) p.cap.visible = false; p.stencil && p.stencil.forEach((s) => (s.visible = false)); }
    $('#secOut').textContent = '—';
    return;
  }
  const a = AXES[S.section];
  if (!Number.isFinite(S.secPos)) S.secPos = S.secPosT = 0.5;   // computed before bounds existed
  const v = sectionValue(S.secPos);
  if (!Number.isFinite(v)) return;        // bounds not ready (model swap in progress): keep the last plane
  const n = a.n.clone().multiplyScalar(S.flip ? -1 : 1);
  // keep n·p + c >= 0  → points beyond the plane along n stay
  clipPlane.normal.copy(n);
  clipPlane.constant = -(n.x * (a.i === 'x' ? v : 0) + n.y * (a.i === 'y' ? v : 0) + n.z * (a.i === 'z' ? v : 0));
  const c = bounds.getCenter(new THREE.Vector3());
  const sz = bounds.getSize(new THREE.Vector3());
  const pt = c.clone();
  pt[a.i] = v;
  secFrame.position.copy(pt);
  secFrame.lookAt(pt.clone().add(a.n));
  const dims = a.i === 'x' ? [sz.z, sz.y] : a.i === 'y' ? [sz.x, sz.z] : [sz.x, sz.y];
  secFrame.scale.set(dims[0] * 1.08 + 0.1, dims[1] * 1.08 + 0.1, 1);
  if (a.i === 'z') secFrame.scale.set(sz.x * 1.08 + 0.1, sz.y * 1.08 + 0.1, 1);
  for (const p of parts) {
    const vis = p.node.visible && partVisible(p) && p.opacity > 0.6;
    if (p.cap) {
      p.cap.visible = showCaps && vis;
      p.cap.position.copy(pt);
      p.cap.lookAt(pt.clone().add(a.n));
      p.cap.scale.set(8, 8, 1);
    }
    if (p.stencil) for (const s of p.stencil) s.visible = showCaps && vis;
  }
  $('#secOut').textContent = (v >= 0 ? '+' : '') + v.toFixed(3) + ' m';
}

// ------------------------------------------------------------------ skins, explode, highlight
function setShell(mode, instant) {
  S.shell = mode;
  document.querySelectorAll('#shellSeg button').forEach((b) => b.classList.toggle('on', b.dataset.v === mode));
  if (instant) { S.skinOp = skinTarget(); S.cut = mode === 'solid' ? 0 : 1; }
}
const skinTarget = () => ({ solid: 1, cutaway: 1, ghost: 0.13, hidden: 0 })[S.shell];
const isPanel = (p) => p.name === 'Access_Panel_L' || p.name === 'Access_Panel_R';
const partVisible = (p) => !S.hidden.has(p.name);
const HL = new THREE.Color();
const ACCENT = new THREE.Color(0x2f73ff);

// ------------------------------------------------------------------ control surfaces
const _pv = new THREE.Vector3(), _pv2 = new THREE.Vector3();
const D2R = Math.PI / 180;
function findHinge(p) {
  // hinge line = leading edge of the surface: the most-forward point at each span end
  const pts = [];
  for (const m of p.meshes) {
    const pos = m.geometry.attributes.position;
    const mat = m === p.node ? null : m.matrix;
    for (let i = 0; i < pos.count; i++) {
      const v = new THREE.Vector3().fromBufferAttribute(pos, i);
      if (mat) v.applyMatrix4(mat);
      pts.push(v);
    }
  }
  if (!pts.length) return null;
  const box = new THREE.Box3().setFromPoints(pts);
  const size = box.getSize(new THREE.Vector3());
  const L = size.y > size.z ? 'y' : 'z';
  const lo = box.min[L], hi = box.max[L], band = (hi - lo) * 0.06;
  const fwd = (sel) => pts.filter(sel).reduce((a, b) => (b.x < a.x ? b : a));
  const a = fwd((v) => v[L] < lo + band), b = fwd((v) => v[L] > hi - band);
  const axis = b.clone().sub(a).normalize();
  if (L === 'z' && axis.z > 0) axis.negate();     // +angle = trailing edge down
  if (L === 'y' && axis.y < 0) axis.negate();
  return { pivot: a, axis };
}
function deflection(p, t) {
  const n = p.name;
  if (n.startsWith('Aileron')) return (n.endsWith('_L') ? 1 : -1) * 20 * D2R * Math.sin(t * 2.0);
  if (n.startsWith('Flap')) return 24 * D2R * (0.5 - 0.5 * Math.cos(t * 0.9));
  if (n.startsWith('Elevator')) return 16 * D2R * Math.sin(t * 1.4 + 1);
  if (n === 'Rudder') return 22 * D2R * Math.sin(t * 1.1 + 2);
  return 0;
}

function updateParts(dt, time) {
  S.explode = approach(S.explode, S.explodeT, 3.2, dt);
  S.ctrl = approach(S.ctrl, S.ctrlT, 2.5, dt);
  S.skinOp = approach(S.skinOp, skinTarget(), 5, dt);
  S.cut = approach(S.cut, S.shell === 'solid' ? 0 : 1, 3.5, dt);
  const xe = S.explode;
  for (const p of parts) {
    const k = easeIO(clamp((xe * 1.45 - p.delay) / 1.0, 0, 1));
    p.node.position.copy(p.base).addScaledVector(p.explode, k);
    let op = 1;
    if (p.skin) op *= S.skinOp;
    if (isPanel(p)) {
      const c = easeIO(S.cut);
      p.node.position.addScaledVector(p.explode, c * 0.9);
      op *= 1 - smooth(0.15, 0.9, c);
    }
    if (p.hinge) {
      // rotate about the hinge line (pivot/axis in the node's parent-aligned frame)
      const ang = S.ctrl * deflection(p, time);
      p.node.quaternion.setFromAxisAngle(p.hinge.axis, ang);
      const piv = _pv.copy(p.node.position).add(p.hinge.pivot);
      p.node.position.copy(piv).sub(_pv2.copy(p.hinge.pivot).applyQuaternion(p.node.quaternion));
    }
    if (p.name === 'Wiring_Harness') op *= 1 - smooth(0.04, 0.3, xe);
    if (S.isolate && S.isolate !== p) op *= 0.07;
    p.opacity = op;
    const vis = partVisible(p) && op > 0.004;
    for (const m of p.meshes) m.visible = vis;
    const transparent = op < 0.995;
    const hot = p === S.selected || p === S.focus ? 0.55 + 0.25 * Math.sin(time * 4) : p === S.hovered ? 0.35 : 0;
    for (const mat of p.mats) {
      if (mat.transparent !== transparent) { mat.transparent = transparent; mat.needsUpdate = true; }
      mat.opacity = op;
      mat.depthWrite = op > 0.5;
      mat.wireframe = S.wire;
      HL.copy(ACCENT).multiplyScalar(hot * 0.22);
      mat.emissive.copy(HL);
    }
    for (const m of p.meshes) m.castShadow = op > 0.5;
  }
  const prop = byName.Propeller;
  if (prop) {
    S.spinRate = approach(S.spinRate, S.spin ? 38 : 0, 1.6, dt);
    prop.node.rotation.x += S.spinRate * dt;
  }
}

// ------------------------------------------------------------------ labels
const LABELS = [
  ['LiDAR_Sensor'], ['Mapping_Camera'], ['Flight_Controller'], ['Data_Processing_Unit', 'Data Processor'], ['Battery_Pack'],
  ['IMU', 'Avionics Bay'], ['GNSS_Antenna'], ['Comm_Antenna', 'Comms Antenna'], ['Electric_Motor'],
  ['Wing_L', 'Left Wing'], ['Wing_R', 'Right Wing'], ['Vertical_Fin', 'Tail Assembly'], ['Propeller'],
  ['Payload_Computer'], ['Satcom_Modem', 'Satcom / LTE'], ['Parachute_System', 'Parachute'],
  ['Belly_Battery', 'Belly Batteries'],
];
const labelEls = [];
function buildLabels() {
  const host = $('#labels');
  for (const [name, text] of LABELS) {
    const p = byName[name];
    if (!p) continue;
    const el = document.createElement('div');
    el.className = 'lbl';
    el.textContent = text || p.label;
    el.addEventListener('click', (ev) => { ev.stopPropagation(); select(p, true); });
    host.appendChild(el);
    const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    line.setAttribute('stroke-width', '1.3');
    const dot = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
    dot.setAttribute('r', '3');
    $('#leaders').append(line, dot);
    labelEls.push({ p, el, line, dot, w: 0, h: 0 });
  }
}

const _v = new THREE.Vector3();
let areaCache = null;
function freeArea() {
  // screen rectangle labels may use: below the title and, where it spans the window, the quick panel; above the
  // tour bar (the menus hide themselves)
  if (areaCache) return areaCache;
  const W = innerWidth, H = innerHeight;
  const tour = $('#tour'), tourB = getComputedStyle(tour).display !== 'none' ? tour.getBoundingClientRect() : null;
  const head = $('header.title').getBoundingClientRect();
  const quick = $('#quick').getBoundingClientRect();
  const top = quick.width > W / 2 ? quick.bottom : head.bottom;
  areaCache = { l: 8, r: W - 8, t: Math.max(top + 6, 8), b: (tourB && tourB.height ? tourB.top : H) - 8 };
  return areaCache;
}
addEventListener('resize', () => (areaCache = null));

// ------------------------------------------------------------------ roll-out menus
// Each menu rests as a small button and rolls out of it on hover, staying open while the pointer is on it.
// Touch has no hover: a tap on the button opens the menu, a tap outside closes it. Phones use #mobileToggle.
const MENUS = [...document.querySelectorAll('.autohide')];
const tabOf = (el) => document.querySelector(`.menuTab[data-menu="${el.id}"]`);
const REVEAL_PAD = 24, TAB_PAD = 4, HIDE_MS = 600;
const hideTimers = new Map();
let dragMenu = null, pinned = null, lastX = -1e4, lastY = -1e4;
function showMenu(el, on) {
  clearTimeout(hideTimers.get(el)); hideTimers.delete(el);
  el.classList.toggle('shown', on);
  tabOf(el).classList.toggle('open', on);
}
function within(node, x, y, pad) {
  const b = node.getBoundingClientRect();
  return b.width > 0 && x > b.left - pad && x < b.right + pad && y > b.top - pad && y < b.bottom + pad;
}
function keepMenu(el) {
  return el === dragMenu || el === pinned || within(el, lastX, lastY, REVEAL_PAD) || !!el.querySelector(':focus-visible') || tabOf(el).matches(':focus-visible');
}
function hideSoon(el) {
  if (!el.classList.contains('shown') || hideTimers.has(el)) return;
  hideTimers.set(el, setTimeout(() => { hideTimers.delete(el); if (keepMenu(el)) hideSoon(el); else showMenu(el, false); }, HIDE_MS));
}
function trackPointer(x, y) {
  lastX = x; lastY = y;
  const hit = MENUS.find((el) => within(tabOf(el), x, y, TAB_PAD));
  for (const el of MENUS) {
    const open = el.classList.contains('shown');
    if (hit) showMenu(el, el === hit);      // moving between the side-by-side buttons swaps menus at once
    else if (open && within(el, x, y, REVEAL_PAD)) showMenu(el, true);
    else if (open && el !== dragMenu) hideSoon(el);
  }
}
const menuOf = (node) => MENUS.find((el) => el.contains(node) || tabOf(el).contains(node));
addEventListener('pointermove', (e) => { if (e.pointerType !== 'touch') trackPointer(e.clientX, e.clientY); }, { passive: true });
addEventListener('pointerdown', (e) => {
  dragMenu = MENUS.find((el) => el.contains(e.target)) || null;   // keep a menu open while dragging its slider out of it
  if (e.pointerType === 'touch' && !menuOf(e.target)) { pinned = null; MENUS.forEach((el) => showMenu(el, false)); }
}, { passive: true });
addEventListener('pointerup', () => { dragMenu = null; trackPointer(lastX, lastY); }, { passive: true });
document.documentElement.addEventListener('pointerleave', () => { lastX = lastY = -1e4; MENUS.forEach(hideSoon); });
for (const el of MENUS) tabOf(el).addEventListener('click', (e) => {
  if (e.pointerType !== 'touch') { showMenu(el, true); return; }
  pinned = el;
  MENUS.forEach((m) => showMenu(m, m === el));
});
document.addEventListener('focusin', (e) => { const el = menuOf(e.target); if (el) showMenu(el, true); });
document.addEventListener('focusout', (e) => { const el = menuOf(e.target); if (el) hideSoon(el); });

// ------------------------------------------------------------------ label placement
// Labels sit on a fixed ring around the drone. Every label owns a slot, in the order its part lies around the
// airframe in the opening 3/4 view. The ring is centred on the orbit point and sized from the drone's span, so
// orbiting leaves the text exactly where it is: only the leader lines move.
const RING_GAP = 14, RING_STAGGER = 26;   // alternate slots step out near the top and bottom, where text runs along the circle
const RING_VIEW = Math.atan2(-5.2, 5.6);  // azimuth of the opening view

function layoutRing() {
  const c = bounds.getCenter(new THREE.Vector3());
  const list = labelEls.map((L) => { const q = partCenter(L.p); return { L, az: Math.atan2(q.z - c.z, q.x - c.x) }; }).sort((a, b) => a.az - b.az);
  const step = 2 * Math.PI / (list.length + list.length % 2);   // even slot count, so the stagger alternates all the way round
  let sx = 0, sy = 0;
  list.forEach(({ az }, k) => { sx += Math.cos(az - k * step); sy += Math.sin(az - k * step); });
  const a0 = Math.atan2(sy, sx);    // best fit of the even slots onto the parts' real directions
  list.forEach(({ L }, k) => { L.slot = a0 + k * step + RING_VIEW; L.level = k % 2; });
}
function labelRing(W, H, box) {
  const size = bounds.getSize(_v);
  const half = Math.max(size.x, size.y, size.z) / 2 * (1 + 0.4 * S.explode);
  const dist = camera.position.distanceTo(controls.target);
  const cx = W / 2, cy = H / 2;
  const fit = dist > half * 1.02 ? (H / 2) * half / (Math.sqrt(dist * dist - half * half) * Math.tan(camera.fov * Math.PI / 360)) : Infinity;
  // zoomed in, the drone outgrows the window: the ring stops at the largest one the window holds
  const rMax = Math.max(60, Math.min(Math.min(cy - box.t, box.b - cy) - 30 - RING_STAGGER, W / 2 - 90));
  return { cx, cy, R: Math.min(fit + RING_GAP, rMax) };
}

// text colour: each label and the title sample the rendered frame behind them and take bright or dark ink. The two
// thresholds keep text on a mid-tone from flickering. Must run right after render, while the drawing buffer is valid.
const INK_DARK_ABOVE = 0.54, INK_BRIGHT_BELOW = 0.46, INK_EVERY = 0.2;
const titleText = [$('header.title h1'), $('header.title p')];
let inkPixels = new Uint8Array(0), inkClock = INK_EVERY;
function backdropLuma(el) {
  const gl = renderer.getContext(), pr = renderer.getPixelRatio(), r = el.getBoundingClientRect();
  const bw = gl.drawingBufferWidth, bh = gl.drawingBufferHeight;
  const x = Math.max(0, Math.floor(r.left * pr)), y = Math.max(0, Math.floor(bh - r.bottom * pr));
  const w = Math.min(bw - x, Math.ceil(r.width * pr)), h = Math.min(bh - y, Math.ceil(r.height * pr));
  if (w <= 0 || h <= 0) return null;
  if (inkPixels.length < w * h * 4) inkPixels = new Uint8Array(w * h * 4);
  gl.readPixels(x, y, w, h, gl.RGBA, gl.UNSIGNED_BYTE, inkPixels);
  const page = isDark() ? 0.06 : 0.92;      // the canvas is transparent where nothing is drawn, showing the page
  let sum = 0;
  for (let i = 0; i < w * h * 4; i += 4) {
    sum += (0.2126 * inkPixels[i] + 0.7152 * inkPixels[i + 1] + 0.0722 * inkPixels[i + 2]) / 255 + (1 - inkPixels[i + 3] / 255) * page;
  }
  return sum / (w * h);
}
function setInk(el) {
  const luma = backdropLuma(el);
  if (luma === null) return;
  const dark = luma > INK_DARK_ABOVE || (luma >= INK_BRIGHT_BELOW && el.classList.contains('ink-dark'));
  el.classList.toggle('ink-dark', dark);
  el.classList.toggle('ink-bright', !dark);
}
function updateInk() {
  for (const el of titleText) setInk(el);
  for (const L of labelEls) {
    if (L.el.style.opacity !== '1') continue;
    setInk(L.el);
    const dark = L.el.classList.contains('ink-dark');     // the arrow changes together with its label
    for (const el of [L.line, L.dot]) {
      el.classList.toggle('ink-dark', dark);
      el.classList.toggle('ink-bright', !dark);
    }
  }
}

function updateLabels() {
  const W = innerWidth, H = innerHeight;
  const cs = getComputedStyle(document.documentElement);
  const lead = cs.getPropertyValue('--leader');
  const box = freeArea();
  const fc = S.focus ? partCenter(S.focus) : null;
  const ring = labelRing(W, H, box);
  for (const L of labelEls) {
    const p = L.p;
    partCenter(p, _v);
    let show = (S.labels || p === S.focus || p === S.selected) && partVisible(p) && (p.opacity > 0.08 || (p.skin && S.shell !== 'hidden'));
    if (fc && _v.distanceTo(fc) > 0.42 && p !== S.focus) show = false;
    if (S.section !== 'off' && clipPlane.distanceToPoint(_v) < -0.02) show = false;
    _v.project(camera);
    if (!show || _v.z > 1) { L.el.style.opacity = 0; L.el.style.pointerEvents = 'none'; L.line.style.opacity = 0; L.dot.style.opacity = 0; continue; }
    const ax = (_v.x * 0.5 + 0.5) * W, ay = (-_v.y * 0.5 + 0.5) * H;
    if (!L.w) { L.w = L.el.offsetWidth; L.h = L.el.offsetHeight; }
    // the circle touches the text's inner edge: text grows outward at every angle, with no left / right switch
    const c = Math.cos(L.slot), s = Math.sin(L.slot);
    const r = ring.R + L.level * RING_STAGGER * Math.abs(s);
    const x = clamp(ring.cx + c * r + (c - 1) * L.w / 2, box.l, box.r - L.w);
    const y = clamp(ring.cy + s * r + (s - 1) * L.h / 2, box.t, box.b - L.h);
    L.el.style.transform = `translate(${x.toFixed(1)}px, ${y.toFixed(1)}px)`;
    L.el.style.opacity = 1;
    L.el.style.pointerEvents = 'auto';
    L.el.classList.toggle('sel', L.p === S.selected || L.p === S.focus);
    const ex = clamp(ax, x - 3, x + L.w + 3), ey = clamp(ay, y + 3, y + L.h - 3);
    L.line.setAttribute('x1', ax); L.line.setAttribute('y1', ay);
    L.line.setAttribute('x2', ex); L.line.setAttribute('y2', ey);
    L.line.setAttribute('stroke', lead);
    L.line.style.opacity = 1;
    L.dot.setAttribute('cx', ax); L.dot.setAttribute('cy', ay);
    L.dot.setAttribute('fill', lead);
    L.dot.style.opacity = 1;
  }
}

// ------------------------------------------------------------------ parts list / info
function hex(c) { return '#' + c.clone().convertLinearToSRGB().getHexString(); }
function buildPartList() {
  const list = $('#partList');
  const groups = {};
  for (const p of parts) (groups[p.group] = groups[p.group] || []).push(p);
  const order = ['Airframe', 'Wings', 'Tail Assembly', 'Payload', 'Avionics', 'Power', 'Propulsion'];
  for (const g of [...order, ...Object.keys(groups).filter((g) => !order.includes(g))]) {
    if (!groups[g]) continue;
    const h = document.createElement('div');
    h.className = 'grp';
    h.textContent = g;
    list.appendChild(h);
    for (const p of groups[g]) {
      const row = document.createElement('div');
      row.className = 'part';
      row.innerHTML = `<span class="sw" style="background:${hex(p.color)}"></span><span class="nm"></span><button class="eye" title="Show / hide">👁</button>`;
      row.querySelector('.nm').textContent = p.label;
      row.addEventListener('click', () => select(p, true));
      row.querySelector('.eye').addEventListener('click', (e) => { e.stopPropagation(); toggleHidden(p); });
      p.row = row;
      list.appendChild(row);
    }
  }
  $('#partCount').textContent = `${parts.length} · ${(totalTris / 1000).toFixed(1)}k tris`;
}

function toggleHidden(p) {
  if (S.hidden.has(p.name)) S.hidden.delete(p.name); else S.hidden.add(p.name);
  p.row.classList.toggle('hidden', S.hidden.has(p.name));
  $('#infoHide').textContent = S.hidden.has(p.name) ? 'Show' : 'Hide';
}

function select(p, focus) {
  stopTour();
  S.selected = p;
  parts.forEach((q) => q.row && q.row.classList.toggle('sel', q === p));
  if (!p) { $('#info').classList.remove('show'); S.isolate = null; return; }
  $('#infoTitle').textContent = p.label;
  $('#infoMeta').textContent = `${p.group} · ${p.tris.toLocaleString()} triangles · node ${p.name}`;
  $('#infoDesc').textContent = p.desc;
  $('#infoHide').textContent = S.hidden.has(p.name) ? 'Show' : 'Hide';
  $('#infoIso').classList.toggle('on', S.isolate === p);
  $('#info').classList.add('show');
  if (p.row) p.row.scrollIntoView({ block: 'nearest' });
  if (focus) focusPart(p);
}

function focusPart(p, dur = 1.1) {
  const c = partCenter(p);
  const dir = camera.position.clone().sub(controls.target).normalize();
  const dist = clamp(p.size * 2.6, 0.45, 4);
  flyTo(c.clone().addScaledVector(dir, dist), c, dur);
}

// ------------------------------------------------------------------ camera flights
let flight = null;
// preset camera offsets were framed for a ~16:10 window: pull back on narrow / portrait screens
const fitScale = () => Math.max(1, Math.pow(1.6 / camera.aspect, 0.62));
function flyTo(pos, tgt, dur = 1.4) {
  const p1 = tgt.clone().add(pos.clone().sub(tgt).multiplyScalar(fitScale()));
  flight = { p0: camera.position.clone(), t0: controls.target.clone(), p1, t1: tgt.clone(), t: 0, dur };
}
function updateFlight(dt) {
  if (!flight) return;
  flight.t += dt / flight.dur;
  const k = easeIO(clamp(flight.t, 0, 1));
  // fly along an arc around the target to avoid passing through the model
  const a = flight.p0.clone().sub(flight.t0), b = flight.p1.clone().sub(flight.t1);
  const la = a.length(), lb = b.length();
  const dirv = a.normalize().lerp(b.normalize(), k).normalize();
  const tg = flight.t0.clone().lerp(flight.t1, k);
  controls.target.copy(tg);
  camera.position.copy(tg).addScaledVector(dirv, la + (lb - la) * k);
  if (flight.t >= 1) flight = null;
}
const VIEWS = {
  iso: [[-3.8, 0.85, 4.0], [0, 0, 0]], side: [[0.05, 0.12, 4.4], [0.05, 0.02, 0]], top: [[0.0, 5.2, 0.01], [0.05, 0, 0]],
  front: [[-4.6, 0.5, 0.0], [0, 0.05, 0]], under: [[-1.4, -3.4, 1.8], [0, 0, 0]],
};

// ------------------------------------------------------------------ guided tour
const TOUR = [
  { t: 'Aerial mapping & survey drone', x: 'Fixed-wing LiDAR and photogrammetry UAV, 3.1 m span, electric pusher. Drag anywhere to take control.',
    cam: [-3.8, 0.85, 4.0], tgt: [0, 0, 0], shell: 'solid', ex: 0, orbit: 0.8, d: 7 },
  { t: 'Cutaway', x: 'The side access panels come off to expose the avionics bay running from the nose to the wing.',
    cam: [-0.9, 0.55, 2.3], tgt: [-0.2, -0.04, 0], shell: 'cutaway', ex: 0, orbit: 0.3, d: 6 },
  { t: 'LiDAR sensor', x: 'Chin-mounted 360° multi-beam laser scanner. Its cooling rings sit inside the nose bay.',
    cam: [-1.45, -0.18, 0.95], tgt: [-0.72, -0.14, 0], shell: 'cutaway', ex: 0, focus: 'LiDAR_Sensor', orbit: 0.5, d: 6 },
  { t: 'Mapping camera', x: 'Full-frame camera on a 2-axis gimbal, hung from rubber dampers through a belly cut-out.',
    cam: [-1.05, -0.38, 0.85], tgt: [-0.52, -0.2, 0], shell: 'cutaway', ex: 0, focus: 'Mapping_Camera', orbit: 0.5, d: 6 },
  { t: 'Flight controller & IMU', x: 'Autopilot stack on vibration grommets, next to the gold tactical-grade INS that georeferences every laser return.',
    cam: [-0.62, 0.22, 0.72], tgt: [-0.44, -0.05, 0], shell: 'cutaway', ex: 0, focus: 'Flight_Controller', orbit: 0.4, d: 6 },
  { t: 'Data processing unit', x: 'Finned aluminium computer that time-syncs LiDAR, camera and INS data and logs it to SSD.',
    cam: [-0.3, 0.24, 0.68], tgt: [-0.17, -0.04, 0], shell: 'cutaway', ex: 0, focus: 'Data_Processing_Unit', orbit: 0.4, d: 6 },
  { t: 'Battery pack', x: 'Two hull-conforming 6S Li-ion packs fill the centre bay on the centre of gravity, under the wing carry-through.',
    cam: [0.02, 0.28, 0.72], tgt: [0.09, -0.04, 0], shell: 'cutaway', ex: 0, focus: 'Battery_Pack', orbit: 0.4, d: 6 },
  { t: 'Mission systems', x: 'Stacked above the avionics deck: edge-AI payload computer, mirrored SSD storage and a satcom / LTE modem for beyond-line-of-sight links.',
    cam: [-1.0, 0.48, 0.98], tgt: [-0.40, 0.02, 0], shell: 'ghost', ex: 0, focus: 'Payload_Computer', orbit: 0.4, d: 6 },
  { t: 'Range & recovery', x: 'Belly battery modules under the floor extend endurance; a ballistic parachute sits on the centre of gravity, between the wing roots.',
    cam: [0.38, 0.6, 1.02], tgt: [0.06, -0.01, 0], shell: 'ghost', ex: 0, focus: 'Parachute_System', orbit: 0.4, d: 6 },
  { t: 'GNSS & datalink', x: 'Survey-grade GNSS antenna and the datalink whip ride on the quick-release top hatch.',
    cam: [-0.95, 0.6, 0.95], tgt: [-0.3, 0.12, 0], shell: 'solid', ex: 0, focus: 'GNSS_Antenna', orbit: 0.5, d: 6 },
  { t: 'Electric pusher', x: 'Brushless outrunner and 2-blade propeller behind the cruciform tail; the ESC sits on the aft tray.',
    cam: [1.75, 0.4, 0.95], tgt: [0.9, 0.04, 0], shell: 'solid', ex: 0, focus: 'Electric_Motor', spin: true, orbit: 0.6, d: 7 },
  { t: 'Flight controls', x: 'Ailerons, flaps, elevators and rudder swing on their hinge lines, each driven by its own servo and pushrod.',
    cam: [-1.9, 1.25, 2.6], tgt: [0.15, 0.0, 0.2], shell: 'solid', ex: 0, ctrl: true, spin: true, orbit: 0.9, d: 8 },
  { t: 'Longitudinal section', x: 'A capped cut along the centreline: about 60 % of the fuselage volume is taken up by equipment, batteries and wiring.',
    cam: [0.05, 0.15, 3.3], tgt: [0.05, 0, 0], shell: 'solid', ex: 0, sec: { a: 'z', from: 0.16, to: 0.0 }, orbit: 0, d: 8 },
  { t: 'Station cuts', x: 'Sweeping a transverse plane from nose to tail through skins, bulkheads and payload.',
    cam: [-2.1, 0.75, 1.5], tgt: [0, 0, 0], shell: 'solid', ex: 0, sec: { a: 'x', from: -0.8, to: 0.6 }, orbit: 0.2, d: 9 },
  { t: 'Exploded view', x: 'Every sub-assembly pulled apart along its service direction.',
    cam: [-3.9, 2.6, 4.1], tgt: [0, 0.1, 0], shell: 'solid', ex: 1, orbit: 1.0, d: 9 },
  { t: 'Wing structure', x: 'Ribs, carbon spars, servos and pushrods drop out from under the wing skin.',
    cam: [-0.6, 0.35, 2.5], tgt: [0.05, -0.05, 1.1], shell: 'solid', ex: 1, focus: 'Wing_Structure_L', orbit: 0.3, d: 7 },
  { t: 'Re-assembled', x: 'Use the panels to cut, explode, isolate parts, or press play to watch the tour again.',
    cam: [-3.2, 1.5, 3.4], tgt: [0, 0, 0], shell: 'cutaway', ex: 0, orbit: 0.8, d: 7 },
];
const tour = { on: false, i: 0, t: 0 };

function buildSteps() {
  const host = $('#tourSteps');
  host.innerHTML = '';
  TOUR.forEach((s, i) => {
    const el = document.createElement('i');
    el.title = s.t;
    el.innerHTML = '<b></b>';
    el.addEventListener('click', () => startTour(i));
    host.appendChild(el);
  });
}
buildSteps();

function applyStep(i) {
  const s = TOUR[i];
  tour.i = i;
  tour.t = 0;
  $('#tourTitle').textContent = s.t;
  $('#tourText').textContent = s.x;
  setShell(s.shell);
  S.explodeT = s.ex;
  $('#explode').value = s.ex;
  S.focus = s.focus ? byName[s.focus] : null;
  S.spin = !!s.spin;
  $('#tSpin').classList.toggle('on', S.spin);
  S.ctrlT = s.ctrl ? 1 : 0;
  $('#tCtrl').classList.toggle('on', !!s.ctrl);
  S.isolate = null;
  if (s.sec) {
    S.sweep = s.sec;
    setSection(s.sec.a);
    S.secPos = S.secPosT = sectionNorm(s.sec.a, s.sec.from);
  } else if (S.section !== 'off') { S.sweep = null; setSection('off'); }
  flyTo(new THREE.Vector3(...s.cam), new THREE.Vector3(...s.tgt), 1.8);
  controls.autoRotate = false;
}

function startTour(i = 0) {
  tour.on = true;
  $('#tourPlay').textContent = '❚❚';
  S.selected = null;
  $('#info').classList.remove('show');
  applyStep(i);
}
function stopTour() {
  if (!tour.on) return;
  tour.on = false;
  S.sweep = null;
  S.focus = null;
  flight = null;
  $('#tourPlay').textContent = '▶';
  controls.autoRotate = S.rotate;
}
function updateTour(dt) {
  const bars = $('#tourSteps').children;
  for (let k = 0; k < bars.length; k++) bars[k].firstChild.style.width = (k < tour.i ? 100 : k > tour.i ? 0 : clamp(tour.t / TOUR[tour.i].d, 0, 1) * 100) + '%';
  if (!tour.on) return;
  const s = TOUR[tour.i];
  tour.t += dt;
  if (!flight && s.orbit) {
    const off = camera.position.clone().sub(controls.target);
    off.applyAxisAngle(new THREE.Vector3(0, 1, 0), THREE.MathUtils.degToRad(s.orbit) * dt);
    camera.position.copy(controls.target).add(off);
  }
  if (s.sec) {
    const k = smooth(0.15, 0.95, tour.t / s.d);
    S.secPosT = sectionNorm(s.sec.a, s.sec.from + (s.sec.to - s.sec.from) * k);
  }
  if (tour.t >= s.d) applyStep((tour.i + 1) % TOUR.length);
}

// ------------------------------------------------------------------ picking
const ray = new THREE.Raycaster();
const ptr = new THREE.Vector2();
let ptrDirty = false, downAt = null;
const tip = $('#tip');
function pickables() {
  const out = [];
  for (const p of parts) if (partVisible(p) && p.opacity > 0.3) out.push(...p.meshes);
  return out;
}
function pick(clientX, clientY) {
  ptr.set((clientX / innerWidth) * 2 - 1, -(clientY / innerHeight) * 2 + 1);
  ray.setFromCamera(ptr, camera);
  const hits = ray.intersectObjects(pickables(), false);
  for (const h of hits) {
    if (S.section !== 'off' && clipPlane.distanceToPoint(h.point) < 0) continue;
    return h.object.userData.part;
  }
  return null;
}
renderer.domElement.addEventListener('pointermove', (e) => {
  ptr.cx = e.clientX; ptr.cy = e.clientY; ptrDirty = true;
});
renderer.domElement.addEventListener('pointerleave', () => { S.hovered = null; tip.style.opacity = 0; });
renderer.domElement.addEventListener('pointerdown', (e) => { downAt = [e.clientX, e.clientY]; stopTour(); });
renderer.domElement.addEventListener('wheel', () => stopTour(), { passive: true });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 5) return;
  const p = pick(e.clientX, e.clientY);
  select(p, false);
});
renderer.domElement.addEventListener('dblclick', (e) => { const p = pick(e.clientX, e.clientY); if (p) select(p, true); });
function updateHover() {
  if (!ptrDirty || matchMedia('(hover: none)').matches) return;
  ptrDirty = false;
  const p = pick(ptr.cx, ptr.cy);
  S.hovered = p;
  renderer.domElement.style.cursor = p ? 'pointer' : 'grab';
  if (p) {
    tip.textContent = p.label;
    tip.style.left = ptr.cx + 14 + 'px';
    tip.style.top = ptr.cy + 12 + 'px';
    tip.style.opacity = 1;
  } else tip.style.opacity = 0;
}

// ------------------------------------------------------------------ UI wiring
function segHandler(sel, fn) {
  document.querySelectorAll(sel + ' button').forEach((b) => b.addEventListener('click', () => { stopTour(); fn(b.dataset.v); }));
}
segHandler('#shellSeg', (v) => setShell(v));
segHandler('#secSeg', (v) => { S.sweep = null; setSection(v); });
segHandler('#viewSeg', (v) => { const [p, t] = VIEWS[v]; flyTo(new THREE.Vector3(...p), new THREE.Vector3(...t), 1.2); });
$('#explode').addEventListener('input', (e) => { stopTour(); S.explodeT = +e.target.value; });
$('#secPos').addEventListener('input', (e) => {
  stopTour();
  if (S.section === 'off') setSection('z');
  S.sweep = null;
  S.secPosT = +e.target.value;
});
$('#secFlip').addEventListener('click', () => { S.flip = !S.flip; $('#secFlip').classList.toggle('on', S.flip); });
$('#secCaps').addEventListener('click', () => { S.caps = !S.caps; $('#secCaps').classList.toggle('on', S.caps); });
let manualSweep = null;
$('#secSweep').addEventListener('click', () => {
  stopTour();
  if (S.section === 'off') setSection('x');
  manualSweep = { t: 0 };
});
function toggle(id, key, after) {
  $(id).addEventListener('click', () => { S[key] = !S[key]; $(id).classList.toggle('on', S[key]); after && after(); });
}
toggle('#tRotate', 'rotate', () => { stopTour(); controls.autoRotate = S.rotate; });
toggle('#tLabels', 'labels');
$('#tLabels').classList.toggle('on', S.labels);
toggle('#tSpin', 'spin');
$('#tCtrl').addEventListener('click', () => { S.ctrlT = S.ctrlT ? 0 : 1; $('#tCtrl').classList.toggle('on', !!S.ctrlT); });
toggle('#tWire', 'wire');
toggle('#tGround', 'ground', () => { ground.visible = S.ground; });
$('#tTheme').addEventListener('click', () => {
  const dark = !isDark();
  document.documentElement.dataset.theme = dark ? 'dark' : 'light';
  applyTheme();
});
function isDark() {
  const t = document.documentElement.dataset.theme;
  return t ? t === 'dark' : matchMedia('(prefers-color-scheme: dark)').matches;
}
function applyTheme() {
  const dark = isDark();
  $('#tTheme').classList.toggle('on', dark);
  const gu = ground.material.uniforms;
  gu.uColor.value.set(dark ? 0xf3f6fb : 0xffffff);
  gu.uShade.value.set(dark ? 0xd5dde9 : 0xe4e9f1);
  gu.uLine.value.set(dark ? 0xa9b8ce : 0xa9b6c6);
  gu.uOpacity.value = dark ? 0.96 : 0.9;
  sky.visible = dark;         // no stars or aurora in a daylight sky
  secFrame.material.color.set(dark ? 0xff5a4e : 0xe0362b);
}
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', applyTheme);
applyTheme();
$('#tourPlay').addEventListener('click', () => (tour.on ? stopTour() : startTour((tour.i + 1) % TOUR.length)));
$('#tourNext').addEventListener('click', () => startTour((tour.i + 1) % TOUR.length));
$('#tourPrev').addEventListener('click', () => startTour((tour.i - 1 + TOUR.length) % TOUR.length));
$('#infoClose').addEventListener('click', () => select(null));
$('#infoFocus').addEventListener('click', () => S.selected && focusPart(S.selected));
$('#infoIso').addEventListener('click', () => {
  if (!S.selected) return;
  S.isolate = S.isolate === S.selected ? null : S.selected;
  $('#infoIso').classList.toggle('on', !!S.isolate);
});
$('#infoHide').addEventListener('click', () => S.selected && toggleHidden(S.selected));
$('#mParts').addEventListener('click', () => { $('#partsPanel').classList.toggle('open'); $('#tools').classList.remove('open'); areaCache = null; });
$('#mTools').addEventListener('click', () => { $('#tools').classList.toggle('open'); $('#partsPanel').classList.remove('open'); areaCache = null; });
addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  if (e.key === ' ') { e.preventDefault(); tour.on ? stopTour() : startTour(tour.i); }
  if (e.key === 'e') { stopTour(); S.explodeT = S.explodeT > 0.5 ? 0 : 1; $('#explode').value = S.explodeT; }
  if (e.key === 'Escape') select(null);
});
addEventListener('resize', () => {
  camera.aspect = viewAspect();
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
  labelEls.forEach((L) => (L.w = 0));
});

// ------------------------------------------------------------------ loop
const clock = new THREE.Clock();
let fpsAcc = 0, fpsN = 0, fps = 0, statT = 0, simTime = 0;
function frame() {
  tick(Math.min(clock.getDelta(), 0.05));
  requestAnimationFrame(frame);
}
// debug / testing hook: advance the simulation without requestAnimationFrame (hidden tabs)
window.__drone = {
  THREE, scene, camera, controls, renderer, S, tour, parts, byName, get flight() { return flight; },
  advance(sec, fps = 30) { for (let t = 0; t < sec; t += 1 / fps) tick(1 / fps); },
  startTour, stopTour, setSection, setShell, select,
};
function tick(dt) {
  simTime += dt;
  const time = simTime;
  aurora.material.uniforms.uTime.value = ground.material.uniforms.uTime.value = time;
  updateTour(dt);
  updateFlight(dt);
  if (manualSweep) {
    manualSweep.t += dt / 6;
    S.secPosT = 0.04 + 0.92 * smooth(0, 1, manualSweep.t);
    if (manualSweep.t >= 1) manualSweep = null;
  }
  S.secPos = approach(S.secPos, S.secPosT, 6, dt);
  $('#secPos').value = S.secPos;
  $('#explodeOut').textContent = Math.round(S.explode * 100) + '%';
  if (!tour.on) $('#explode').value = S.explodeT;
  if (parts.length) {
    updateParts(dt, time);
    updateSection();
  }
  controls.update();
  updateHover();
  renderer.render(scene, camera);
  if (parts.length) {
    updateLabels();
    inkClock += dt;
    if (inkClock >= INK_EVERY) { inkClock = 0; updateInk(); }
  }
  fpsAcc += dt; fpsN++;
  statT += dt;
  if (fpsAcc > 0.5) { fps = fpsN / fpsAcc; fpsAcc = 0; fpsN = 0; }
  if (statT > 0.5 && parts.length) {
    statT = 0;
    const i = renderer.info.render;
    $('#stats').innerHTML = `${(totalTris / 1000).toFixed(1)}k tris · ${parts.length} parts<br>${interiorFill ? `interior ${Math.round(interiorFill * 100)} % filled<br>` : ''}${i.calls} draw calls · ${fps.toFixed(0)} fps<br>${lod === "hd" ? "HD" : "Lite"} model · ${(glbBytes / 1024).toFixed(0)} KB ${window.DRONE_EMBED ? "embedded" : "Draco"}`;
  }
}
$('#tHD').addEventListener('click', () => loadModel(lod === 'hd' ? 'lite' : 'hd'));
loadModel(lod);
requestAnimationFrame(frame);
