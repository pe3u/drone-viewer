// Bake the viewer's wind lines from the real airframe (reads artifact/raw/drone.glb, writes web/airflow.js).
// The mesh is voxelised, potential flow is solved around it on a half-model grid (mirror plane z = 0), and a
// horseshoe vortex is added for the wing's lift: upwash ahead, downwash behind and the two tip vortices.
// Streamlines are traced both ways from anchors just off the skin, so they hug the surfaces they show.
// Drone frame: nose towards -x, y up, span along z. Air moves towards +x at speed 1.
// usage: node scripts/bake_airflow.js
const fs = require('fs'), path = require('path');
const ROOT = path.join(__dirname, '..');

// ---------------------------------------------------------------- mesh
function readTriangles(file, skip) {
  const b = fs.readFileSync(file);
  const n = b.readUInt32LE(12), j = JSON.parse(b.slice(20, 20 + n)), bin = b.slice(20 + n + 8);
  const access = (i) => {
    const a = j.accessors[i], v = j.bufferViews[a.bufferView], off = (v.byteOffset || 0) + (a.byteOffset || 0);
    const comps = { SCALAR: 1, VEC3: 3 }[a.type], out = new Array(a.count * comps);
    const stride = v.byteStride || comps * { 5126: 4, 5125: 4, 5123: 2, 5121: 1 }[a.componentType];
    for (let k = 0; k < a.count; k++) for (let c = 0; c < comps; c++) {
      const at = off + k * stride;
      out[k * comps + c] = a.componentType === 5126 ? bin.readFloatLE(at + c * 4)
        : a.componentType === 5125 ? bin.readUInt32LE(at + c * 4)
        : a.componentType === 5123 ? bin.readUInt16LE(at + c * 2) : bin.readUInt8(at + c);
    }
    return out;
  };
  const mul = (a, c) => { const r = new Array(16).fill(0); for (let i = 0; i < 4; i++) for (let k = 0; k < 4; k++) for (let m = 0; m < 4; m++) r[k * 4 + i] += a[m * 4 + i] * c[k * 4 + m]; return r; };
  const trs = (o) => {
    if (o.matrix) return o.matrix;
    const [x, y, z, w] = o.rotation || [0, 0, 0, 1], [sx, sy, sz] = o.scale || [1, 1, 1], [tx, ty, tz] = o.translation || [0, 0, 0];
    return [(1 - 2 * (y * y + z * z)) * sx, 2 * (x * y + z * w) * sx, 2 * (x * z - y * w) * sx, 0,
      2 * (x * y - z * w) * sy, (1 - 2 * (x * x + z * z)) * sy, 2 * (y * z + x * w) * sy, 0,
      2 * (x * z + y * w) * sz, 2 * (y * z - x * w) * sz, (1 - 2 * (x * x + y * y)) * sz, 0, tx, ty, tz, 1];
  };
  const tris = [];
  const walk = (i, parent) => {
    const o = j.nodes[i];
    if (skip.test(o.name || '')) return;
    const m = mul(parent, trs(o));
    if (o.mesh != null) for (const pr of j.meshes[o.mesh].primitives) {
      const p = access(pr.attributes.POSITION), idx = access(pr.indices);
      const w = (k) => [m[0] * p[k * 3] + m[4] * p[k * 3 + 1] + m[8] * p[k * 3 + 2] + m[12],
        m[1] * p[k * 3] + m[5] * p[k * 3 + 1] + m[9] * p[k * 3 + 2] + m[13],
        m[2] * p[k * 3] + m[6] * p[k * 3 + 1] + m[10] * p[k * 3 + 2] + m[14]];
      for (let t = 0; t < idx.length; t += 3) tris.push([w(idx[t]), w(idx[t + 1]), w(idx[t + 2])]);
    }
    (o.children || []).forEach((c) => walk(c, m));
  };
  j.scenes[0].nodes.forEach((i) => walk(i, [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]));
  return tris;
}

// ---------------------------------------------------------------- grid
const H = 0.02, X0 = -1.9, Y0 = -0.95, Z0 = 0;
const NX = 240, NY = 105, NZ = 125, SY = NX, SZ = NX * NY, N = NX * NY * NZ;
const at = (i, j, k) => i + j * SY + k * SZ;
const solid = new Uint8Array(N);

const tris = readTriangles(path.join(ROOT, 'artifact/raw/drone.glb'), /^Propeller/);
for (const [a, b, c] of tris) {
  if (a[2] < -H && b[2] < -H && c[2] < -H) continue;   // the mirror half comes from the other wing
  const len = Math.max(Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]), Math.hypot(a[0] - c[0], a[1] - c[1], a[2] - c[2]), Math.hypot(b[0] - c[0], b[1] - c[1], b[2] - c[2]));
  const n = Math.ceil(len / (H * 0.5)) + 1;
  for (let u = 0; u <= n; u++) for (let v = 0; v <= n - u; v++) {
    const p = u / n, q = v / n, r = 1 - p - q;
    const i = Math.round((a[0] * p + b[0] * q + c[0] * r - X0) / H), j = Math.round((a[1] * p + b[1] * q + c[1] * r - Y0) / H);
    const k = Math.round(Math.abs(a[2] * p + b[2] * q + c[2] * r) / H);
    if (i >= 0 && i < NX && j >= 0 && j < NY && k >= 0 && k < NZ) solid[at(i, j, k)] = 1;
  }
}
{ // whatever the outside air cannot reach is solid too
  const seen = new Uint8Array(N), stack = [at(0, 0, NZ - 1)];
  seen[stack[0]] = 1;
  while (stack.length) {
    const c = stack.pop(), i = c % NX, j = ((c / SY) | 0) % NY, k = (c / SZ) | 0;
    const push = (d) => { const e = c + d; if (!seen[e] && !solid[e]) { seen[e] = 1; stack.push(e); } };
    if (i > 0) push(-1); if (i < NX - 1) push(1);
    if (j > 0) push(-SY); if (j < NY - 1) push(SY);
    if (k > 0) push(-SZ); if (k < NZ - 1) push(SZ);
  }
  for (let c = 0; c < N; c++) if (!seen[c]) solid[c] = 1;
}
let nSolid = 0; for (let c = 0; c < N; c++) nSolid += solid[c];
console.log(tris.length, 'triangles,', nSolid, 'solid cells of', N);

// ---------------------------------------------------------------- potential flow (SOR, phi = x on the outer box)
const phi = new Float32Array(N);
for (let k = 0; k < NZ; k++) for (let j = 0; j < NY; j++) for (let i = 0; i < NX; i++) phi[at(i, j, k)] = X0 + i * H;
const OMEGA = 1.96, SWEEPS = 900;
for (let s = 0; s < SWEEPS; s++) {
  let worst = 0;
  for (let k = 0; k < NZ - 1; k++) for (let j = 1; j < NY - 1; j++) {
    let c = at(1, j, k);
    for (let i = 1; i < NX - 1; i++, c++) {
      if (solid[c]) continue;
      const p = phi[c];
      const zm = k > 0 ? (solid[c - SZ] ? p : phi[c - SZ]) : phi[c + SZ];   // k = 0 is the mirror plane
      const sum = (solid[c - 1] ? p : phi[c - 1]) + (solid[c + 1] ? p : phi[c + 1])
        + (solid[c - SY] ? p : phi[c - SY]) + (solid[c + SY] ? p : phi[c + SY])
        + zm + (solid[c + SZ] ? p : phi[c + SZ]);
      const d = OMEGA * (sum / 6 - p);
      phi[c] = p + d;
      if (d > worst) worst = d; else if (-d > worst) worst = -d;
    }
  }
  if (s % 150 === 0) console.log('sweep', s, 'change', worst.toExponential(2));
}

// cell velocities; solid cells borrow from their neighbours so sampling stays smooth right at the skin
const vel = [new Float32Array(N), new Float32Array(N), new Float32Array(N)];
const known = new Uint8Array(N);
for (let k = 0; k < NZ; k++) for (let j = 0; j < NY; j++) for (let i = 0; i < NX; i++) {
  const c = at(i, j, k);
  if (solid[c]) continue;
  known[c] = 1;
  const grad = (d, lo, hi, axis) => {
    const a = lo && !solid[c - d] ? phi[c - d] : phi[c], b = hi && !solid[c + d] ? phi[c + d] : phi[c];
    const w = (lo && !solid[c - d] ? 1 : 0) + (hi && !solid[c + d] ? 1 : 0);
    return w ? (b - a) / (w * H) : (axis === 0 ? 1 : 0);
  };
  vel[0][c] = grad(1, i > 0, i < NX - 1, 0);
  vel[1][c] = grad(SY, j > 0, j < NY - 1, 1);
  vel[2][c] = k === 0 ? 0 : grad(SZ, true, k < NZ - 1, 2);
}
for (let pass = 0; pass < 4; pass++) {
  const add = [];
  for (let k = 1; k < NZ - 1; k++) for (let j = 1; j < NY - 1; j++) for (let i = 1; i < NX - 1; i++) {
    const c = at(i, j, k);
    if (known[c]) continue;
    let n = 0, a = 0, b = 0, d = 0;
    for (const e of [c - 1, c + 1, c - SY, c + SY, c - SZ, c + SZ]) if (known[e]) { n++; a += vel[0][e]; b += vel[1][e]; d += vel[2][e]; }
    if (n) add.push([c, a / n, b / n, d / n]);
  }
  for (const [c, a, b, d] of add) { vel[0][c] = a; vel[1][c] = b; vel[2][c] = d; known[c] = 1; }
}

// ---------------------------------------------------------------- sampling
function potential(x, y, z, out) {
  const az = Math.abs(z);
  const fx = (x - X0) / H, fy = (y - Y0) / H, fz = az / H;
  if (fx < 0 || fx >= NX - 1 || fy < 0 || fy >= NY - 1 || fz >= NZ - 1) { out[0] = 1; out[1] = 0; out[2] = 0; return; }
  const i = fx | 0, j = fy | 0, k = fz | 0, u = fx - i, v = fy - j, w = fz - k, c = at(i, j, k);
  for (let a = 0; a < 3; a++) {
    const f = vel[a];
    out[a] = (1 - w) * ((1 - v) * ((1 - u) * f[c] + u * f[c + 1]) + v * ((1 - u) * f[c + SY] + u * f[c + SY + 1]))
      + w * ((1 - v) * ((1 - u) * f[c + SZ] + u * f[c + SZ + 1]) + v * ((1 - u) * f[c + SZ + SY] + u * f[c + SZ + SY + 1]));
  }
  if (z < 0) out[2] = -out[2];
}
const isSolid = (x, y, z) => {
  const i = Math.round((x - X0) / H), j = Math.round((y - Y0) / H), k = Math.round(Math.abs(z) / H);
  return i >= 0 && i < NX && j >= 0 && j < NY && k < NZ && solid[at(i, j, k)] === 1;
};

// wing outline from the voxels
function stationOf(z) {
  const k = Math.round(z / H);
  let lo = 1e9, hi = -1e9;
  for (let j = 0; j < NY; j++) for (let i = 0; i < NX; i++) if (solid[at(i, j, k)] && X0 + i * H < 0.5) { lo = Math.min(lo, X0 + i * H); hi = Math.max(hi, X0 + i * H); }
  return hi < lo ? null : { lo, hi };
}
function column(x, z) {   // lowest and highest solid y at a point of the planform
  const i = Math.round((x - X0) / H), k = Math.round(z / H);
  let top = -1e9, bot = 1e9;
  for (let j = 0; j < NY; j++) if (solid[at(i, j, k)]) { top = Math.max(top, Y0 + j * H); bot = Math.min(bot, Y0 + j * H); }
  return top < bot ? null : { top, bot };
}
let tipZ = 0;
for (let k = 0; k < NZ; k++) if (stationOf(k * H)) tipZ = k * H;
const wingY = (z) => { const st = stationOf(z), c = column((st.lo + st.hi) / 2, z); return (c.top + c.bot) / 2; };
const mid = stationOf(0.8);
for (const z of [0.2, 0.5, 0.8, 1.1, tipZ - 0.04]) console.log('z', z.toFixed(2), 'wing y', wingY(z).toFixed(3), 'chord', stationOf(z).lo.toFixed(2), stationOf(z).hi.toFixed(2));

// ---------------------------------------------------------------- lift: horseshoe vortex
const GAMMA = 0.11, CORE = 0.035, XB = mid.lo + (mid.hi - mid.lo) * 0.3, YR = wingY(0.2), BZ = tipZ - 0.06, YB = wingY(BZ), FAR = 40;
function segment(p, a, b, g, out) {   // Biot-Savart of a straight filament with a smoothed core
  const r1 = [p[0] - a[0], p[1] - a[1], p[2] - a[2]], r2 = [p[0] - b[0], p[1] - b[1], p[2] - b[2]];
  const cx = r1[1] * r2[2] - r1[2] * r2[1], cy = r1[2] * r2[0] - r1[0] * r2[2], cz = r1[0] * r2[1] - r1[1] * r2[0];
  const l1 = Math.hypot(...r1), l2 = Math.hypot(...r2), L = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
  const ll = Math.hypot(...L), c2 = cx * cx + cy * cy + cz * cz, h2 = c2 / (ll * ll);
  const dot = L[0] * (r1[0] / l1 - r2[0] / l2) + L[1] * (r1[1] / l1 - r2[1] / l2) + L[2] * (r1[2] / l1 - r2[2] / l2);
  const k = g / (4 * Math.PI) * dot / (c2 + ll * ll * CORE * CORE);
  out[0] += k * cx; out[1] += k * cy; out[2] += k * cz;
  return h2;
}
const A = [XB + FAR, YB, -BZ], B = [XB, YB, -BZ], M = [XB, YR, 0], C = [XB, YB, BZ], D = [XB + FAR, YB, BZ];
let sign = 1;
function lift(p, out) { segment(p, A, B, sign * GAMMA, out); segment(p, B, M, sign * GAMMA, out); segment(p, M, C, sign * GAMMA, out); segment(p, C, D, sign * GAMMA, out); }
{ const v = [0, 0, 0]; lift([XB + 0.6, YR, 0], v); if (v[1] > 0) sign = -1; }   // downwash behind the wing

const pv = [0, 0, 0];
function field(p, out) {
  potential(p[0], p[1], p[2], pv);
  out[0] = pv[0]; out[1] = pv[1]; out[2] = pv[2];
  lift(p, out);
}

// ---------------------------------------------------------------- streamlines
const XA = -2.5, XE = 3.3, DS = 0.008, KEEP = 4;   // kept points are DS * KEEP apart
function trace(p0, dir) {
  const pts = [], k = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]], v = [0, 0, 0], q = [0, 0, 0];
  const unit = (p, o) => { field(p, o); const s = Math.hypot(o[0], o[1], o[2]) || 1e-6; o[0] *= dir / s; o[1] *= dir / s; o[2] *= dir / s; return s; };
  let p = [...p0], tau = 0, hits = 0;
  for (let s = 0; s < 3000; s++) {
    const speed = unit(p, k[0]);
    if (s % KEEP === 0) pts.push([p[0], p[1], p[2], tau, speed]);
    if (speed < 0.08 || p[0] < XA || p[0] > XE) break;
    for (let a = 0; a < 3; a++) q[a] = p[a] + k[0][a] * DS / 2; unit(q, k[1]);
    for (let a = 0; a < 3; a++) q[a] = p[a] + k[1][a] * DS / 2; unit(q, k[2]);
    for (let a = 0; a < 3; a++) q[a] = p[a] + k[2][a] * DS; unit(q, k[3]);
    for (let a = 0; a < 3; a++) v[a] = (k[0][a] + 2 * k[1][a] + 2 * k[2][a] + k[3][a]) / 6;
    p = [p[0] + v[0] * DS, p[1] + v[1] * DS, p[2] + v[2] * DS];
    tau += dir * DS / speed;
    if (isSolid(p[0], p[1], p[2])) hits++;
  }
  return { pts, hits, end: p };
}
function line(anchor) {
  const back = trace(anchor, -1), fwd = trace(anchor, 1);
  const pts = back.pts.slice(1).reverse().concat(fwd.pts);
  const t0 = pts[0][3];
  pts.forEach((q) => (q[3] -= t0));
  for (let pass = 0; pass < 14; pass++) {   // irons out the voxel staircase the lines pick up near the skin
    let prev = pts[0].slice();
    for (let i = 1; i < pts.length - 1; i++) {
      const cur = pts[i].slice();
      for (const a of [0, 1, 2, 4]) pts[i][a] = (prev[a] + 2 * cur[a] + pts[i + 1][a]) / 4;
      prev = cur;
    }
  }
  const inside = pts.filter((q) => isSolid(q[0], q[1], q[2])).length;
  return { pts, hits: back.hits + fwd.hits + inside * 4, reach: back.end[0] < XA + 0.2 && fwd.end[0] > XE - 0.2 };
}

const anchors = [];
const CLOSE = 0.04, MID = 0.11, FAR_OFF = 0.26;
for (const [z, off] of [[0.32, CLOSE], [0.5, MID], [0.62, CLOSE], [0.8, FAR_OFF], [0.92, CLOSE], [1.1, MID], [1.22, CLOSE], [1.38, CLOSE]]) {
  const st = stationOf(z); if (!st) continue;
  const x = st.lo + (st.hi - st.lo) * 0.4, col = column(x, z); if (!col) continue;
  anchors.push([x, col.top + off, z], [x, col.bot - off, z]);
}
const TIPX = stationOf(tipZ - 0.04).hi + 0.04;
for (const [dy, dz] of [[0.07, 0.0], [-0.07, 0.0], [0.0, 0.08], [0.0, -0.08], [0.13, 0.05], [-0.12, -0.06]]) anchors.push([TIPX, YB + dy, BZ + dz]);
for (const th of [0.5, 1.05, 1.6, 2.15, 2.7]) for (const off of [CLOSE, MID + 0.05]) {   // around the nose bay
  const x = -0.45, cy = 0, dy = Math.cos(th), dz = Math.sin(th);
  let r = 0; while (r < 0.5 && isSolid(x, cy + dy * r, dz * r) || r < 0.03) r += H / 2;
  let far = r; for (let t = r; t < 0.4; t += H / 2) if (isSolid(x, cy + dy * t, dz * t)) far = t;
  anchors.push([x, cy + dy * (far + off), dz * (far + off)]);
}
for (const [x, y, z] of [[0.72, 0.1, 0.16], [0.72, -0.03, 0.16], [0.72, 0.09, 0.32], [0.72, -0.02, 0.32], [0.74, 0.22, 0.06]]) anchors.push([x, y, z]);

const kept = [];
let dropped = 0;
for (const a of anchors) {
  if (isSolid(...a)) { dropped++; console.log('anchor inside', a.map((v) => +v.toFixed(2))); continue; }
  const l = line(a);
  if (!l.reach || l.hits > 6) { dropped++; console.log('dropped', a.map((v) => +v.toFixed(2)), 'hits', l.hits, 'reach', l.reach, l.pts.length); continue; }
  kept.push(l);
}
console.log(kept.length, 'lines kept,', dropped, 'dropped; points', kept.reduce((s, l) => s + l.pts.length, 0));

// ---------------------------------------------------------------- output: x y z tau speed per point, as int16
const POS = 8000, TAU = 3000, SPD = 10000;
const counts = kept.map((l) => l.pts.length), flat = new Int16Array(counts.reduce((a, b) => a + b, 0) * 5);
let o = 0, vMax = 0, vMin = 9, tMax = 0;
for (const l of kept) for (const [x, y, z, t, s] of l.pts) {
  flat[o++] = Math.round(x * POS); flat[o++] = Math.round(y * POS); flat[o++] = Math.round(z * POS);
  flat[o++] = Math.round(t * TAU); flat[o++] = Math.round(Math.min(s, 3) * SPD);
  vMax = Math.max(vMax, s); vMin = Math.min(vMin, s); tMax = Math.max(tMax, t);
}
console.log('speed', vMin.toFixed(2), vMax.toFixed(2), 'longest tau', tMax.toFixed(2));
const data = Buffer.from(flat.buffer).toString('base64');
fs.writeFileSync(path.join(ROOT, 'web/airflow.js'), `// generated by scripts/bake_airflow.js\nwindow.DRONE_AIRFLOW = ${JSON.stringify({ counts, pos: POS, tau: TAU, speed: SPD, data })};\n`);
console.log('web/airflow.js', (data.length / 1024).toFixed(0), 'KB');
