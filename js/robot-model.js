// A detailed 3D robot vacuum built from simple shapes, about 35 cm across, in metres.
// Local frame: +X forward, +Y up, +Z to the robot's right.
// createRobotModel({ studio: true }) builds the high-detail version for "Robot up close":
// physical materials, a printed circuit board with glowing traces, wires, springs, a suction fan,
// a pleated filter, a lid on a hinge and a shell that can be cut away.
// The simulation uses the light version: same outside, fewer hidden parts.

import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { rng } from './util.js';

const ACCENT = 0xff4f00;
const Cyl = (rt, rb, h, seg = 48, open = false, ts = 0, tl = Math.PI * 2) =>
  new THREE.CylinderGeometry(rt, rb, h, seg, 1, open, ts, tl);
const Box = (x, y, z) => new THREE.BoxGeometry(x, y, z);
const RBox = (x, y, z, r) => new RoundedBoxGeometry(x, y, z, 3, Math.min(r, Math.min(x, y, z) / 2 - 1e-4));
// Turn a radius/height profile into a smooth round shell (like a potter's wheel).
const Lathe = (pts, seg = 96, ps = 0, pl = Math.PI * 2) =>
  new THREE.LatheGeometry(pts.map(([r, y]) => new THREE.Vector2(r, y)), seg, ps, pl);
const smoother = (t) => t * t * t * (t * (t * 6 - 15) + 10);
const clamp01 = (t) => (t < 0 ? 0 : t > 1 ? 1 : t);
const V = (a) => new THREE.Vector3(...a);

class Helix extends THREE.Curve {
  constructor(r, h, turns) { super(); this.r = r; this.h = h; this.turns = turns; }
  getPoint(u, out = new THREE.Vector3()) {
    const a = u * this.turns * Math.PI * 2;
    return out.set(Math.cos(a) * this.r, u * this.h, Math.sin(a) * this.r);
  }
}

// ---------------------------------------------------------------- printed textures
function canvasTexture(w, h, draw, srgb = true) {
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  draw(c.getContext('2d'), w, h);
  const t = new THREE.CanvasTexture(c);
  if (srgb) t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
}

// The circuit board. "glow" draws the same copper in white on black, used as an emissive map
// so the traces can light up. Board spans local x -0.055..0.095 and z -0.06..0.06.
function drawPcb(g, W, H, glow) {
  const R = rng(5);
  const P = (x, z) => [((x + 0.055) / 0.15) * W, ((z + 0.06) / 0.12) * H];
  g.fillStyle = glow ? '#000' : '#103426';
  g.fillRect(0, 0, W, H);
  g.strokeStyle = glow ? '#fff' : '#1b5039';
  g.lineCap = 'round'; g.lineJoin = 'round';
  for (let i = 0; i < 80; i++) {
    g.lineWidth = R() < 0.25 ? 6 : 3;
    let x = R() * W, y = R() * H;
    g.beginPath(); g.moveTo(x, y);
    for (let k = 0; k < 4; k++) {
      if (R() < 0.5) x += (R() - 0.5) * 240; else y += (R() - 0.5) * 240;
      if (R() < 0.35) { const d = (R() - 0.5) * 70; x += d; y += d; }
      g.lineTo(x, y);
    }
    g.stroke();
  }
  const gold = glow ? '#fff' : '#c9a54e';
  g.fillStyle = gold;
  const [mx, my] = P(0.045, 0.02), half = (0.016 / 0.15) * W, pw = 7, pl = 18;
  for (let k = 0; k < 10; k++) {
    const o = -half + (k + 0.5) * ((2 * half) / 10);
    g.fillRect(mx + o - pw / 2, my - half - pl, pw, pl);
    g.fillRect(mx + o - pw / 2, my + half, pw, pl);
    g.fillRect(mx - half - pl, my + o - pw / 2, pl, pw);
    g.fillRect(mx + half, my + o - pw / 2, pl, pw);
  }
  const [hx, hy] = P(0.08, -0.04);
  for (let k = 0; k < 6; k++) { g.beginPath(); g.arc(hx - 50 + k * 20, hy, 7, 0, Math.PI * 2); g.fill(); }
  for (let i = 0; i < 140; i++) {
    const x = R() * W, y = R() * H;
    g.fillStyle = gold; g.beginPath(); g.arc(x, y, 4, 0, Math.PI * 2); g.fill();
    g.fillStyle = glow ? '#000' : '#0a2219'; g.beginPath(); g.arc(x, y, 1.6, 0, Math.PI * 2); g.fill();
  }
  for (const [x, y] of [[26, 26], [W - 26, 26], [26, H - 26], [W - 26, H - 26]]) {
    g.fillStyle = gold; g.beginPath(); g.arc(x, y, 15, 0, Math.PI * 2); g.fill();
    g.fillStyle = '#060606'; g.beginPath(); g.arc(x, y, 8, 0, Math.PI * 2); g.fill();
  }
  if (glow) return;
  g.fillStyle = '#e8eee9';
  g.font = '800 20px Figtree, system-ui, sans-serif';
  const text = (s, x, z, dx = 0, dy = 0) => { const [px, py] = P(x, z); g.fillText(s, px + dx, py + dy); };
  text('U1', 0.045, 0.02, -14, -58);
  text('IMU', -0.025, 0.03, -18, -24);
  text('J3', 0.08, -0.04, -12, -20);
  text('RF', 0.0, -0.035, -10, -40);
  for (let k = 0; k < 4; k++) text('C' + (k + 1), -0.03 + k * 0.012, -0.045, -10, 34);
  const [lx, ly] = P(-0.05, 0.05);
  g.fillRect(lx, ly, 120, 3);
  g.font = '700 14px Figtree, system-ui, sans-serif';
  g.fillText('NAV CTRL REV 3', lx, ly - 8);
}

function batteryLabel() {
  return canvasTexture(512, 256, (g) => {
    g.fillStyle = '#f1efe9'; g.fillRect(0, 0, 512, 256);
    g.fillStyle = '#141414';
    g.font = '800 44px Figtree, system-ui, sans-serif';
    g.fillText('Li-ion pack', 28, 70);
    g.font = '700 30px Figtree, system-ui, sans-serif';
    g.fillText('14.4 V   5200 mAh   74.9 Wh', 28, 120);
    g.fillStyle = '#ff4f00'; g.fillRect(28, 146, 180, 10);
    g.fillStyle = '#141414';
    let x = 28;
    const R = rng(9);
    while (x < 300) { const w = 2 + Math.floor(R() * 5); g.fillRect(x, 180, w, 54); x += w + 2 + Math.floor(R() * 4); }
  });
}

// ---------------------------------------------------------------- the model
export function createRobotModel({ studio = false } = {}) {
  const root = new THREE.Group();
  root.name = 'robot';
  const lift = new THREE.Group();
  root.add(lift);
  // the lid and the lidar on top of it swing open around a hinge at the back edge
  const hinge = new THREE.Group();
  hinge.position.set(-0.166, 0.0795, 0);
  lift.add(hinge);
  const LID = [0.166, -0.0795, 0];

  const parts = {};
  const shellMats = [];
  const cutMats = new Set();
  const cutPlanes = [new THREE.Plane(new THREE.Vector3(-1, 0, 0), 0.3), new THREE.Plane(new THREE.Vector3(0, 0, 1), 0.3)];

  const std = (color, roughness = 0.6, metalness = 0, extra = {}) => new THREE.MeshStandardMaterial({ color, roughness, metalness, ...extra });
  const phys = (o) => new THREE.MeshPhysicalMaterial(o);
  const metal = (color = 0xbfc2c6, roughness = 0.3) => std(color, roughness, 1);

  // explode: where the part goes when taken apart. delay: later parts move later.
  // slide: extra offset when the robot is opened. cut: the shell the cutaway removes.
  const part = (name, explode, anchor, delay = 0, opt = {}) => {
    const g = new THREE.Group();
    g.name = 'part:' + name;
    (opt.parent || lift).add(g);
    const base = V(opt.base || [0, 0, 0]);
    g.position.copy(base);
    parts[name] = { name, group: g, explode: V(explode), anchor: V(anchor), mats: new Set(), h: 0, delay, base, slide: V(opt.slide || [0, 0, 0]), shell: !!opt.shell, cut: !!opt.cut };
    return g;
  };
  const add = (name, geo, mat, pos = [0, 0, 0], parent = null, highlight = true, rot = null) => {
    const m = new THREE.Mesh(geo, mat);
    m.position.set(...pos);
    if (rot) m.rotation.set(...rot);
    m.castShadow = true;
    m.receiveShadow = true;
    (parent || parts[name].group).add(m);
    for (const mm of [].concat(mat)) {
      if (highlight) parts[name].mats.add(mm);
      if (parts[name].shell && highlight) shellMats.push(mm);
      if (parts[name].cut) cutMats.add(mm);
    }
    return m;
  };
  const inst = (name, geo, mat, list, parent = null) => {
    const im = new THREE.InstancedMesh(geo, mat, list.length);
    const o = new THREE.Object3D();
    list.forEach((it, i) => {
      o.position.set(...it.p);
      o.rotation.set(...(it.r || [0, 0, 0]));
      o.scale.set(...(it.s || [1, 1, 1]));
      o.updateMatrix();
      im.setMatrixAt(i, o.matrix);
    });
    im.castShadow = true;
    im.receiveShadow = true;
    (parent || parts[name].group).add(im);
    parts[name].mats.add(mat);
    if (parts[name].cut) cutMats.add(mat);
    return im;
  };
  const shell = (m) => { m.transparent = true; return m; };

  // ---------- chassis: an open tub, so the inside shows when the lid is up ----------
  part('body', [0, 0, 0], [0.02, 0.04, 0.176], 0, { shell: true, cut: true });
  add('body', Lathe([[0.1675, 0.014], [0.169, 0.018], [0.17, 0.026], [0.17, 0.064], [0.1692, 0.0683], [0.1665, 0.0703], [0.157, 0.0705], [0.155, 0.0688]]),
    shell(phys({ color: 0xf1f1ee, roughness: 0.4, clearcoat: 0.3, clearcoatRoughness: 0.35, side: THREE.DoubleSide })));
  add('body', Lathe([[0, 0.004], [0.146, 0.004], [0.157, 0.0055], [0.1645, 0.0095], [0.1675, 0.0142]]),
    shell(std(0x2f2f2d, 0.85, 0, { side: THREE.DoubleSide })));
  const cliffMat = phys({ color: 0x050505, roughness: 0.08, clearcoat: 1, clearcoatRoughness: 0.05 });
  if (studio) {
    // inside the tub: a molded floor with ribs and screw bosses
    add('body', Cyl(0.163, 0.163, 0.002, 96), std(0xd3d1cb, 0.62), [0, 0.0145, 0]);
    const rib = std(0xc6c3bc, 0.6);
    for (let k = 0; k < 8; k++) {
      const a = (k / 8) * Math.PI * 2 + 0.2;
      add('body', Box(0.05, 0.006, 0.003), rib, [Math.cos(a) * 0.135, 0.018, Math.sin(a) * 0.135], null, true, [0, -a, 0]);
    }
    const screw = metal(0x9ea2a6, 0.35);
    for (let k = 0; k < 6; k++) {
      const a = (k / 6) * Math.PI * 2 + 0.3;
      add('body', Cyl(0.0042, 0.0042, 0.0022, 20), screw, [Math.cos(a) * 0.161, 0.0712, Math.sin(a) * 0.161]);
    }
    for (const a of [0.45, -0.45, 2.55, -2.55]) add('body', RBox(0.018, 0.004, 0.01, 0.002), cliffMat, [Math.cos(a) * 0.14, 0.0035, Math.sin(a) * 0.14], null, true, [0, -a, 0]);
    const brass = metal(0xd2b67a, 0.28);
    for (const z of [-0.026, 0.026]) add('body', RBox(0.024, 0.003, 0.012, 0.0014), brass, [-0.15, 0.004, z]);
  }

  // ---------- top cover (on the hinge) ----------
  part('cover', [0, 0.26, 0], [0.12, 0.085, -0.13], 0.06, { shell: true, cut: true, parent: hinge, base: LID });
  add('cover', Cyl(0.1662, 0.1662, 0.0025, 96), shell(std(0x8f8f89, 0.6)), [0, 0.0711, 0]);
  add('cover', Lathe([[0.1655, 0.0716], [0.1683, 0.0732], [0.169, 0.0756], [0.1678, 0.079], [0.1635, 0.0811], [0.155, 0.0816], [0, 0.0818]]),
    shell(phys({ color: 0xf8f8f6, roughness: 0.16, clearcoat: 1, clearcoatRoughness: 0.05, side: THREE.DoubleSide })));
  const btn = shell(phys({ color: 0xe6e6e2, roughness: 0.3, clearcoat: 0.7, clearcoatRoughness: 0.2 }));
  add('cover', Cyl(0.0085, 0.0085, 0.0025, 32), btn, [0.098, 0.0818, 0.022]);
  add('cover', Cyl(0.0085, 0.0085, 0.0025, 32), btn, [0.098, 0.0818, -0.022]);
  const ledMat = std(0x777777, 0.4, 0, { emissive: ACCENT, emissiveIntensity: 1.4 });
  add('cover', Cyl(0.0108, 0.0108, 0.0008, 32), ledMat, [0.098, 0.0821, 0.022], null, false);
  add('cover', Box(0.006, 0.0006, 0.0012), std(0x9a9a95, 0.5), [0.098, 0.0832, -0.022]);
  if (studio) {
    // the underside of the lid: a grey liner with a ring, seen when it opens
    add('cover', Cyl(0.15, 0.15, 0.0015, 96), std(0xcac8c2, 0.7), [0, 0.0718, 0]);
    add('cover', new THREE.TorusGeometry(0.13, 0.0016, 8, 120), std(0xb9b6af, 0.6), [0, 0.0712, 0], null, true, [Math.PI / 2, 0, 0]);
  }

  // ---------- lidar turret (on the lid; retracts when switched off) ----------
  part('lidar', [0, 0.36, 0], [-0.03, 0.112, 0.05], 0, { cut: true, parent: hinge, base: LID });
  const lidarIn = new THREE.Group();
  lidarIn.position.set(-0.03, 0.081, 0);
  parts.lidar.group.add(lidarIn);
  add('lidar', Cyl(0.044, 0.046, 0.008, 64), std(0x1e1e1e, 0.55), [0, 0.004, 0], lidarIn);
  add('lidar', Cyl(0.0425, 0.0425, 0.011, 64), phys({ color: 0x060606, roughness: 0.06, metalness: 0.2, clearcoat: 1, clearcoatRoughness: 0.03 }), [0, 0.0135, 0], lidarIn);
  add('lidar', Cyl(0.0441, 0.0441, 0.0012, 64), metal(0x9fa3a8, 0.3), [0, 0.0192, 0], lidarIn);
  add('lidar', Lathe([[0.0425, 0.019], [0.044, 0.0197], [0.0444, 0.0222], [0.0436, 0.0248], [0.0405, 0.0262], [0, 0.0264]], 64),
    phys({ color: 0xf6f6f3, roughness: 0.2, clearcoat: 1, clearcoatRoughness: 0.06, side: THREE.DoubleSide }), [0, 0, 0], lidarIn);
  add('lidar', Cyl(0.02, 0.02, 0.0008, 40), std(0xe2e2dd, 0.4), [0, 0.0267, 0], lidarIn);
  const spinner = new THREE.Group();
  spinner.position.y = 0.0135;
  lidarIn.add(spinner);
  const emitMat = std(0x330a00, 0.3, 0, { emissive: ACCENT, emissiveIntensity: 3 });
  add('lidar', Box(0.004, 0.0055, 0.012), emitMat, [0.0412, 0, -0.004], spinner, false);
  add('lidar', Box(0.003, 0.0055, 0.008), std(0x050505, 0.1, 0.5), [0.0418, 0, 0.009], spinner, false);
  if (studio) {
    // inside the turret: the spinning optics platform, visible in the cutaway
    add('lidar', Cyl(0.034, 0.034, 0.003, 48), std(0x2c2c2c, 0.5), [0, -0.004, 0], spinner, false);
    add('lidar', RBox(0.016, 0.008, 0.012, 0.002), metal(0x8f9398, 0.35), [0.02, 0.0005, 0], spinner, false);
    add('lidar', Cyl(0.005, 0.005, 0.008, 20), std(0x111111, 0.2, 0.4), [0.03, 0.0005, 0.008], spinner, false, [0, 0, Math.PI / 2]);
  }

  // ---------- bumper ----------
  part('bumper', [0.13, 0.03, 0], [0.2, 0.05, 0.1], 0.2, { shell: true, cut: true });
  add('bumper', Lathe([[0.1712, 0.0175], [0.1738, 0.0183], [0.1756, 0.0212], [0.176, 0.03], [0.176, 0.056], [0.1754, 0.0605], [0.1735, 0.0628], [0.1712, 0.0634]], 96, -0.2, Math.PI + 0.4),
    shell(phys({ color: 0x1b1b1a, roughness: 0.5, clearcoat: 0.35, clearcoatRoughness: 0.3, side: THREE.DoubleSide })));
  add('bumper', Lathe([[0.1758, 0.0368], [0.1773, 0.0374], [0.1775, 0.0416], [0.1758, 0.0422]], 96, -0.15, Math.PI + 0.3),
    std(0x0c0c0c, 0.95, 0, { side: THREE.DoubleSide }));
  add('bumper', RBox(0.004, 0.011, 0.05, 0.0015), phys({ color: 0x050505, roughness: 0.05, clearcoat: 1 }), [0.1758, 0.05, 0]);

  // ---------- drive wheels with encoders ----------
  const wheel = {};
  for (const side of [-1, 1]) {
    const name = side > 0 ? 'wheelR' : 'wheelL';
    part(name, [0, 0.02, side * 0.13], [0, 0.04, side * 0.17], 0.25);
    const hub = new THREE.Group();
    hub.position.set(0, 0.036, side * 0.1175);
    parts[name].group.add(hub);
    const spin = new THREE.Group();
    hub.add(spin);
    const tire = Cyl(0.036, 0.036, 0.022, 56); tire.rotateX(Math.PI / 2);
    add(name, tire, std(0x161616, 0.92), [0, 0, 0], spin);
    const treads = [];
    for (let k = 0; k < 24; k++) {
      const a = (k / 24) * Math.PI * 2;
      treads.push({ p: [Math.cos(a) * 0.0362, Math.sin(a) * 0.0362, 0], r: [0, 0, a + 0.25] });
    }
    inst(name, Box(0.0035, 0.0065, 0.021), std(0x101010, 0.95), treads, spin);
    const rim = Cyl(0.0245, 0.0245, 0.0236, 48); rim.rotateX(Math.PI / 2);
    add(name, rim, metal(0xc4c7cb, 0.32), [0, 0, 0], spin);
    const spokes = [];
    for (let k = 0; k < 5; k++) {
      const a = (k / 5) * Math.PI * 2;
      spokes.push({ p: [Math.cos(a) * 0.0105, Math.sin(a) * 0.0105, 0], r: [0, 0, a] });
    }
    inst(name, Box(0.021, 0.0052, 0.0244), metal(0x8e9297, 0.38), spokes, spin);
    const cap = Cyl(0.0075, 0.0075, 0.0248, 28); cap.rotateX(Math.PI / 2);
    add(name, cap, metal(0x55585c, 0.4), [0, 0, 0], spin);
    const disc = Cyl(0.028, 0.028, 0.002, 56); disc.rotateX(Math.PI / 2);
    add(name, disc, std(0xe4e4df, 0.5), [0, 0, -side * 0.016], spin);
    const slots = [];
    for (let k = 0; k < 16; k++) {
      const a = (k / 16) * Math.PI * 2;
      slots.push({ p: [Math.cos(a) * 0.022, Math.sin(a) * 0.022, -side * 0.016], r: [0, 0, a] });
    }
    inst(name, Box(0.006, 0.0035, 0.0026), std(0x242424, 0.6), slots, spin);
    add(name, RBox(0.012, 0.012, 0.012, 0.002), std(0x2a2a29, 0.5), [0, 0.027, -side * 0.016], hub);
    const encLed = std(0x220800, 0.4, 0, { emissive: ACCENT, emissiveIntensity: 0.2 });
    add(name, Box(0.003, 0.003, 0.003), encLed, [0.0068, 0.032, -side * 0.016], hub, false);
    add(name, RBox(0.075, 0.03, 0.022, 0.004), std(0xb9b9b4, 0.5), [-0.036, 0, -side * 0.034], hub);
    if (studio) {
      const can = Cyl(0.0115, 0.0115, 0.032, 40); can.rotateZ(Math.PI / 2);
      add(name, can, metal(0xaeb2b6, 0.25), [-0.088, 0.002, -side * 0.034], hub);
      const end = Cyl(0.0117, 0.0117, 0.004, 40); end.rotateZ(Math.PI / 2);
      add(name, end, metal(0xc08a52, 0.35), [-0.106, 0.002, -side * 0.034], hub);
      add(name, new THREE.TubeGeometry(new Helix(0.0055, 0.024, 6), 160, 0.0011, 8, false), metal(0xd0d3d6, 0.25), [-0.012, 0.015, -side * 0.034], hub);
    }
    wheel[name] = { hub, spin, encLed };
  }

  // ---------- front caster ----------
  part('caster', [0.06, -0.035, 0], [0.13, 0.0, 0.03], 0.3);
  const ball = add('caster', new THREE.SphereGeometry(0.011, 28, 20), std(0x1e1e1e, 0.55), [0.125, 0.011, 0]);
  add('caster', Cyl(0.017, 0.017, 0.01, 32), std(0xd9d9d4, 0.5), [0.125, 0.022, 0]);
  if (studio) add('caster', new THREE.TorusGeometry(0.0118, 0.0014, 10, 40), metal(0xb0b4b8, 0.3), [0.125, 0.013, 0], null, true, [Math.PI / 2, 0, 0]);

  // ---------- main brush ----------
  part('brush', [0, -0.06, 0], [-0.012, -0.01, 0.1], 0.3);
  const mb = new THREE.Group();
  mb.position.set(-0.012, 0.019, 0);
  parts.brush.group.add(mb);
  const roller = Cyl(0.011, 0.011, 0.17, 28); roller.rotateX(Math.PI / 2);
  add('brush', roller, std(0xd6d6d0, 0.6), [0, 0, 0], mb);
  const blades = [];
  for (let h = 0; h < 2; h++)
    for (let s = 0; s < 14; s++) {
      const t = s / 13, a = t * Math.PI * 1.2 + h * Math.PI;
      blades.push({ p: [Math.cos(a) * 0.0145, Math.sin(a) * 0.0145, -0.078 + 0.156 * t], r: [0, 0, a] });
    }
  inst('brush', Box(0.0045, 0.012, 0.012), std(0x4a4a47, 0.75), blades, mb);
  for (const z of [-0.086, 0.086]) {
    const e = Cyl(0.014, 0.014, 0.006, 28); e.rotateX(Math.PI / 2);
    add('brush', e, std(0x2a2a29, 0.5), [0, 0, z], mb);
  }
  if (studio) {
    const bristles = [];
    for (let h = 0; h < 2; h++)
      for (let s = 0; s < 40; s++) {
        const t = s / 39, a = t * Math.PI * 1.2 + h * Math.PI + Math.PI / 2;
        bristles.push({ p: [Math.cos(a) * 0.016, Math.sin(a) * 0.016, -0.08 + 0.16 * t], r: [0, 0, a - Math.PI / 2] });
      }
    inst('brush', Cyl(0.0009, 0.0006, 0.012, 5), std(0x353535, 0.8), bristles, mb);
  }

  // ---------- side brush ----------
  part('side', [0.06, -0.045, 0.07], [0.14, 0.0, 0.17], 0.3);
  const sb = new THREE.Group();
  sb.position.set(0.118, 0.007, 0.104);
  parts.side.group.add(sb);
  add('side', Cyl(0.011, 0.011, 0.007, 24), std(0x2a2a29, 0.5), [0, 0, 0], sb);
  const armMat = std(0x3a3a38, 0.6), tuftMat = std(0x9c978c, 0.85);
  for (let k = 0; k < 3; k++) {
    const pivot = new THREE.Group();
    pivot.rotation.y = (k / 3) * Math.PI * 2;
    sb.add(pivot);
    add('side', RBox(0.068, 0.0024, 0.005, 0.001), armMat, [0.034, -0.001, 0], pivot);
    const spread = studio ? [-0.12, -0.06, 0, 0.06, 0.12] : [0];
    for (const f of spread) {
      const tuft = new THREE.ConeGeometry(studio ? 0.004 : 0.008, 0.036, 8);
      tuft.rotateZ(-Math.PI / 2 - 0.06);
      tuft.rotateY(f);
      tuft.translate(0.082, -0.002, 0);
      add('side', tuft, tuftMat, [0, 0, 0], pivot);
    }
  }

  // ---------- inside: battery ----------
  part('battery', [0, 0.105, 0], [-0.096, 0.045, -0.05], 0.15);
  const wrap = phys({ color: 0x2c3440, roughness: 0.42, clearcoat: 0.5, clearcoatRoughness: 0.25 });
  add('battery', RBox(0.12, 0.026, 0.066, 0.006), wrap, [-0.035, 0.032, 0]);
  if (studio) {
    const lbl = new THREE.Mesh(new THREE.PlaneGeometry(0.07, 0.035).rotateX(-Math.PI / 2), std(0xffffff, 0.6, 0, { map: batteryLabel() }));
    lbl.position.set(-0.04, 0.0452, 0);
    lbl.receiveShadow = true;
    parts.battery.group.add(lbl);
    const term = metal(0xb4b8bc, 0.3);
    for (const z of [-0.012, 0.012]) add('battery', Box(0.004, 0.008, 0.008), term, [0.026, 0.034, z]);
    const wire = (z, color) => {
      const c = new THREE.CatmullRomCurve3([
        new THREE.Vector3(0.027, 0.034, z), new THREE.Vector3(0.042, 0.036, z * 1.2),
        new THREE.Vector3(0.055, 0.05, z - 0.012), new THREE.Vector3(0.066, 0.056, -0.02),
      ]);
      add('battery', new THREE.TubeGeometry(c, 40, 0.0016, 10, false), phys({ color, roughness: 0.35, clearcoat: 0.6 }));
    };
    wire(-0.008, 0x1a1a1a);
    wire(0.008, 0xc8281e);
    add('battery', RBox(0.01, 0.007, 0.012, 0.0015), std(0xf0eee8, 0.5), [0.07, 0.057, -0.022]);
  }

  // ---------- inside: computer with gyroscope ----------
  part('pcb', [0, 0.175, 0], [0.096, 0.07, 0.065], 0.1);
  const boardSide = std(0x0d2a1f, 0.6);
  const boardTop = studio
    ? phys({
      map: canvasTexture(640, 512, (g, w, h) => drawPcb(g, w, h, false)),
      emissiveMap: canvasTexture(640, 512, (g, w, h) => drawPcb(g, w, h, true), false),
      emissive: 0x000000, emissiveIntensity: 2.2, roughness: 0.42, clearcoat: 0.6, clearcoatRoughness: 0.3,
    })
    : std(0x1e2521, 0.6);
  add('pcb', Box(0.15, 0.0025, 0.12), [boardSide, boardSide, boardTop, boardSide, boardSide, boardSide], [0.02, 0.062, 0]);
  const chip = std(0x18191b, 0.55, 0.1);
  add('pcb', RBox(0.024, 0.0026, 0.024, 0.0006), chip, [0.045, 0.0645, 0.02]);
  const imuMat = std(0x18191b, 0.5, 0.1, { emissiveIntensity: 2.5 });
  add('pcb', RBox(0.008, 0.0022, 0.008, 0.0005), imuMat, [-0.025, 0.0643, 0.03]);
  add('pcb', Cyl(0.0008, 0.0008, 0.0003, 12), std(0xe8e8e2, 0.5), [-0.0275, 0.0655, 0.0275]);
  add('pcb', RBox(0.02, 0.003, 0.016, 0.0008), metal(0xcdd0d3, 0.22), [0.0, 0.0645, -0.035]);
  add('pcb', RBox(0.006, 0.0016, 0.003, 0.0006), metal(0xdadcdf, 0.2), [0.02, 0.0637, 0.035]);
  add('pcb', RBox(0.02, 0.006, 0.006, 0.0008), std(0xf2f0ea, 0.5), [0.08, 0.066, -0.04]);
  for (let k = 0; k < 4; k++) {
    const x = -0.03 + k * 0.012;
    add('pcb', Cyl(0.0035, 0.0035, 0.008, 24), std(k % 2 ? 0x1f2a44 : 0x23262c, 0.45), [x, 0.0675, -0.045]);
    add('pcb', Cyl(0.0033, 0.0033, 0.0004, 24), metal(0xc9ccd0, 0.25), [x, 0.0717, -0.045]);
  }
  if (studio) {
    const pins = [];
    for (let k = 0; k < 10; k++) {
      const o = -0.0108 + k * 0.0024;
      pins.push({ p: [0.045 + o, 0.0637, 0.02 - 0.0132] }, { p: [0.045 + o, 0.0637, 0.02 + 0.0132] });
      pins.push({ p: [0.045 - 0.0132, 0.0637, 0.02 + o], r: [0, Math.PI / 2, 0] }, { p: [0.045 + 0.0132, 0.0637, 0.02 + o], r: [0, Math.PI / 2, 0] });
    }
    inst('pcb', Box(0.0009, 0.0006, 0.0028), metal(0xd7d9dc, 0.25), pins);
    const hp = [];
    for (let k = 0; k < 6; k++) hp.push({ p: [0.072 + k * 0.0032, 0.0705, -0.04] });
    inst('pcb', Box(0.0008, 0.004, 0.0008), metal(0xd8b25a, 0.25), hp);
    const R = rng(21), smd = [], smd2 = [];
    for (let k = 0; k < 46; k++) {
      const p = [-0.05 + R() * 0.14, 0.0639, -0.055 + R() * 0.11];
      if (Math.abs(p[0] - 0.045) < 0.02 && Math.abs(p[2] - 0.02) < 0.02) continue;
      (k % 3 ? smd : smd2).push({ p, r: [0, R() < 0.5 ? 0 : Math.PI / 2, 0] });
    }
    inst('pcb', Box(0.0022, 0.0009, 0.0012), std(0x141414, 0.5), smd);
    inst('pcb', Box(0.0022, 0.0009, 0.0012), std(0xb89b6a, 0.5), smd2);
  }

  // ---------- inside: suction fan (high-detail model only) ----------
  let rotor = null;
  if (studio) {
    part('fan', [-0.06, 0.105, -0.075], [-0.105, 0.06, -0.105], 0.15);
    const fx = -0.075, fz = -0.07;
    add('fan', Cyl(0.026, 0.026, 0.03, 56, true), std(0xd9d9d4, 0.5, 0, { side: THREE.DoubleSide }), [fx, 0.035, fz]);
    add('fan', Cyl(0.026, 0.026, 0.003, 56), std(0xcfcfca, 0.5), [fx, 0.0205, fz]);
    rotor = new THREE.Group();
    rotor.position.set(fx, 0.032, fz);
    parts.fan.group.add(rotor);
    add('fan', Cyl(0.008, 0.008, 0.02, 24), metal(0x6d7176, 0.3), [0, 0, 0], rotor);
    const vanes = [];
    for (let k = 0; k < 11; k++) {
      const a = (k / 11) * Math.PI * 2;
      vanes.push({ p: [Math.cos(a) * 0.0155, 0, Math.sin(a) * 0.0155], r: [0, -a + 0.45, 0] });
    }
    inst('fan', Box(0.0145, 0.02, 0.0014), std(0x3a3d42, 0.35, 0.6), vanes, rotor);
    add('fan', RBox(0.03, 0.018, 0.022, 0.004), std(0xd2d2cd, 0.5), [fx - 0.028, 0.033, fz + 0.018]);
    add('fan', Cyl(0.013, 0.013, 0.016, 32), metal(0xa9adb1, 0.28), [fx, 0.012, fz]);
  }

  // ---------- inside: dust bin (slides out backwards when the robot is opened) ----------
  part('bin', [-0.17, 0.06, 0], [-0.175, 0.07, 0.07], 0.2, { slide: [-0.08, 0.004, 0] });
  if (studio) {
    add('bin', RBox(0.055, 0.036, 0.15, 0.007),
      phys({ color: 0x8e979f, roughness: 0.1, transparent: true, opacity: 0.52, clearcoat: 1, clearcoatRoughness: 0.06, depthWrite: false }),
      [-0.122, 0.046, 0]);
    const fins = [];
    for (let k = 0; k < 17; k++) fins.push({ p: [-0.099, 0.046, -0.056 + k * 0.007] });
    inst('bin', Box(0.006, 0.028, 0.0016), std(0xf4f2ec, 0.9), fins);
    add('bin', RBox(0.004, 0.032, 0.124, 0.001), std(0xe9e6de, 0.8), [-0.103, 0.046, 0]);
    const R = rng(33), crumbs = [];
    for (let k = 0; k < 40; k++) crumbs.push({ p: [-0.14 + R() * 0.03, 0.031 + R() * 0.004, -0.06 + R() * 0.12], r: [R() * 3, R() * 3, R() * 3], s: [1, 0.5 + R() * 0.6, 1] });
    inst('bin', new THREE.IcosahedronGeometry(0.0028, 0), std(0x8b8277, 0.95, 0, { flatShading: true }), crumbs);
    add('bin', RBox(0.012, 0.006, 0.05, 0.003), std(0x2a2a29, 0.5), [-0.15, 0.066, 0]);
  } else {
    add('bin', Box(0.055, 0.034, 0.15), std(0x55585b, 0.2, 0, { transparent: true, opacity: 0.72 }), [-0.122, 0.046, 0]);
  }

  // ---------- cutaway: clip the shell, darken its inner faces so the cut reads ----------
  if (studio) {
    for (const m of cutMats) {
      m.side = THREE.DoubleSide;
      m.clippingPlanes = cutPlanes;
      m.clipIntersection = true;
      m.clipShadows = true;
      m.onBeforeCompile = (sh) => {
        sh.fragmentShader = sh.fragmentShader.replace('#include <dithering_fragment>',
          'gl_FragColor.rgb *= gl_FrontFacing ? 1.0 : 0.6;\n#include <dithering_fragment>');
      };
      m.needsUpdate = true;
    }
  }
  wrap.userData.glowable = true;
  imuMat.userData.glowable = true;
  boardTop.userData.glowable = true;
  cliffMat.userData.glowable = true;

  // ---------- animation ----------
  const st = { lidarUp: 1, lift: 0, xray: 0, E: 0, explode: 0, bump: 0, open: 0, openV: 0, cut: 0 };
  const highlight = new THREE.Color(ACCENT);
  const black = new THREE.Color(0x000000);

  function update(s, dt) {
    const k = 1 - Math.exp(-dt * 9);
    st.lidarUp += ((s.lidarOn ? 1 : 0) - st.lidarUp) * (1 - Math.exp(-dt * 6));
    st.lift += ((s.lifted ? 1 : 0) - st.lift) * k;
    st.xray += ((s.xray || 0) - st.xray) * (1 - Math.exp(-dt * 6));
    st.bump = Math.max(st.bump * Math.exp(-dt * 8), s.bump || 0);
    // taking apart: one clock E runs 0..1, each part starts a little later than the one above it
    const target = s.explode ? 1 : 0;
    const rate = dt / 1.6;
    st.E = target > st.E ? Math.min(target, st.E + rate) : Math.max(target, st.E - rate * 1.25);
    st.explode = smoother(st.E);
    // opening the lid: a spring, so it settles with a small bounce like a real hinge
    st.openV += ((s.open ? 1 : 0) - st.open) * 70 * dt;
    st.openV *= Math.exp(-dt * 10);
    st.open = Math.max(-0.05, Math.min(1.08, st.open + st.openV * dt));
    // cutaway: the two clipping planes slide from outside the robot to its centre
    st.cut += ((s.cut ? 1 : 0) - st.cut) * (1 - Math.exp(-dt * 3.2));
    const c = smoother(clamp01(st.cut));
    cutPlanes[0].constant = 0.3 + (-0.09 - 0.3) * c;   // removes x > -0.09 ...
    cutPlanes[1].constant = 0.3 + (0.0 - 0.3) * c;     // ... where z < 0: the side facing the camera

    lift.position.y = 0.11 * st.lift + 0.09 * st.explode;
    hinge.rotation.z = Math.max(0, st.open) * 1.12;
    spinner.rotation.y = -s.spin;
    wheel.wheelL.spin.rotation.z = -s.wheelL;
    wheel.wheelR.spin.rotation.z = -s.wheelR;
    wheel.wheelL.hub.position.y = 0.036 - 0.013 * st.lift;
    wheel.wheelR.hub.position.y = 0.036 - 0.013 * st.lift;
    mb.rotation.z = s.brush;
    sb.rotation.y = s.side;
    if (rotor) rotor.rotation.y = s.fan || 0;
    ball.rotation.z = -(s.caster || 0);

    for (const p of Object.values(parts)) {
      const local = smoother(clamp01((st.E - p.delay) / 0.65));
      p.group.position.copy(p.base).addScaledVector(p.explode, local).addScaledVector(p.slide, Math.max(0, st.open));
    }
    parts.bumper.group.position.x -= 0.007 * st.bump;
    parts.lidar.group.position.y -= 0.0195 * (1 - st.lidarUp);

    const op = 1 - 0.84 * st.xray;
    for (const m of shellMats) { m.opacity = op; m.depthWrite = op > 0.6; }

    const g = s.glow || {};
    wrap.userData.glow = g.battery || 0;
    imuMat.userData.glow = g.imu || 0;
    boardTop.userData.glow = g.traces || 0;
    cliffMat.userData.glow = g.cliff || 0;
    for (const p of Object.values(parts)) {
      const on = s.hover === p.name || (s.hover === 'wheel' && p.name.startsWith('wheel'));
      p.h += ((on ? 1 : 0) - p.h) * k;
      for (const m of p.mats) {
        if (!m.emissive) continue;
        const glow = m.userData.glowable ? m.userData.glow || 0 : 0;
        m.emissive.copy(black).lerp(highlight, Math.min(1, 0.3 * p.h + glow));
      }
    }

    for (const w of [wheel.wheelL, wheel.wheelR]) {
      const a = w === wheel.wheelL ? s.wheelL : s.wheelR;
      w.encLed.emissiveIntensity = Math.floor((a * 16) / (Math.PI * 2)) % 2 ? 2.6 : 0.15;
    }
    ledMat.emissiveIntensity = s.lidarOn ? 1.4 : 0;
    emitMat.emissiveIntensity = s.lidarOn ? 3 : 0;
  }

  // Which labelled part does a raycast hit belong to?
  function pick(obj) {
    for (let o = obj; o; o = o.parent) {
      if (o.name && o.name.startsWith('part:')) {
        const n = o.name.slice(5);
        return n.startsWith('wheel') ? 'wheel' : n;
      }
      if (o === root) break;
    }
    return null;
  }

  const _v = new THREE.Vector3();
  function anchorWorld(name) {
    const p = parts[name === 'wheel' ? 'wheelL' : name];   // the wheel on the camera's side
    return p.group.localToWorld(_v.copy(p.anchor));
  }

  // the lidar centre in world space, for the laser fan in the studio
  const lidarCenter = new THREE.Vector3(-0.03, 0.0945, 0);
  function lidarWorld(out) { return parts.lidar.group.localToWorld(out.copy(lidarCenter)); }

  return { root, update, parts, pick, anchorWorld, lidarWorld, state: st, cutPlanes };
}

// Parts listed on "Robot up close". nav: used to work out where the robot is.
export const PARTS = [
  { id: 'lidar', nav: true },
  { id: 'wheel', nav: true },
  { id: 'pcb', nav: true },
  { id: 'bumper', nav: true },
  { id: 'battery' },
  { id: 'fan' },
  { id: 'brush' },
  { id: 'side' },
  { id: 'bin' },
  { id: 'caster' },
  { id: 'cover' },
  { id: 'body' },
];
