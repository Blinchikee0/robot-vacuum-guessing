// Everything drawn with WebGL in the simulation.
// Two renderers, each with its own canvas inside its own element, so both scroll with the page:
//  - the world view (the whole flat; also hosts the photo studio on the "Robot up close" page)
//  - the robot camera that follows the robot.
// They share one scene. Layers keep each camera's lights and objects apart.

import * as THREE from 'three';
import { W, H, ROBOT_R, LEGS, footprint } from './world.js';
import { FX0, FY0, FW, FH } from './floor.js';
import { createRobotModel } from './robot-model.js';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { GTAOPass } from 'three/addons/postprocessing/GTAOPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { clamp, damp, easeOutCubic } from './util.js';

export const WALL_H = 0.42;
export const WORLD_BG = 0xefeeea;
const CLOSE_BG = 0xf6f5f2;
const L_WORLD = 1;   // seen only by the world camera (walls, furniture, guesses, world sun)
const L_CLOSE = 2;   // seen only by the robot camera (its own sun)
const COL = {
  wall: 0xf6f5f1, wallTop: 0x56524d, fabric: 0xd9d5cc, fabric2: 0xe6e2da, white: 0xf3f2ef,
  top: 0xeae9e5, stone: 0xd4d1ca, black: 0x1f1f1e, line: 0xcfcec8, accent: 0xff4f00,
  laser: 0xff3300, ink: 0x121212, leaf: 0x8fa684, leaf2: 0x7d9572, basket: 0xd5c9b2,
  mattress: 0xfbfbf9, blanket: 0xdfd8cc, glass: 0xffffff,
};

function box(parent, w, h, d, x, y, z, mat, cast = true, r = 0.012) {
  const rr = Math.min(r, Math.min(w, h, d) / 2 - 0.0005);
  const geo = rr > 0.003 ? new RoundedBoxGeometry(w, h, d, 3, rr) : new THREE.BoxGeometry(w, h, d);
  const m = new THREE.Mesh(geo, mat);
  m.position.set(x, y, z);
  m.castShadow = cast;
  m.receiveShadow = true;
  parent.add(m);
  return m;
}

function setLayer(obj, layer) { obj.traverse((o) => o.layers.set(layer)); }

function makeRenderer(canvas) {
  const r = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
  r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.75));
  r.shadowMap.enabled = true;
  r.shadowMap.type = THREE.PCFSoftShadowMap;
  r.shadowMap.autoUpdate = false;
  r.toneMapping = THREE.NeutralToneMapping;
  r.outputColorSpace = THREE.SRGBColorSpace;
  return r;
}

function envFor(r) {
  const pm = new THREE.PMREMGenerator(r);
  const t = pm.fromScene(new RoomEnvironment(), 0.04).texture;
  pm.dispose();
  return t;
}

export class Stage {
  constructor(worldCanvas, closeCanvas) {
    this.rW = makeRenderer(worldCanvas);
    this.rW.localClippingEnabled = true;          // the studio's cutaway
    this.rC = makeRenderer(closeCanvas);
    this.renderer = this.rW;
    this.cW = worldCanvas;
    this.cC = closeCanvas;
    this.scene = new THREE.Scene();
    // each renderer needs its own reflection map (a map made in one WebGL context cannot be used in another)
    this.envW = envFor(this.rW);
    this.envC = envFor(this.rC);
    this.scene.environmentIntensity = 0.5;
    this.maxAniso = this.rW.capabilities.getMaxAnisotropy();

    this.camW = new THREE.PerspectiveCamera(26, 1, 0.1, 60);
    this.camW.layers.enable(L_WORLD);
    this.camC = new THREE.PerspectiveCamera(34, 1, 0.02, 30);
    this.camC.layers.enable(L_CLOSE);

    this.view = { el: 1.42, elT: 0.98, az: -0.55, azT: 0, tilt: 0, tiltT: 0, base: null, focus: 0, focusT: 0, fx: W / 2, fy: H / 2 };
    this.close = { yaw: 0, pitch: 0.5, dist: 0.8, ty: 0.06, userYaw: 0, userPitch: 0, mode: 'default', until: 0, x: 0, y: 0, init: false };
    this.intro = 0;
    this.reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    if (this.reduced) this.intro = 10;
    this.quality = { ao: false, lowfx: false };   // ambient occlusion stays for the photo studio, where the picture is still
    this.frameN = 0;
    this.sizeW = { w: 0, h: 0 };
    this.sizeC = { w: 0, h: 0 };
    this.inset = { left: 0, top: 0, right: 0, bottom: 0 };

    this._lights();
    this.items = {};
    this.walls = [];
    this._observe();
  }

  // keep each canvas exactly the size of its element
  _observe() {
    const fit = (canvas, r, size, onResize) => {
      const ro = new ResizeObserver(() => {
        const w = Math.max(2, Math.round(canvas.clientWidth)), h = Math.max(2, Math.round(canvas.clientHeight));
        if (w === size.w && h === size.h) return;
        size.w = w; size.h = h;
        r.setSize(w, h, false);
        if (onResize) onResize(w, h);
      });
      ro.observe(canvas);
    };
    fit(this.cW, this.rW, this.sizeW, () => { this.composerSize = null; });
    fit(this.cC, this.rC, this.sizeC);
  }

  _lights() {
    const s = this.scene;
    const hemi = new THREE.HemisphereLight(0xffffff, 0xe6e2da, 0.9);
    hemi.layers.enableAll();
    s.add(hemi);
    // sun for the whole flat: tight frustum computed from the flat's corners
    const sun = new THREE.DirectionalLight(0xfff6ec, 1.8);
    sun.position.set(W / 2 - 3.2, 9, H / 2 + 4.5);
    sun.target.position.set(W / 2, 0, H / 2);
    sun.castShadow = true;
    const size = window.matchMedia('(pointer: coarse)').matches ? 2048 : 3072;
    sun.shadow.mapSize.set(size, size);
    const c = sun.shadow.camera;
    c.left = -5.8; c.right = 5.8; c.top = 5.05; c.bottom = -4.65; c.near = 6.6; c.far = 14;
    c.updateProjectionMatrix();
    sun.shadow.bias = -0.0004;
    sun.shadow.normalBias = 0.015;
    sun.layers.set(L_WORLD);
    s.add(sun, sun.target);
    this.sun = sun;
    // a second sun that follows the robot, with a small sharp shadow map for the robot camera
    const sunC = new THREE.DirectionalLight(0xfff6ec, 1.8);
    sunC.castShadow = true;
    sunC.shadow.mapSize.set(1024, 1024);
    const cc = sunC.shadow.camera;
    cc.left = -0.65; cc.right = 0.65; cc.top = 0.65; cc.bottom = -0.65; cc.near = 0.5; cc.far = 8;
    cc.updateProjectionMatrix();
    sunC.shadow.bias = -0.0003;
    sunC.shadow.normalBias = 0.004;
    sunC.layers.set(L_CLOSE);
    s.add(sunC, sunC.target);
    this.sunC = sunC;
  }

  _tex(canvas, mip) {
    const t = new THREE.CanvasTexture(canvas);
    t.colorSpace = THREE.SRGBColorSpace;
    t.anisotropy = this.maxAniso;
    if (!mip) { t.generateMipmaps = false; t.minFilter = THREE.LinearFilter; }
    return t;
  }

  _plane(tex, y, order, opts = {}) {
    const g = new THREE.PlaneGeometry(FW, FH);
    g.rotateX(-Math.PI / 2);
    // the floor is lacquered: a faint clear coat picks up soft reflections
    const mat = opts.lit
      ? new THREE.MeshPhysicalMaterial({ map: tex, roughness: 0.62, clearcoat: 0.32, clearcoatRoughness: 0.42 })
      : new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false, toneMapped: false });
    const m = new THREE.Mesh(g, mat);
    m.position.set(FX0 + FW / 2, y, FY0 + FH / 2);
    m.renderOrder = order;
    if (opts.lit) m.receiveShadow = true;
    this.scene.add(m);
    return m;
  }

  // ---------------- static world ----------------
  build(world, canvases) {
    this.texBase = this._tex(canvases.base, true);
    this.texDust = this._tex(canvases.dust, false);
    this.texPath = this._tex(canvases.path, false);
    this.texField = this._tex(canvases.field, false);
    this.floor = this._plane(this.texBase, 0, 0, { lit: true });
    this.dustPlane = this._plane(this.texDust, 0.001, 1);
    this._initMask();
    this.dustPlane.material.alphaMap = this.maskRT.texture;
    this.dustPlane.material.needsUpdate = true;
    this.fieldPlane = this._plane(this.texField, 0.002, 2);
    this.fieldPlane.visible = false;
    this.pathPlane = this._plane(this.texPath, 0.003, 3);

    // walls: warm white with a dark cut top, like a cutaway architectural model
    const mSide = new THREE.MeshStandardMaterial({ color: COL.wall, roughness: 0.88 });
    const mTop = new THREE.MeshStandardMaterial({ color: COL.wallTop, roughness: 0.75 });
    const wallGroup = new THREE.Group();
    world.walls.forEach((w) => {
      const g = new THREE.BoxGeometry(w.x1 - w.x0, WALL_H, w.y1 - w.y0);
      g.translate(0, WALL_H / 2, 0);
      const m = new THREE.Mesh(g, [mSide, mSide, mTop, mSide, mSide, mSide]);
      m.position.set((w.x0 + w.x1) / 2, 0, (w.y0 + w.y1) / 2);
      m.castShadow = true; m.receiveShadow = true;
      wallGroup.add(m);
      this.walls.push(m);
    });
    setLayer(wallGroup, L_WORLD);
    this.scene.add(wallGroup);

    const fabric = (color) => new THREE.MeshPhysicalMaterial({ color, roughness: 0.92, sheen: 0.7, sheenRoughness: 0.75, sheenColor: new THREE.Color(0xffffff) });
    const mats = {
      fabric: fabric(COL.fabric),
      fabric2: fabric(COL.fabric2),
      white: new THREE.MeshPhysicalMaterial({ color: COL.white, roughness: 0.55, clearcoat: 0.25, clearcoatRoughness: 0.4 }),
      top: new THREE.MeshStandardMaterial({ color: COL.top, roughness: 0.5 }),
      stone: new THREE.MeshPhysicalMaterial({ color: COL.stone, roughness: 0.35, clearcoat: 0.6, clearcoatRoughness: 0.2 }),
      black: new THREE.MeshStandardMaterial({ color: COL.black, roughness: 0.38, metalness: 0.6 }),
      line: new THREE.MeshStandardMaterial({ color: COL.line, roughness: 0.8 }),
      glass: new THREE.MeshPhysicalMaterial({ color: COL.glass, roughness: 0.08, transparent: true, opacity: 0.32, depthWrite: false, clearcoat: 1 }),
      leaf: new THREE.MeshStandardMaterial({ color: COL.leaf, roughness: 0.7, side: THREE.DoubleSide }),
      leaf2: new THREE.MeshStandardMaterial({ color: COL.leaf2, roughness: 0.7, side: THREE.DoubleSide }),
      basket: new THREE.MeshStandardMaterial({ color: COL.basket, roughness: 0.95, side: THREE.DoubleSide }),
      mattress: fabric(COL.mattress),
      blanket: fabric(COL.blanket),
      led: new THREE.MeshStandardMaterial({ color: 0x331100, emissive: COL.accent, emissiveIntensity: 1.6 }),
    };
    for (const it of world.items) {
      const g = buildItem(it, mats);
      setLayer(g, L_WORLD);
      this.scene.add(g);
      this.items[it.id] = { group: g, baseY: 0 };
    }

    const fp = new THREE.Mesh(new THREE.PlaneGeometry(1, 1).rotateX(-Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: COL.accent, transparent: true, opacity: 0.22, depthWrite: false, toneMapped: false }));
    fp.position.y = 0.004; fp.renderOrder = 4; fp.visible = false;
    fp.layers.set(L_WORLD);
    this.scene.add(fp);
    this.footprint = fp;

    this._dynamic();
  }

  _dynamic() {
    const s = this.scene;
    this.robot = createRobotModel();
    s.add(this.robot.root);

    const shape = new THREE.Shape();
    shape.moveTo(0.05, 0); shape.lineTo(-0.028, 0.021); shape.lineTo(-0.014, 0); shape.lineTo(-0.028, -0.021); shape.closePath();
    const pg = new THREE.ShapeGeometry(shape); pg.rotateX(-Math.PI / 2);
    const pm = new THREE.MeshBasicMaterial({ color: COL.accent, transparent: true, opacity: 0.85, depthWrite: false, toneMapped: false });
    this.parts = new THREE.InstancedMesh(pg, pm, 4000);
    this.parts.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    this.parts.frustumCulled = false;
    this.parts.renderOrder = 6;
    const white = new THREE.Color(1, 1, 1);
    for (let i = 0; i < 4000; i++) this.parts.setColorAt(i, white);
    this.parts.layers.set(L_WORLD);
    s.add(this.parts);
    this.selPrev = -1;

    const MAXSEG = 300;
    const tg = new THREE.BufferGeometry();
    tg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAXSEG * 18), 3).setUsage(THREE.DynamicDrawUsage));
    tg.setDrawRange(0, 0);
    this.trail = new THREE.Mesh(tg, new THREE.MeshBasicMaterial({ color: COL.accent, toneMapped: false, transparent: true, opacity: 0.95, depthWrite: false, side: THREE.DoubleSide }));
    this.trail.position.y = 0.004;
    this.trail.renderOrder = 4;
    this.trail.frustumCulled = false;
    this.trailMax = MAXSEG;
    s.add(this.trail);

    const ghost = new THREE.Group();
    const disc = new THREE.Mesh(new THREE.CircleGeometry(ROBOT_R, 48).rotateX(-Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: COL.accent, transparent: true, opacity: 0.26, depthWrite: false, toneMapped: false }));
    const wshape = new THREE.Shape();
    wshape.moveTo(ROBOT_R * 0.95, 0); wshape.lineTo(ROBOT_R * 0.35, 0.07); wshape.lineTo(ROBOT_R * 0.35, -0.07); wshape.closePath();
    const wedge = new THREE.Mesh(new THREE.ShapeGeometry(wshape).rotateX(-Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: COL.accent, transparent: true, opacity: 0.9, depthWrite: false, toneMapped: false }));
    wedge.position.y = 0.001;
    ghost.add(disc, wedge);
    ghost.position.y = 0.006; ghost.renderOrder = 5;
    disc.renderOrder = 5; wedge.renderOrder = 5;
    setLayer(ghost, L_WORLD);
    s.add(ghost);
    this.ghost = ghost;

    const og = ghost.clone(true);
    og.traverse((o) => { if (o.material) { o.material = o.material.clone(); o.material.color.set(COL.ink); o.material.opacity *= 0.55; } });
    og.visible = false;
    s.add(og);
    this.odoGhost = og;

    this.hitPts = new THREE.InstancedMesh(new THREE.BoxGeometry(0.022, 0.022, 0.022),
      new THREE.MeshBasicMaterial({ color: COL.laser, toneMapped: false }), 360);
    this.hitPts.frustumCulled = false;
    this.hitPts.count = 0;
    this.hitPts.layers.set(L_WORLD);
    s.add(this.hitPts);
    const RAYS = 360;
    const rg = new THREE.BufferGeometry();
    rg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(RAYS * 6), 3).setUsage(THREE.DynamicDrawUsage));
    rg.setAttribute('color', new THREE.BufferAttribute(new Float32Array(RAYS * 8), 4).setUsage(THREE.DynamicDrawUsage));
    this.rays = new THREE.LineSegments(rg, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, depthWrite: false, toneMapped: false }));
    this.rays.frustumCulled = false;
    this.rays.renderOrder = 7;
    s.add(this.rays);

    this.insp = new THREE.InstancedMesh(new THREE.PlaneGeometry(0.045, 0.045).rotateX(-Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, depthWrite: false, toneMapped: false }), 360);
    this.insp.frustumCulled = false; this.insp.count = 0; this.insp.renderOrder = 8;
    for (let i = 0; i < 360; i++) this.insp.setColorAt(i, white);
    this.insp.layers.set(L_WORLD);
    s.add(this.insp);
    this.inspLines = new THREE.LineSegments(new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(new Float32Array(360 * 6), 3)),
      new THREE.LineBasicMaterial({ color: COL.ink, transparent: true, opacity: 0.25, depthWrite: false, toneMapped: false }));
    this.inspLines.frustumCulled = false; this.inspLines.renderOrder = 8;
    this.inspLines.layers.set(L_WORLD);
    s.add(this.inspLines);
  }

  markDirty(which) { this[which].needsUpdate = true; }

  // ---------------- dust mask on the GPU ----------------
  // White = dust stays, black = cleaned. Each renderer keeps its own copy, painted with the same stamps.
  _initMask() {
    const w = 1024, h = Math.round((1024 * FH) / FW);
    this.maskRT = new THREE.WebGLRenderTarget(w, h, { depthBuffer: false, stencilBuffer: false });
    this.maskRT.texture.generateMipmaps = false;
    this.maskRT.texture.minFilter = THREE.LinearFilter;
    const cam = new THREE.OrthographicCamera(-FW / 2, FW / 2, FH / 2, -FH / 2, 0.1, 20);
    cam.position.set(FX0 + FW / 2, 10, FY0 + FH / 2);
    cam.up.set(0, 0, -1);
    cam.lookAt(FX0 + FW / 2, 0, FY0 + FH / 2);
    this.brushCam = cam;
    this.brushScene = new THREE.Scene();
    const g = new THREE.CircleGeometry(1, 40).rotateX(-Math.PI / 2);
    const m = new THREE.MeshBasicMaterial({ color: 0x000000, toneMapped: false });
    this.brushes = [];
    for (let i = 0; i < 48; i++) {
      const b = new THREE.Mesh(g, m);
      b.visible = false;
      b.frustumCulled = false;
      this.brushScene.add(b);
      this.brushes.push(b);
    }
    this.stamps = [];
    this.stampR = 0.16;
    this.maskClear = { W: true, C: true };
  }

  clearDustMask() { this.maskClear.W = true; this.maskClear.C = true; this.stamps.length = 0; }

  queueSweep(x0, y0, x1, y1, r) {
    const n = Math.max(1, Math.ceil(Math.hypot(x1 - x0, y1 - y0) / 0.03));
    for (let i = 1; i <= n; i++) this.stamps.push(x0 + ((x1 - x0) * i) / n, y0 + ((y1 - y0) * i) / n);
    this.stampR = r;
  }

  _paintMask(r, which) {
    r.setRenderTarget(this.maskRT);
    if (this.maskClear[which]) {
      r.setClearColor(0xffffff, 1);
      r.clear(true, false, false);
      this.maskClear[which] = false;
    }
    if (this.stamps.length) {
      const auto = r.autoClear;
      r.autoClear = false;
      const B = this.brushes;
      let k = 0;
      const flush = () => {
        r.render(this.brushScene, this.brushCam);
        for (let j = 0; j < k; j++) B[j].visible = false;
        k = 0;
      };
      for (let i = 0; i < this.stamps.length; i += 2) {
        const b = B[k++];
        b.position.set(this.stamps[i], 0, this.stamps[i + 1]);
        b.scale.setScalar(this.stampR);
        b.visible = true;
        if (k === B.length) flush();
      }
      if (k) flush();
      r.autoClear = auto;
    }
    r.setRenderTarget(null);
  }

  updateTrail(pts) {
    const a = this.trail.geometry.attributes.position.array;
    const hw = 0.008, y = 0;
    let n = 0;
    const start = Math.max(0, pts.length - 2 * (this.trailMax + 1));
    for (let i = start; i + 3 < pts.length && n < this.trailMax; i += 2) {
      const x0 = pts[i], y0 = pts[i + 1], x1 = pts[i + 2], y1 = pts[i + 3];
      if (x0 !== x0 || x1 !== x1) continue;
      const dx = x1 - x0, dy = y1 - y0, L = Math.hypot(dx, dy);
      if (L < 1e-5) continue;
      const nx = (-dy / L) * hw, ny = (dx / L) * hw;
      const o = n * 18;
      a[o] = x0 + nx; a[o + 1] = y; a[o + 2] = y0 + ny;
      a[o + 3] = x0 - nx; a[o + 4] = y; a[o + 5] = y0 - ny;
      a[o + 6] = x1 + nx; a[o + 7] = y; a[o + 8] = y1 + ny;
      a[o + 9] = x1 + nx; a[o + 10] = y; a[o + 11] = y1 + ny;
      a[o + 12] = x0 - nx; a[o + 13] = y; a[o + 14] = y0 - ny;
      a[o + 15] = x1 - nx; a[o + 16] = y; a[o + 17] = y1 - ny;
      n++;
    }
    this.trail.geometry.setDrawRange(0, n * 6);
    this.trail.geometry.attributes.position.needsUpdate = true;
  }

  // ---------------- per-frame updates ----------------
  updateRobot(robot, vis, dt) {
    const root = this.robot.root;
    root.position.set(robot.x, 0, robot.y);
    root.rotation.y = -robot.th;
    this.robot.update(vis, dt);
  }

  updateParticles(pf, sel, weightView) {
    const a = this.parts.instanceMatrix.array;
    const n = pf.n, avg = 1 / n;
    for (let i = 0; i < n; i++) {
      const rel = pf.w[i] / avg;
      let k = weightView ? clamp(Math.sqrt(rel) * 0.9, 0.2, 3.2) : clamp(0.8 + 0.2 * Math.sqrt(rel), 0.75, 1.5);
      if (i === sel) k = Math.max(k, 1.8);
      const c = Math.cos(pf.th[i]) * k, s = Math.sin(pf.th[i]) * k, o = i * 16;
      a[o] = c; a[o + 1] = 0; a[o + 2] = s; a[o + 3] = 0;
      a[o + 4] = 0; a[o + 5] = k; a[o + 6] = 0; a[o + 7] = 0;
      a[o + 8] = -s; a[o + 9] = 0; a[o + 10] = c; a[o + 11] = 0;
      a[o + 12] = pf.x[i]; a[o + 13] = 0.012; a[o + 14] = pf.y[i]; a[o + 15] = 1;
    }
    this.parts.count = n;
    this.parts.instanceMatrix.needsUpdate = true;
    if (sel !== this.selPrev) {
      if (this.selPrev >= 0) this.parts.setColorAt(this.selPrev, new THREE.Color(1, 1, 1));
      if (sel >= 0) this.parts.setColorAt(sel, new THREE.Color(0.02, 0.02, 0.02));
      this.parts.instanceColor.needsUpdate = true;
      this.selPrev = sel;
    }
  }

  updateGhosts(belief, odo, showOdo) {
    this.ghost.position.x = belief.x; this.ghost.position.z = belief.y;
    this.ghost.rotation.y = -belief.th;
    this.odoGhost.visible = showOdo;
    this.odoGhost.position.set(odo.x, 0.005, odo.y);
    this.odoGhost.rotation.y = -odo.th;
  }

  updateLidar(on, hits, fresh, lx, ly, th, spin, allRays, showRays) {
    if (!on) { this.hitPts.count = 0; this.rays.geometry.setDrawRange(0, 0); return; }
    if (fresh) {
      const a = this.hitPts.instanceMatrix.array;
      let n = 0;
      for (let i = 0; i < 360; i++) {
        const x = hits[2 * i];
        if (x !== x) continue;
        const o = n * 16;
        a.fill(0, o, o + 16);
        a[o] = 1; a[o + 5] = 1; a[o + 10] = 1; a[o + 15] = 1;
        a[o + 12] = x; a[o + 13] = 0.085; a[o + 14] = hits[2 * i + 1];
        n++;
      }
      this.hitPts.count = n;
      this.hitPts.instanceMatrix.needsUpdate = true;
    }
    if (!showRays) { this.rays.geometry.setDrawRange(0, 0); return; }
    const pos = this.rays.geometry.attributes.position.array;
    const col = this.rays.geometry.attributes.color.array;
    const count = allRays ? 360 : 70;
    const spinDeg = Math.floor(((spin % (Math.PI * 2)) + Math.PI * 2) * 180 / Math.PI) % 360;
    let n = 0;
    for (let k = 0; k < count; k++) {
      const i = allRays ? k : (spinDeg - k + 720) % 360;
      const x = hits[2 * i];
      if (x !== x) continue;
      const o = n * 6, c = n * 8;
      pos[o] = lx; pos[o + 1] = 0.085; pos[o + 2] = ly;
      pos[o + 3] = x; pos[o + 4] = 0.085; pos[o + 5] = hits[2 * i + 1];
      const alpha = allRays ? 0.3 : 0.75 * (1 - k / count);
      col[c] = 1; col[c + 1] = 0.2; col[c + 2] = 0; col[c + 3] = alpha;
      col[c + 4] = 1; col[c + 5] = 0.2; col[c + 6] = 0; col[c + 7] = alpha * 0.6;
      n++;
    }
    this.rays.geometry.setDrawRange(0, n * 2);
    this.rays.geometry.attributes.position.needsUpdate = true;
    this.rays.geometry.attributes.color.needsUpdate = true;
  }

  updateInspect(pts, k, px, py) {
    if (!pts || !k) { this.insp.count = 0; this.inspLines.geometry.setDrawRange(0, 0); return; }
    const a = this.insp.instanceMatrix.array;
    const lp = this.inspLines.geometry.attributes.position.array;
    const hit = new THREE.Color(0.07, 0.07, 0.07), miss = new THREE.Color(1, 0.31, 0);
    for (let i = 0; i < k; i++) {
      const o = i * 16;
      a.fill(0, o, o + 16);
      a[o] = 1; a[o + 5] = 1; a[o + 10] = 1; a[o + 15] = 1;
      a[o + 12] = pts[3 * i]; a[o + 13] = 0.03; a[o + 14] = pts[3 * i + 1];
      this.insp.setColorAt(i, pts[3 * i + 2] ? hit : miss);
      lp[i * 6] = px; lp[i * 6 + 1] = 0.03; lp[i * 6 + 2] = py;
      lp[i * 6 + 3] = pts[3 * i]; lp[i * 6 + 4] = 0.03; lp[i * 6 + 5] = pts[3 * i + 1];
    }
    this.insp.count = k;
    this.insp.instanceMatrix.needsUpdate = true;
    this.insp.instanceColor.needsUpdate = true;
    this.inspLines.geometry.setDrawRange(0, k * 2);
    this.inspLines.geometry.attributes.position.needsUpdate = true;
  }

  moveItem(it) {
    const e = this.items[it.id];
    if (e) e.group.position.set(it.cx, e.group.position.y, it.cy);
  }

  showFootprint(it) {
    if (!it) { this.footprint.visible = false; return; }
    const f = footprint(it);
    this.footprint.visible = true;
    this.footprint.position.set((f.x0 + f.x1) / 2, 0.004, (f.y0 + f.y1) / 2);
    this.footprint.scale.set(f.x1 - f.x0 + 0.06, 1, f.y1 - f.y0 + 0.06);
  }

  setCloseMode(mode, seconds) {
    this.close.mode = mode;
    this.close.until = seconds ? performance.now() + seconds * 1000 : 0;
  }

  // ---------------- cameras ----------------
  // The flat is framed in the part of the canvas that glass panels do not cover (this.inset).
  _fitWorld() {
    const cam = this.camW;
    const { w, h } = this.sizeW;
    const ins = this.inset;
    const freeW = Math.max(80, w - ins.left - ins.right), freeH = Math.max(80, h - ins.top - ins.bottom);
    const aspect = freeW / freeH;
    cam.aspect = w / h;
    const baseT = aspect < 0.9 ? -Math.PI / 2 : 0;
    if (this.view.base === null) this.view.base = baseT;
    this.view.base = damp(this.view.base, baseT, 4, 1 / 60);
    const f0 = this.view.focus;
    const el = this.view.el + (1.5 - this.view.el) * f0 + this.view.tilt, az = this.view.az + this.view.base;
    const target = new THREE.Vector3(W / 2, 0, H / 2);
    const dir = new THREE.Vector3(Math.sin(az) * Math.cos(el), Math.sin(el), Math.cos(az) * Math.cos(el));
    const fwd = dir.clone().negate();
    const right = new THREE.Vector3().crossVectors(fwd, new THREE.Vector3(0, 1, 0)).normalize();
    const up = new THREE.Vector3().crossVectors(right, fwd);
    const t = Math.tan(THREE.MathUtils.degToRad(cam.fov / 2));
    const tY = t * (freeH / h) * 0.97;
    const tX = t * (freeW / h) * 0.97;
    let d = 1;
    const p = new THREE.Vector3();
    for (const x of [-0.15, W + 0.15]) for (const z of [-0.15, H + 0.15]) for (const y of [0, WALL_H]) {
      p.set(x, y, z).sub(target);
      const px = p.dot(right), py = p.dot(up), pz = p.dot(fwd);
      d = Math.max(d, Math.abs(px) / tX - pz, Math.abs(py) / tY - pz);
    }
    const f = this.view.focus;
    if (f > 0.001) {
      target.lerp(new THREE.Vector3(this.view.fx, 0, this.view.fy), f);
      d += (5.2 - d) * f;
    }
    cam.position.copy(target).addScaledVector(dir, d);
    cam.up.set(0, 1, 0);
    cam.lookAt(target);
    const cx = ins.left + freeW / 2, cy = ins.top + freeH / 2;
    cam.setViewOffset(w, h, -(cx - w / 2), -(cy - h / 2), w, h);
    cam.updateProjectionMatrix();
  }

  _updateClose(dt, robot) {
    const c = this.close;
    if (c.until && performance.now() > c.until) { c.mode = 'default'; c.until = 0; }
    const T = {
      default: { pitch: 0.5, dist: 0.82, ty: 0.06 },
      lidar: { pitch: 0.78, dist: 0.6, ty: 0.08 },
      wheels: { pitch: 0.2, dist: 0.66, ty: 0.05 },
      lifted: { pitch: 0.32, dist: 0.85, ty: 0.12 },
    }[c.mode] || { pitch: 0.5, dist: 0.82, ty: 0.06 };
    if (!c.init) { c.x = robot.x; c.y = robot.y; c.init = true; c.pitch = T.pitch; c.dist = T.dist; }
    c.x = damp(c.x, robot.x, 10, dt); c.y = damp(c.y, robot.y, 10, dt);
    c.pitch = damp(c.pitch, clamp(T.pitch + c.userPitch, 0.08, 1.4), 4, dt);
    c.dist = damp(c.dist, T.dist, 4, dt);
    c.ty = damp(c.ty, T.ty, 4, dt);
    const phi = 1.15 + c.userYaw;
    const cam = this.camC;
    cam.aspect = this.sizeC.w / this.sizeC.h;
    const tx = c.x, tz = c.y, ty = c.ty + this.robot.state.lift * 0.11;
    cam.position.set(tx + c.dist * Math.cos(c.pitch) * Math.cos(phi), ty + c.dist * Math.sin(c.pitch), tz + c.dist * Math.cos(c.pitch) * Math.sin(phi));
    cam.lookAt(tx, ty, tz);
    cam.updateProjectionMatrix();
    // the robot camera's own sun rides along with the robot
    this.sunC.position.set(robot.x - 1.1, 2.4, robot.y + 1.5);
    this.sunC.target.position.set(robot.x, 0, robot.y);
    this.sunC.target.updateMatrixWorld();
  }

  _introStep() {
    if (this.intro > 3) return;
    if (!this.introStart) this.introStart = performance.now();
    this.intro = (performance.now() - this.introStart) / 1000;
    const t = this.intro;
    this.walls.forEach((m, i) => {
      const k = easeOutCubic(clamp((t - i * 0.035) / 0.55, 0, 1));
      m.scale.y = Math.max(0.001, k);
    });
    let i = 0;
    for (const id in this.items) {
      const e = this.items[id];
      const k = easeOutCubic(clamp((t - 0.35 - i * 0.04) / 0.5, 0, 1));
      e.group.position.y = (1 - k) * 0.9;
      e.group.visible = k > 0.001;
      i++;
    }
  }

  // Ambient occlusion pass for the world view (darkens corners and contact areas).
  _worldComposer() {
    const { w, h } = this.sizeW;
    const pr = this.rW.getPixelRatio();
    const key = w + 'x' + h + '@' + pr;
    if (this.composerW && this.composerSize === key) return this.composerW;
    if (!this.composerW) {
      const rt = new THREE.WebGLRenderTarget(w * pr, h * pr, { type: THREE.HalfFloatType, samples: 4 });
      const c = new EffectComposer(this.rW, rt);
      c.addPass(new RenderPass(this.scene, this.camW));
      const ao = new GTAOPass(this.scene, this.camW, w, h);
      ao.updateGtaoMaterial({ radius: 0.32, distanceExponent: 1, thickness: 1, scale: 1, samples: 16, distanceFallOff: 1, screenSpaceRadius: false });
      ao.updatePdMaterial({ lumaPhi: 10, depthPhi: 2, normalPhi: 3, radius: 4, radiusExponent: 1, rings: 2, samples: 16 });
      ao.blendIntensity = 0.8;
      c.addPass(ao);
      c.addPass(new OutputPass());
      this.composerW = c;
    }
    this.composerW.setPixelRatio(pr);
    this.composerW.setSize(w, h);
    this.composerSize = key;
    return this.composerW;
  }

  // When frames run slow: drop ambient occlusion, then glass blur, then pixels, then shadow detail.
  lowerQuality() {
    if (!this.quality.lowfx) { this.quality.lowfx = true; return 'lowfx'; }
    for (const r of [this.rW, this.rC]) {
      const pr = r.getPixelRatio();
      if (pr > 1.01) r.setPixelRatio(Math.max(1, pr - 0.25));
    }
    this.sizeW.w = 0; this.sizeC.w = 0;
    this.composerSize = null;
    const sh = this.sun.shadow;
    if (sh.mapSize.x > 1024) {
      sh.mapSize.set(sh.mapSize.x / 2, sh.mapSize.y / 2);
      if (sh.map) { sh.map.dispose(); sh.map = null; }
    }
    return 'pixels';
  }

  // ---------------- render ----------------
  // ctx: { world: bool, close: bool } which views are on screen.
  render(dt, robot, ctx) {
    this._introStep();
    const v = this.view;
    v.el = damp(v.el, v.elT, this.intro < 2.5 ? 2.2 : 5, dt);
    v.az = damp(v.az, v.azT, this.intro < 2.5 ? 1.8 : 3, dt);
    v.tilt = damp(v.tilt, v.tiltT, 3, dt);
    v.focus = damp(v.focus, v.focusT, 3.5, dt);
    v.fx = damp(v.fx, robot.x, 6, dt);
    v.fy = damp(v.fy, robot.y, 6, dt);
    // the canvases follow their elements; make sure the drawing buffers match before drawing
    for (const [cv, r, size] of [[this.cW, this.rW, this.sizeW], [this.cC, this.rC, this.sizeC]]) {
      const w = Math.max(2, Math.round(cv.clientWidth)), h = Math.max(2, Math.round(cv.clientHeight));
      if (w !== size.w || h !== size.h) { size.w = w; size.h = h; r.setSize(w, h, false); }
    }

    if (this.maskClear.W || this.maskClear.C || this.stamps.length) {
      this._paintMask(this.rW, 'W');
      this._paintMask(this.rC, 'C');
      this.stamps.length = 0;
    }

    if (this.studio && this.studio.active) {
      if (ctx.world) this.studio.render(dt, this.sizeW.w, this.sizeW.h, this.inset);
      return;
    }
    if (ctx.world && this.sizeW.w > 2) {
      this._fitWorld();
      this.scene.environment = this.envW;
      this.rW.setClearColor(WORLD_BG, 1);
      // the flat's big shadow map is refreshed every other frame (only the robot moves in it)
      if ((this.frameN++ & 1) === 0 || this.intro < 3) this.rW.shadowMap.needsUpdate = true;
      if (this.quality.ao) this._worldComposer().render(dt);
      else this.rW.render(this.scene, this.camW);
    }
    if (ctx.close && this.sizeC.w > 2) {
      this._updateClose(dt, robot);
      this.scene.environment = this.envC;
      this.rC.setClearColor(CLOSE_BG, 1);
      this.rC.shadowMap.needsUpdate = true;
      this.rC.render(this.scene, this.camC);
    }
  }

  // Client point -> floor point (metres) through the world camera.
  floorAt(clientX, clientY) {
    const a = this.cW.getBoundingClientRect();
    const ndc = new THREE.Vector2(((clientX - a.left) / a.width) * 2 - 1, -(((clientY - a.top) / a.height) * 2 - 1));
    const rc = new THREE.Raycaster();
    rc.setFromCamera(ndc, this.camW);
    const p = new THREE.Vector3();
    if (!rc.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), p)) return null;
    return { x: p.x, y: p.z };
  }

  // World point -> pixels inside the world canvas.
  project(x, y, z) {
    const v = new THREE.Vector3(x, y, z).project(this.camW);
    return { x: ((v.x + 1) / 2) * this.sizeW.w, y: ((1 - v.y) / 2) * this.sizeW.h, ok: v.z < 1 };
  }

  pickPart(clientX, clientY) {
    const b = this.cC.getBoundingClientRect();
    const ndc = new THREE.Vector2(((clientX - b.left) / b.width) * 2 - 1, -(((clientY - b.top) / b.height) * 2 - 1));
    const rc = new THREE.Raycaster();
    rc.setFromCamera(ndc, this.camC);
    const hit = rc.intersectObject(this.robot.root, true)[0];
    return hit ? this.robot.pick(hit.object) : null;
  }
}

// ---------------- furniture ----------------
function buildItem(it, M) {
  const g = new THREE.Group();
  const inner = new THREE.Group();
  g.add(inner);
  const rot = { n: 0, s: Math.PI, e: -Math.PI / 2, w: Math.PI / 2 }[it.back || 'n'];
  inner.rotation.y = rot;
  const swap = it.back === 'e' || it.back === 'w';
  const lw = swap ? it.d : it.w, ld = swap ? it.w : it.d;

  switch (it.kind) {
    case 'sofa':
    case 'armchair': {
      const arm = it.kind === 'sofa' ? 0.2 : 0.15, back = 0.2;
      box(inner, lw, 0.12, ld, 0, 0.09, 0, M.fabric, true, 0.03);
      box(inner, lw, 0.27, back, 0, 0.285, -ld / 2 + back / 2, M.fabric, true, 0.06);
      box(inner, arm, 0.17, ld, -lw / 2 + arm / 2, 0.235, 0, M.fabric, true, 0.06);
      box(inner, arm, 0.17, ld, lw / 2 - arm / 2, 0.235, 0, M.fabric, true, 0.06);
      const seats = it.kind === 'sofa' ? 3 : 1;
      const sw = (lw - 2 * arm - 0.02 * (seats + 1)) / seats;
      const sd = ld - back - 0.03;
      for (let k = 0; k < seats; k++) {
        const x = -lw / 2 + arm + 0.02 + sw / 2 + k * (sw + 0.02);
        box(inner, sw, 0.085, sd, x, 0.1925, -ld / 2 + back + sd / 2, M.fabric2, true, 0.035);
        box(inner, sw, 0.2, 0.09, x, 0.34, -ld / 2 + back + 0.045, M.fabric2, true, 0.04);
      }
      for (const sx of [-1, 1]) for (const sz of [-1, 1])
        box(inner, 0.03, 0.03, 0.03, sx * (lw / 2 - 0.06), 0.015, sz * (ld / 2 - 0.06), M.black, true, 0.006);
      break;
    }
    case 'table': {
      const { s, inset } = LEGS.table;
      box(inner, lw, 0.025, ld, 0, it.h, 0, M.glass, false);
      for (const sx of [-1, 1]) for (const sz of [-1, 1])
        box(inner, s, it.h, s, sx * (lw / 2 - inset - s / 2), it.h / 2, sz * (ld / 2 - inset - s / 2), M.black);
      break;
    }
    case 'chair': {
      const { s, inset } = LEGS.chair;
      for (const sx of [-1, 1]) for (const sz of [-1, 1])
        box(inner, s, 0.44, s, sx * (lw / 2 - inset - s / 2), 0.22, sz * (ld / 2 - inset - s / 2), M.black);
      box(inner, lw, 0.03, ld, 0, 0.455, 0, M.white);
      box(inner, lw, 0.3, 0.03, 0, 0.62, -ld / 2 + 0.015, M.white);
      break;
    }
    case 'counter': {
      box(inner, lw, 0.48, ld, 0, 0.24, 0, M.white);
      box(inner, lw, 0.03, ld + 0.02, 0, 0.495, 0.01, M.stone);
      box(inner, 0.5, 0.012, 0.34, -0.7, 0.506, 0.04, M.black, false);
      for (const [x, z] of [[0.65, -0.1], [0.65, 0.15], [0.95, -0.1], [0.95, 0.15]]) {
        const hob = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.07, 0.004, 32), M.black);
        hob.position.set(x, 0.512, z); inner.add(hob);
      }
      for (let k = -3; k <= 3; k++) box(inner, 0.004, 0.42, 0.004, k * 0.48, 0.24, ld / 2 + 0.002, M.line, false);
      break;
    }
    case 'bed': {
      box(inner, lw, 0.2, ld, 0, 0.12, 0, M.white, true, 0.02);
      box(inner, lw - 0.06, 0.13, ld - 0.06, 0, 0.285, 0.01, M.mattress, true, 0.05);
      box(inner, lw, 0.5, 0.06, 0, 0.25, -ld / 2 + 0.03, M.white, true, 0.025);
      for (const sx of [-1, 1]) box(inner, 0.62, 0.08, 0.34, sx * 0.42, 0.39, -ld / 2 + 0.3, M.fabric2, true, 0.04);
      box(inner, lw - 0.03, 0.035, ld * 0.6, 0, 0.368, ld / 2 - ld * 0.3 - 0.015, M.blanket, true, 0.017);
      break;
    }
    case 'media': {
      box(inner, lw, it.h - 0.02, ld, 0, (it.h - 0.02) / 2, 0, M.white);
      box(inner, lw, 0.02, ld, 0, it.h - 0.01, 0, M.top);
      box(inner, 0.9, 0.06, 0.09, 0, it.h + 0.03, -0.03, M.black);
      for (const x of [-lw / 4, lw / 4]) box(inner, 0.004, it.h - 0.08, 0.004, x, it.h / 2, ld / 2 + 0.002, M.line, false);
      break;
    }
    case 'wardrobe': {
      box(inner, lw, it.h, ld, 0, it.h / 2, 0, M.white);
      box(inner, lw, 0.015, ld, 0, it.h + 0.007, 0, M.top);
      for (const x of [-lw / 6, lw / 6]) box(inner, 0.004, it.h - 0.04, 0.004, x, it.h / 2, ld / 2 + 0.002, M.line, false);
      break;
    }
    case 'cabinet': {
      box(inner, lw, it.h - 0.02, ld, 0, (it.h - 0.02) / 2, 0, M.white);
      box(inner, lw, 0.02, ld, 0, it.h - 0.01, 0, M.top);
      break;
    }
    case 'plant': {
      const pot = new THREE.Mesh(new THREE.CylinderGeometry(it.r, it.r * 0.8, 0.3, 40), M.white);
      pot.position.y = 0.15; pot.castShadow = pot.receiveShadow = true; inner.add(pot);
      const soil = new THREE.Mesh(new THREE.CylinderGeometry(it.r * 0.92, it.r * 0.92, 0.01, 32), M.black);
      soil.position.y = 0.295; inner.add(soil);
      // long curved leaves fanning out of the pot
      const leafGeo = new THREE.SphereGeometry(1, 12, 8, 0, Math.PI * 2, 0, Math.PI);
      leafGeo.scale(0.035, 0.006, 0.16);
      leafGeo.translate(0, 0, 0.15);
      for (let k = 0; k < 14; k++) {
        const l = new THREE.Mesh(leafGeo, k % 2 ? M.leaf : M.leaf2);
        const a = (k / 14) * Math.PI * 2 + (k % 3) * 0.3;
        l.position.set(0, 0.3 + (k % 4) * 0.02, 0);
        l.rotation.set(-0.5 - (k % 3) * 0.25, a, 0);
        l.castShadow = true;
        inner.add(l);
      }
      break;
    }
    case 'basket': {
      const b = new THREE.Mesh(new THREE.CylinderGeometry(it.r, it.r * 0.92, it.h, 40, 1, true), M.basket);
      b.position.y = it.h / 2; b.castShadow = b.receiveShadow = true; inner.add(b);
      const bottom = new THREE.Mesh(new THREE.CircleGeometry(it.r * 0.92, 32).rotateX(-Math.PI / 2), M.basket);
      bottom.position.y = 0.01; inner.add(bottom);
      box(inner, 0.26, 0.08, 0.2, 0.0, it.h - 0.02, 0.0, M.fabric2);
      break;
    }
    case 'dock': {
      box(inner, lw, it.h, ld, 0, it.h / 2, 0, M.black);
      box(inner, lw * 0.9, 0.008, 0.07, 0, 0.004, ld / 2 + 0.035, M.black);
      box(inner, 0.03, 0.006, 0.004, 0, 0.075, ld / 2 + 0.002, M.led, false);
      break;
    }
  }
  g.position.set(it.cx, 0, it.cy);
  return g;
}
