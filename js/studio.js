// "Robot up close": a product photo studio for the high-detail robot.
// Own scene, lights and camera. It only draws when something changes: a quick frame while
// things move, then one refined frame (multisampling + ambient occlusion) once it is still.
// Four views of the same robot: whole, open (lid on its hinge), cutaway, taken apart.
// Picking a part plays a short live demo of what that part does.

import * as THREE from 'three';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { GTAOPass } from 'three/addons/postprocessing/GTAOPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { HorizontalBlurShader } from 'three/addons/shaders/HorizontalBlurShader.js';
import { VerticalBlurShader } from 'three/addons/shaders/VerticalBlurShader.js';
import { createRobotModel, PARTS } from './robot-model.js';
import { clamp, damp } from './util.js';

export const STUDIO_BG = 0xecebe7;
const TAU = Math.PI * 2;
const RAYS = 90;

export class Studio {
  constructor(renderer) {
    this.r = renderer;
    this.active = false;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(STUDIO_BG);
    this.camera = new THREE.PerspectiveCamera(24, 1, 0.05, 30);
    this.robot = createRobotModel({ studio: true });
    this.scene.add(this.robot.root);
    this.mode = 'whole';
    this.demo = null;
    this.time = 0;
    this.ticks = 0;
    this.heading = 0;
    this.idle = 10;
    this.pose = {
      lidarOn: true, spin: 0.9, wheelL: 0.35, wheelR: 1.1, brush: 0.5, side: 0.35, fan: 0, caster: 0,
      bump: 0, lifted: false, xray: 0, explode: false, open: false, cut: false, hover: null, glow: {},
    };
    this.view = { yaw: -0.95, pitch: 0.36, dist: 1.0, tx: 0, ty: 0.07, tz: 0, userYaw: 0, userPitch: 0, auto: 0, focus: null };
    this.size = { w: 0, h: 0, pr: 0 };
    this._environment();
    this._lights();
    this._contactShadow();
    this._laserFan();
    this.robot.update(this.pose, 0.016);
  }

  // A small invisible "room" with softboxes, baked into a reflection map, like product photography.
  _environment() {
    const env = new THREE.Scene();
    const room = new THREE.Mesh(new THREE.BoxGeometry(14, 9, 14), new THREE.MeshBasicMaterial({ color: 0x3c3c3a, side: THREE.BackSide }));
    room.position.y = 3.5;
    env.add(room);
    const panel = (w, h, pos, rot, k) => {
      const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial({ color: new THREE.Color(k, k, k), side: THREE.DoubleSide }));
      m.position.set(...pos);
      m.rotation.set(...rot);
      env.add(m);
    };
    panel(6, 6, [0, 7.8, 0], [Math.PI / 2, 0, 0], 3.2);
    panel(1.6, 6, [-6.8, 3.2, 0.5], [0, Math.PI / 2, 0], 4.5);
    panel(1.6, 6, [6.8, 3.2, -0.8], [0, -Math.PI / 2, 0], 2.2);
    panel(7, 2.2, [0, 2.2, 6.8], [0, Math.PI, 0], 1.4);
    panel(14, 14, [0, -0.95, 0], [-Math.PI / 2, 0, 0], 0.75);
    const pm = new THREE.PMREMGenerator(this.r);
    this.scene.environment = pm.fromScene(env, 0.015).texture;
    this.scene.environmentIntensity = 1.0;
    pm.dispose();
  }

  _lights() {
    const key = new THREE.DirectionalLight(0xfffaf3, 2.1);
    key.position.set(0.55, 1.7, 0.85);
    key.target.position.set(0, 0.1, 0);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    const c = key.shadow.camera;
    c.left = -0.5; c.right = 0.5; c.top = 0.5; c.bottom = -0.5; c.near = 0.8; c.far = 3.2;
    c.updateProjectionMatrix();
    key.shadow.bias = -0.00025;
    key.shadow.normalBias = 0.0025;
    const rim = new THREE.DirectionalLight(0xf2f4ff, 0.9);
    rim.position.set(-1.3, 0.9, -1.1);
    this.scene.add(key, key.target, rim);
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(8, 8).rotateX(-Math.PI / 2), new THREE.ShadowMaterial({ opacity: 0.1 }));
    floor.receiveShadow = true;
    this.floor = floor;
    this.scene.add(floor);
  }

  // Soft contact shadow: render the robot's depth from below, blur it, lay it on the floor.
  _contactShadow() {
    const S = 1.4, H = 0.8;
    const g = new THREE.Group();
    g.position.y = 0.0006;
    this.scene.add(g);
    this.csRT = new THREE.WebGLRenderTarget(512, 512);
    this.csRT.texture.generateMipmaps = false;
    this.csBlurRT = new THREE.WebGLRenderTarget(512, 512);
    this.csBlurRT.texture.generateMipmaps = false;
    const plane = new THREE.PlaneGeometry(S, S).rotateX(Math.PI / 2);
    this.csPlane = new THREE.Mesh(plane, new THREE.MeshBasicMaterial({ map: this.csRT.texture, opacity: 0.85, transparent: true, depthWrite: false }));
    this.csPlane.renderOrder = 1;
    this.csPlane.scale.y = -1;
    g.add(this.csPlane);
    this.csBlurPlane = new THREE.Mesh(plane);
    this.csBlurPlane.visible = false;
    g.add(this.csBlurPlane);
    this.csCam = new THREE.OrthographicCamera(-S / 2, S / 2, S / 2, -S / 2, 0, H);
    this.csCam.rotation.x = Math.PI / 2;
    g.add(this.csCam);
    const dm = new THREE.MeshDepthMaterial();
    dm.userData.darkness = { value: 1.35 };
    dm.onBeforeCompile = (shader) => {
      shader.uniforms.darkness = dm.userData.darkness;
      shader.fragmentShader = 'uniform float darkness;\n' + shader.fragmentShader.replace(
        'gl_FragColor = vec4( vec3( 1.0 - fragCoordZ ), opacity );',
        'gl_FragColor = vec4( vec3( 0.0 ), ( 1.0 - fragCoordZ ) * darkness );');
    };
    dm.depthTest = false;
    dm.depthWrite = false;
    this.csDepth = dm;
    this.hBlur = new THREE.ShaderMaterial(HorizontalBlurShader);
    this.hBlur.depthTest = false;
    this.vBlur = new THREE.ShaderMaterial(VerticalBlurShader);
    this.vBlur.depthTest = false;
    this.csKey = '';
  }

  // The laser fan: a sweep of rays from the turret, shown during the lidar demo.
  _laserFan() {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(RAYS * 6), 3).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(RAYS * 8), 4).setUsage(THREE.DynamicDrawUsage));
    this.fan = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, depthWrite: false, toneMapped: false }));
    this.fan.frustumCulled = false;
    this.fan.visible = false;
    this.scene.add(this.fan);
    this.fanOn = 0;
  }

  _updateFan(dt) {
    const want = this.demo === 'lidar' && this.robot.state.explode < 0.5 ? 1 : 0;
    this.fanOn += (want - this.fanOn) * (1 - Math.exp(-dt * 5));
    this.fan.visible = this.fanOn > 0.02;
    if (!this.fan.visible) return;
    const c = this.robot.lidarWorld(new THREE.Vector3());
    const pos = this.fan.geometry.attributes.position.array;
    const col = this.fan.geometry.attributes.color.array;
    const L = 0.36;
    for (let k = 0; k < RAYS; k++) {
      const a = this.pose.spin - (k / RAYS) * TAU * 0.9;
      const fade = Math.pow(1 - k / RAYS, 2.2) * this.fanOn;
      const dx = Math.cos(a), dz = Math.sin(a);
      pos.set([c.x + dx * 0.046, c.y, c.z + dz * 0.046, c.x + dx * L, c.y, c.z + dz * L], k * 6);
      col.set([1, 0.31, 0, 0.9 * fade, 1, 0.31, 0, 0.05 * fade], k * 8);
    }
    this.fan.geometry.attributes.position.needsUpdate = true;
    this.fan.geometry.attributes.color.needsUpdate = true;
  }

  _blur(amount) {
    const r = this.r;
    this.csBlurPlane.visible = true;
    this.csBlurPlane.material = this.hBlur;
    this.hBlur.uniforms.tDiffuse.value = this.csRT.texture;
    this.hBlur.uniforms.h.value = amount / 256;
    r.setRenderTarget(this.csBlurRT);
    r.render(this.csBlurPlane, this.csCam);
    this.csBlurPlane.material = this.vBlur;
    this.vBlur.uniforms.tDiffuse.value = this.csBlurRT.texture;
    this.vBlur.uniforms.v.value = amount / 256;
    r.setRenderTarget(this.csRT);
    r.render(this.csBlurPlane, this.csCam);
    this.csBlurPlane.visible = false;
  }

  _renderContactShadow() {
    const r = this.r, s = this.scene;
    const bg = s.background;
    s.background = null;
    this.csPlane.visible = false;
    this.floor.visible = false;
    const fanVis = this.fan.visible;
    this.fan.visible = false;
    s.overrideMaterial = this.csDepth;
    const cc = r.getClearColor(new THREE.Color()), ca = r.getClearAlpha();
    r.setClearColor(0x000000, 0);
    r.setRenderTarget(this.csRT);
    r.clear();
    r.render(s, this.csCam);
    s.overrideMaterial = null;
    this._blur(3.2);
    this._blur(1.3);
    r.setRenderTarget(null);
    r.setClearColor(cc, ca);
    s.background = bg;
    this.csPlane.visible = true;
    this.floor.visible = true;
    this.fan.visible = fanVis;
  }

  _ensureComposer(w, h) {
    const pr = this.r.getPixelRatio();
    if (this.composer && this.size.w === w && this.size.h === h && this.size.pr === pr) return;
    this.size = { w, h, pr };
    if (!this.composer) {
      const rt = new THREE.WebGLRenderTarget(w * pr, h * pr, { type: THREE.HalfFloatType, samples: 4 });
      const c = new EffectComposer(this.r, rt);
      c.addPass(new RenderPass(this.scene, this.camera));
      const ao = new GTAOPass(this.scene, this.camera, w, h);
      ao.updateGtaoMaterial({ radius: 0.035, distanceExponent: 1, thickness: 1, scale: 1.15, samples: 12, distanceFallOff: 1, screenSpaceRadius: false });
      ao.updatePdMaterial({ lumaPhi: 10, depthPhi: 2, normalPhi: 3, radius: 4, radiusExponent: 1, rings: 2, samples: 12 });
      ao.blendIntensity = 0.9;
      c.addPass(ao);
      c.addPass(new OutputPass());
      this.composer = c;
    }
    this.composer.setPixelRatio(pr);
    this.composer.setSize(w, h);
  }

  // ---------------- controls ----------------
  setMode(mode) {
    this.mode = mode;
    this.pose.explode = mode === 'apart';
    this.pose.open = mode === 'open';
    this.pose.cut = mode === 'cut';
  }
  get exploded() { return this.mode === 'apart'; }
  setHover(name) { this.pose.hover = name; }
  focus(name) { this.view.focus = name; }
  setDemo(name) { this.demo = name; this.ticks = 0; }
  touch() { this.idle = 0; }
  invalidate() { this.lastSig = null; }

  // Everything that changes the picture. If none of it moved, the last frame stays on screen.
  _signature(w, h) {
    const c = this.camera, st = this.robot.state, p = this.pose, g = p.glow;
    let hs = 0;
    for (const part of Object.values(this.robot.parts)) hs += part.h;
    const v = [c.position.x, c.position.y, c.position.z, c.quaternion.x, c.quaternion.y, c.quaternion.z, c.quaternion.w, c.fov,
      st.E, st.open, st.cut, st.bump, st.lidarUp, this.robot.root.rotation.y, hs,
      p.spin, p.wheelL, p.wheelR, p.brush, p.side, p.fan, p.caster,
      g.traces || 0, g.imu || 0, g.battery || 0, g.cliff || 0, this.fanOn, w, h];
    let s = '';
    for (const x of v) s += x.toFixed(3) + ',';
    return s;
  }

  // ---------------- live demos ----------------
  _demo(dt) {
    const p = this.pose, d = this.demo, t = (this.time += dt);
    if (d === 'lidar') p.spin += dt * TAU * 1.3;
    if (d === 'wheel') { p.wheelL += dt * 5; p.wheelR += dt * 5; this.ticks += (dt * 5 / TAU) * 1000; }
    if (d === 'brush') p.brush += dt * 16;
    if (d === 'side') p.side += dt * 6;
    if (d === 'fan') p.fan += dt * 38;
    if (d === 'caster') p.caster += dt * 4;
    p.bump = d === 'bumper' && Math.sin(t * 3) > 0.55 ? 1 : 0;
    p.glow = {
      traces: d === 'pcb' ? 0.32 + 0.22 * Math.sin(t * 3.2) : 0,
      imu: d === 'pcb' ? 0.55 + 0.45 * Math.sin(t * 7) : 0,
      battery: d === 'battery' ? 0.12 + 0.08 * Math.sin(t * 2.5) : 0,
      cliff: d === 'body' ? 0.5 + 0.5 * Math.sin(t * 6) : 0,
    };
    // the gyroscope demo turns the robot gently, and the chip reads the angle
    const root = this.robot.root;
    const turn = d === 'pcb' && this.mode !== 'cut' ? 0.38 * Math.sin(t * 0.9) : 0;
    root.rotation.y = damp(root.rotation.y, turn, 4, dt);
    this.heading = (-root.rotation.y * 180) / Math.PI;
  }

  // ---------------- camera ----------------
  // inset: the parts of the canvas covered by glass panels; the robot is framed in the rest.
  _camera(dt, w, h, inset) {
    const v = this.view;
    const freeW = Math.max(60, w - inset.left - inset.right), freeH = Math.max(60, h - inset.top - inset.bottom);
    const aspect = freeW / freeH;
    v.frame = damp(v.frame || 0, this.pose.explode ? 1 : 0, 2.6, dt);
    v.openF = damp(v.openF || 0, this.pose.open ? 1 : 0, 2.6, dt);
    const f = v.frame, o = v.openF;
    const fov = aspect < 1 ? 30 : 24;
    const t = Math.tan(THREE.MathUtils.degToRad(fov / 2));
    const tanV = t * (freeH / h), tanH = t * (freeW / h);
    const halfW = 0.25 + 0.1 * f + 0.05 * o, halfH = 0.15 + 0.25 * f + 0.09 * o;
    let dist = Math.max(halfW / tanH, halfH / tanV) * 1.05, tx = 0, ty = 0.05 + 0.25 * f + 0.06 * o, tz = 0;
    if (v.focus) {
      const p = this.robot.anchorWorld(v.focus);
      tx = p.x * 0.85; ty = p.y; tz = p.z * 0.85;
      dist *= 0.62;
    }
    v.dist = damp(v.dist, dist, 4, dt);
    v.tx = damp(v.tx, tx, 4, dt); v.ty = damp(v.ty, ty, 4, dt); v.tz = damp(v.tz, tz, 4, dt);
    this.idle += dt;
    const yaw = v.yaw + v.userYaw + v.auto, pitch = clamp(v.pitch - 0.08 * f + 0.05 * o + v.userPitch, 0.05, 1.35);
    const cam = this.camera;
    cam.fov = fov;
    cam.aspect = w / h;
    cam.position.set(v.tx + v.dist * Math.cos(pitch) * Math.cos(yaw), v.ty + v.dist * Math.sin(pitch), v.tz + v.dist * Math.cos(pitch) * Math.sin(yaw));
    cam.lookAt(v.tx, v.ty, v.tz);
    const cx = inset.left + freeW / 2, cy = inset.top + freeH / 2;
    cam.setViewOffset(w, h, -(cx - w / 2), -(cy - h / 2), w, h);
    cam.updateProjectionMatrix();
  }

  // ---------------- render (into the renderer's whole canvas) ----------------
  render(dt, w, h, inset) {
    this._demo(dt);
    this.robot.update(this.pose, dt);
    this._camera(dt, w, h, inset);
    this._updateFan(dt);
    this.w = w; this.h = h;
    const now = performance.now();
    const sig = this._signature(w, h);
    const r = this.r;
    if (sig !== this.lastSig) {
      this.lastSig = sig;
      this.changedAt = now;
      this.refined = false;
      // the soft shadow only needs redrawing while the robot changes shape or turns
      const st = this.robot.state;
      const key = [st.E, st.open, st.bump, this.robot.root.rotation.y].map((x) => x.toFixed(4)).join();
      if (key !== this.csKey) { this._renderContactShadow(); this.csKey = key; }
      r.setRenderTarget(null);
      r.setViewport(0, 0, w, h);
      r.setScissorTest(false);
      r.setClearColor(STUDIO_BG, 1);
      r.shadowMap.needsUpdate = true;
      r.render(this.scene, this.camera);          // quick frame while things move
      return;
    }
    if (!this.refined && now - this.changedAt > 220) {
      this._ensureComposer(w, h);
      r.setRenderTarget(null);
      r.setViewport(0, 0, w, h);
      r.setScissorTest(false);
      r.setClearColor(STUDIO_BG, 1);
      r.shadowMap.needsUpdate = true;
      this.composer.render(dt);                   // one refined frame once it is still
      this.refined = true;
    }
  }

  // client point -> part id. rect: the canvas's client rect.
  pick(clientX, clientY, rect) {
    if (!rect) return null;
    const ndc = new THREE.Vector2(((clientX - rect.left) / rect.width) * 2 - 1, -(((clientY - rect.top) / rect.height) * 2 - 1));
    const rc = new THREE.Raycaster();
    rc.setFromCamera(ndc, this.camera);
    const hit = rc.intersectObject(this.robot.root, true).find((h) => h.object !== this.fan);
    return hit ? this.robot.pick(hit.object) : null;
  }

  // Canvas pixel positions of every part's label, pushed away from the robot's centre.
  labelPositions() {
    const out = [];
    if (!this.w) return out;
    const W = this.w, Hh = this.h;
    const toScreen = (v) => ({ x: ((v.x + 1) / 2) * W, y: ((1 - v.y) / 2) * Hh, z: v.z });
    const v = new THREE.Vector3(0, 0.05 + 0.18 * this.robot.state.explode, 0).project(this.camera);
    const c = toScreen(v);
    for (const p of PARTS) {
      if (!this.robot.parts[p.id === 'wheel' ? 'wheelL' : p.id]) continue;
      const s = toScreen(v.copy(this.robot.anchorWorld(p.id)).project(this.camera));
      const dx = s.x - c.x, dy = s.y - c.y, L = Math.hypot(dx, dy) || 1;
      out.push({ id: p.id, x: s.x + (dx / L) * 34, y: s.y + (dy / L) * 34, ax: s.x, ay: s.y, ok: s.z < 1 });
    }
    return out;
  }

  // Where a part sits on screen (for the live readout chip).
  screenOf(id) {
    if (!this.w) return null;
    const v = this.robot.anchorWorld(id).clone().project(this.camera);
    return { x: ((v.x + 1) / 2) * this.w, y: ((1 - v.y) / 2) * this.h, ok: v.z < 1 };
  }
}
