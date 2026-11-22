// Glue: owns the simulation loop, connects it to the 3D views and the interface.

import { World, DistField, DOCK, W, H, ROBOT_R, footprint } from './world.js';
import { planRoute } from './planner.js';
import { Robot } from './robot.js';
import { Controller } from './controller.js';
import { ParticleFilter } from './filter.js';
import { Coverage, CLEAN_R } from './coverage.js';
import { Stage } from './scene.js';
import { DustLayer, PathLayer, makeCanvas, paintBase, paintField, PX_BASE, PX_DYN } from './floor.js';
import { Studio } from './studio.js';
import { PARTS } from './robot-model.js';
import { STR, fmt } from './i18n.js';
import { motionAlphas } from './motion-noise.js';
import { clamp, wrap } from './util.js';

const $ = (s) => document.querySelector(s);
const $$ = (s) => Array.from(document.querySelectorAll(s));
const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

// ---------------------------------------------------------------- state
const S = {
  lang: 'en',
  lidarOn: true, slip: 0.06, noise: 0.02, beams: 36, distrust: 1, gyro: true,
  n: 1000, speed: 4, view: 'model',
  show: { rays: true, ghost: false, field: false, plan: false },
  mode: 'run', phase: 0, moveLeft: 0, weightView: false, allRays: false,
  page: 'sim', part: 'lidar', rbMode: 'whole', tour: -1, mapStale: false,
  tasks: [false, false, false], blindT: 0, slipT: 0,
  kidnapAnim: null, search: null,
  runDoneAt: -1, dragHintUntil: 0, interacted: false,
  inspectPt: null, sel: -1,
};
const T = () => STR[S.lang];

// ---------------------------------------------------------------- simulation objects
const world = new World();          // the real apartment
let mapWorld = world.clone();       // what the robot believes the apartment looks like
let df = new DistField(mapWorld);   // its map, as distance-to-wall
let route = planRoute(df, DOCK);
const robot = new Robot(world); robot.reset(DOCK);
const ctrl = new Controller(); ctrl.setRoute(route);
const pf = new ParticleFilter(4000); pf.init(DOCK, S.n);
const cov = new Coverage(world);
const scan = new Float32Array(360), hits = new Float32Array(720);
hits.fill(NaN);
let scanFresh = false;
let belief = { ...DOCK };
let odoMark = { ...robot.odo };
let simTime = 0, tickAcc = 0, pathAcc = 0;
const trail = { belief: [] };
let trailDirty = false;
const history = [];   // { t, e, blind }
let lastBump = 0;

// ---------------------------------------------------------------- rendering objects
const canvases = { base: makeCanvas(PX_BASE), field: makeCanvas(PX_DYN) };
const dust = new DustLayer();
const paths = new PathLayer();
canvases.dust = dust.canvas;
canvases.path = paths.canvas;
const roomLabels = () => ({ living: T().room_living, kitchen: T().room_kitchen, bedroom: T().room_bedroom, hall: T().room_hall });
paintBase(canvases.base, roomLabels());
paintField(canvases.field, df);

const stage = new Stage($('#glWorld'), $('#glClose'));
stage.build(world, canvases);
const studio = new Studio(stage.renderer);   // "Robot up close"
stage.studio = studio;
let lastSweep = { x: robot.x, y: robot.y };

// which 3D views are on screen (on phones they scroll away)
const onScreen = { world: true, close: true };
const io = new IntersectionObserver((entries) => {
  for (const e of entries) onScreen[e.target.id === 'world' ? 'world' : 'close'] = e.isIntersecting;
}, { rootMargin: '120px' });
io.observe($('#world'));
io.observe($('#closeup'));

// ---------------------------------------------------------------- filter parameters
const alphas = () => motionAlphas(S.gyro, S.distrust);
const sigma = () => Math.max(0.03, 1.3 * S.noise) + 0.015;
const temper = () => 5 / S.beams;

// ---------------------------------------------------------------- simulation step
function physicsStep(dt) {
  simTime += dt;
  if (robot.lifted) return;
  const cmd = ctrl.update(dt, belief);
  const r = robot.step(dt, cmd.v, cmd.w, { slip: S.slip, gyro: S.gyro });
  if (r.bumped) { ctrl.bump(); lastBump = performance.now(); }
  // between filter cycles the belief is carried forward by odometry
  belief.x += r.ods * Math.cos(belief.th + r.odth / 2);
  belief.y += r.ods * Math.sin(belief.th + r.odth / 2);
  belief.th = wrap(belief.th + r.odth);
  cov.mark(robot.x, robot.y);

  pathAcc += dt;
  if (pathAcc >= 0.1) {
    pathAcc = 0;
    const b = trail.belief, n = b.length;
    if (n >= 2 && Math.hypot(b[n - 2] - belief.x, b[n - 1] - belief.y) > 0.25) b.push(NaN, NaN);
    b.push(belief.x, belief.y);
    if (b.length > 2 * 260) b.splice(0, 2);
    trailDirty = true;
  }

  if (!S.lidarOn) S.blindT += dt;
  if (S.slip >= 0.18 && Math.abs(cmd.v) > 0.05) S.slipT += dt;

  if (S.mode === 'run') {
    tickAcc += dt;
    if (tickAcc >= 0.2) { tickAcc -= 0.2; filterCycle(); }
  }

  if (ctrl.state === 'done' && S.runDoneAt < 0) {
    S.runDoneAt = simTime;
    showToast(fmt(T().run_done, { p: Math.round(cov.pct) }), 3200);
  }
  if (S.runDoneAt >= 0 && simTime - S.runDoneAt > 3.5) newRun(false);
}

function phaseMove() {
  pf.move(odoMark, robot.odo, alphas());
  odoMark = { ...robot.odo };
}
function phaseSense() {
  if (!S.lidarOn) return false;
  robot.scan(scan, S.noise, hits);
  scanFresh = true;
  return true;
}
function phaseWeigh() { pf.weigh(scan, df, { beams: S.beams, sigma: sigma(), temper: temper() }); }
function phaseResample() { pf.resample(df); }

function filterCycle() {
  phaseMove();
  if (phaseSense()) {
    phaseWeigh();
    if (pf.neff < pf.n * 0.6) phaseResample();
  }
  finishCycle();
}

function finishCycle() {
  pf.computeEstimate();
  belief = { ...pf.est };
  const e = Math.hypot(robot.x - belief.x, robot.y - belief.y);
  history.push({ t: simTime, e, blind: !S.lidarOn });
  while (history.length && simTime - history[0].t > 90) history.shift();
  if (S.search) {
    if (!S.lidarOn) S.search.ok = 0;
    else if (e < 0.12 && pf.spread < 0.3) {
      S.search.ok += 0.2;
      if (S.search.ok >= 1) {
        S.search.found = simTime - S.search.since - 1;
        completeTask(2);
        S.search.doneAt = simTime;
      }
    } else S.search.ok = 0;
  }
}

function newRun(full) {
  S.runDoneAt = -1;
  dust.reset();
  stage.clearDustMask();
  cov.reset();
  trail.belief.length = 0;
  trailDirty = true;
  if (full) {
    robot.reset(DOCK);
    pf.init(DOCK, S.n);
    belief = { ...DOCK };
    odoMark = { ...robot.odo };
    history.length = 0;
    S.search = null; S.kidnapAnim = null;
    hits.fill(NaN);
    route = planRoute(df, DOCK);
  } else {
    route = planRoute(df, belief);
  }
  ctrl.setRoute(route);
  drawPlan();
  lastSweep = { x: robot.x, y: robot.y };
}

// The planned route is drawn once into a floor texture, only when it changes.
function drawPlan() {
  paths.draw({ plan: route, showPlan: S.show.plan });
  stage.markDirty('texPath');
}

// ---------------------------------------------------------------- kidnapping
function startKidnap(target) {
  const to = target || pickKidnapSpot();
  S.kidnapAnim = { t: 0, from: { x: robot.x, y: robot.y, th: robot.th }, to };
  robot.lifted = true;
  stage.setCloseMode('lifted', 2.5);
}
function pickKidnapSpot() {
  const trueDf = new DistField(world, 0.05, ROBOT_R + 0.05);
  for (let k = 0; k < 200; k++) {
    const p = trueDf.randomFree();
    if (Math.hypot(p.x - robot.x, p.y - robot.y) > 2.6 && !world.collides(p.x, p.y, ROBOT_R + 0.02))
      return { x: p.x, y: p.y, th: Math.random() * Math.PI * 2 - Math.PI };
  }
  return { x: 7.2, y: 4.8, th: 2 };
}
function landed() {
  robot.lifted = false;
  lastSweep = { x: robot.x, y: robot.y };
  ctrl.state = ctrl.state === 'done' ? 'done' : 'drive';
  S.search = { since: simTime, ok: 0, found: null, doneAt: null };
  S.interacted = true;
}

// ---------------------------------------------------------------- tasks
function completeTask(i) {
  if (S.tasks[i]) return;
  S.tasks[i] = true;
  const card = $('#task' + (i + 1));
  card.classList.add('done', 'flash');
  setTimeout(() => card.classList.remove('flash'), 800);
  if (S.tasks.every(Boolean)) $('#insight').hidden = false;
}

// ---------------------------------------------------------------- UI: text
function applyText() {
  const t = T();
  document.documentElement.lang = S.lang;
  document.title = t.doc_title;
  for (const el of $$('[data-i18n]')) {
    const k = el.dataset.i18n;
    if (t[k] !== undefined) el.textContent = t[k];
  }
  paintBase(canvases.base, roomLabels());
  stage.markDirty('texBase');
  buildRobotPage();
  updatePhaseUI();
  refreshSliderLabels();
}

function refreshSliderLabels() {
  $('#slipVal').textContent = Math.round(S.slip * 100) + '%';
  $('#nVal').textContent = String(S.n);
  $('#noiseVal').textContent = (S.noise * 100).toFixed(1) + ' ' + T().cm;
  $('#beamsVal').textContent = String(S.beams);
  $('#distrustVal').textContent = S.distrust.toFixed(1) + 'x';
  for (const r of $$('input[type=range]')) {
    const p = ((r.value - r.min) / (r.max - r.min)) * 100;
    r.style.setProperty('--p', p + '%');
  }
}

let toastTimer = 0;
function showToast(text, ms) {
  const el = $('#toast');
  el.textContent = text;
  el.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.hidden = true; }, ms);
}

// ---------------------------------------------------------------- segmented controls
function pressed(group, attr, val) {
  $$(group + ' button').forEach((b) => b.dataset[attr] !== undefined && b.setAttribute('aria-pressed', String(b.dataset[attr] === String(val))));
  syncThumbs(true);
}

// A white "lens" sits behind the chosen option. When the choice changes it stretches across
// to the new option and settles with a spring, like the liquid tab bars of iOS 26.
function initThumbs() {
  for (const seg of $$('.seg, .pages')) {
    const t = document.createElement('span');
    t.className = 'seg-thumb';
    seg.prepend(t);
    seg._thumb = t;
  }
  $('[data-laser="off"]').classList.add('on-accent');
  window.addEventListener('resize', () => syncThumbs(false));
}
function syncThumbs(animate) {
  for (const seg of $$('.seg, .pages')) {
    const t = seg._thumb;
    if (!t) continue;
    const on = seg.querySelector('button[aria-pressed="true"], button[aria-selected="true"]');
    if (!on || !on.offsetWidth) { t.style.opacity = '0'; t._x = undefined; continue; }
    const x = on.offsetLeft, w = on.offsetWidth;
    t.style.opacity = '1';
    t.classList.toggle('accent', on.dataset.laser === 'off');
    const px = t._x, pw = t._w;
    t.style.width = w + 'px';
    t.style.transform = `translateX(${x}px)`;
    if (animate && !reduced && px !== undefined && (px !== x || pw !== w) && t.animate) {
      const L = Math.min(px, x), R = Math.max(px + pw, x + w);
      t.animate([
        { transform: `translateX(${px}px) scaleY(1)`, width: pw + 'px' },
        { transform: `translateX(${L}px) scaleY(0.86)`, width: R - L + 'px', offset: 0.4 },
        { transform: `translateX(${x}px) scaleY(1)`, width: w + 'px' },
      ], { duration: 620, easing: 'cubic-bezier(.3,1.3,.4,1)' });
    }
    t._x = x; t._w = w;
  }
}

function setLidar(on) {
  S.lidarOn = on;
  pressed('#laserSeg', 'laser', on ? 'on' : 'off');
  stage.setCloseMode('lidar', 3.2);
  S.interacted = true;
  if (!on) S.blindT = 0;
}

function bindControls() {
  $$('#laserSeg button').forEach((b) => b.addEventListener('click', () => setLidar(b.dataset.laser === 'on')));
  $$('#viewSeg button').forEach((b) => b.addEventListener('click', () => {
    S.view = b.dataset.view;
    stage.view.elT = S.view === 'plan' ? 1.52 : 0.98;
    pressed('#viewSeg', 'view', S.view);
  }));
  $$('#speedSeg button').forEach((b) => b.addEventListener('click', () => { S.speed = +b.dataset.speed; pressed('#speedSeg', 'speed', S.speed); }));

  const slip = $('#slip');
  slip.addEventListener('input', () => { S.slip = slip.value / 100; refreshSliderLabels(); stage.setCloseMode('wheels', 3.5); S.interacted = true; });
  const np = $('#nParticles');
  const mapN = (v) => Math.round(20 * Math.pow(150, v / 100));
  np.addEventListener('input', () => { S.n = mapN(+np.value); pf.resize(S.n); refreshSliderLabels(); });
  $('#noise').addEventListener('input', (e) => { S.noise = e.target.value / 1000; refreshSliderLabels(); });
  $('#beams').addEventListener('input', (e) => { S.beams = +e.target.value; refreshSliderLabels(); });
  $('#distrust').addEventListener('input', (e) => { S.distrust = e.target.value / 10; refreshSliderLabels(); });

  $('#kidnap').addEventListener('click', () => { if (!S.kidnapAnim && !robot.lifted) startKidnap(); });
  $('#reset').addEventListener('click', () => newRun(true));
  $('#mapUpdate').addEventListener('click', () => {
    mapWorld = world.clone();
    df = new DistField(mapWorld);
    paintField(canvases.field, df); stage.markDirty('texField');
    route = planRoute(df, belief);
    let bi = 0, bd = Infinity;
    route.forEach((p, i) => { const d = Math.hypot(p.x - belief.x, p.y - belief.y); if (d < bd) { bd = d; bi = i; } });
    ctrl.setRoute(route, bi);
    drawPlan();
    S.mapStale = false; $('#mapBanner').hidden = true; markInset();
  });

  $$('.tog').forEach((b) => b.addEventListener('click', () => {
    const k = b.dataset.show;
    const on = b.getAttribute('aria-pressed') !== 'true';
    b.setAttribute('aria-pressed', String(on));
    if (k === 'gyro') { S.gyro = on; return; }
    S.show[k] = on;
    if (k === 'field') stage.fieldPlane.visible = on;
    if (k === 'ghost') $('#legWheels').hidden = !on;
    if (k === 'plan') drawPlan();
  }));

  $$('.tabs button').forEach((b) => b.addEventListener('click', () => selectTab(b.dataset.tab)));

  $('#btnRun').addEventListener('click', () => { S.mode = 'run'; S.weightView = false; S.allRays = false; S.phase = 0; updatePhaseUI(); });
  $('#btnPause').addEventListener('click', () => { S.mode = 'step'; S.phase = 0; tickAcc = 0; updatePhaseUI(); });
  $('#btnStep').addEventListener('click', stepPhase);

  $('#lookInside').addEventListener('click', () => setPage('robot'));
  $$('#pageSeg button').forEach((b) => b.addEventListener('click', () => setPage(b.dataset.page)));
  $$('#modeSeg button').forEach((b) => b.addEventListener('click', () => { endTour(); setRbMode(b.dataset.mode); }));
  $('#rbReset').addEventListener('click', resetStudioView);
  $('#rbBack').addEventListener('click', () => setPage('sim'));
  $('#tourBtn').addEventListener('click', () => startTour());
  $('#tourNext').addEventListener('click', () => (S.tour >= TOUR.length - 1 ? endTour() : showTourStep(S.tour + 1)));
  $('#tourBack').addEventListener('click', () => showTourStep(Math.max(0, S.tour - 1)));
  $('#tourExit').addEventListener('click', endTour);

  bindWorldPointer();
  bindStudioPointer();
  bindClosePointer();
  pressed('#laserSeg', 'laser', 'on');
  pressed('#viewSeg', 'view', 'model');
  pressed('#speedSeg', 'speed', S.speed);
  pressed('#pageSeg', 'page', S.page);
  pressed('#modeSeg', 'mode', S.rbMode);
}

function selectTab(name) {
  $$('.tabs button').forEach((b) => b.setAttribute('aria-selected', String(b.dataset.tab === name)));
  $$('.tab').forEach((s) => { s.hidden = s.id !== 'tab-' + name; });
  syncThumbs(true);
}

// ---------------------------------------------------------------- step mode
function stepPhase() {
  if (S.mode !== 'step') { S.mode = 'step'; S.phase = 0; tickAcc = 0; }
  if (S.moveLeft > 0) return;
  switch (S.phase) {
    case 0: S.moveLeft = 0.4; S.weightView = false; S.allRays = false; break;
    case 1: if (phaseSense()) { S.allRays = true; S.phase = 2; } else { S.phase = 0; } break;
    case 2: phaseWeigh(); S.weightView = true; S.allRays = false; S.phase = 3; break;
    case 3: phaseResample(); S.weightView = false; finishCycle(); S.phase = 0; break;
  }
  updatePhaseUI();
}

function updatePhaseUI() {
  const t = T();
  $$('.ph').forEach((el) => {
    const i = +el.dataset.ph;
    el.classList.toggle('now', S.mode === 'step' && i === S.phase);
    el.classList.toggle('run', S.mode === 'run');
  });
  const d = [t.ph1_d, t.ph2_d, t.ph3_d, t.ph4_d];
  $('#phDesc').textContent = S.mode === 'run' ? t.ph_run : d[S.phase];
  $('#btnRun').disabled = S.mode === 'run';
  $('#btnPause').disabled = S.mode === 'step';
}

// ---------------------------------------------------------------- pointer: world
function bindWorldPointer() {
  const el = $('#wview');
  let drag = null;
  el.addEventListener('pointermove', (e) => {
    if (S.page !== 'sim') return;
    const p = stage.floorAt(e.clientX, e.clientY);
    if (drag) {
      if (!p) return;
      if (drag.type === 'robot') {
        const nx = clamp(p.x, 0.2, W - 0.2), ny = clamp(p.y, 0.2, H - 0.2);
        if (!world.collides(nx, ny, ROBOT_R)) { robot.x = nx; robot.y = ny; drag.moved = true; }
      } else {
        const it = world.item(drag.id);
        if (world.tryMove(drag.id, p.x - drag.ox, p.y - drag.oy, robot)) {
          stage.moveItem(it); stage.showFootprint(it); drag.moved = true;
        }
      }
      return;
    }
    if (!p || e.pointerType === 'touch') return;
    el.classList.remove('grab', 'move');
    if (Math.hypot(p.x - robot.x, p.y - robot.y) < ROBOT_R + 0.08) el.classList.add('grab');
    else {
      const it = itemAt(p);
      stage.showFootprint(it);
      if (it) el.classList.add('move');
    }
  });
  el.addEventListener('pointerdown', (e) => {
    if (S.page !== 'sim') return;
    const p = stage.floorAt(e.clientX, e.clientY);
    if (!p || S.kidnapAnim) return;
    if (Math.hypot(p.x - robot.x, p.y - robot.y) < ROBOT_R + 0.1) {
      el.setPointerCapture(e.pointerId);
      drag = { type: 'robot', moved: false, sx: robot.x, sy: robot.y };
      robot.lifted = true;
      stage.setCloseMode('lifted', 0);
      el.classList.add('grabbing');
      S.dragHintUntil = 0;
      return;
    }
    const it = itemAt(p);
    if (it && e.pointerType !== 'touch') {
      el.setPointerCapture(e.pointerId);
      drag = { type: 'item', id: it.id, ox: p.x - it.cx, oy: p.y - it.cy, moved: false };
      return;
    }
    S.inspectPt = p;   // otherwise: inspect the nearest guess
  });
  const end = () => {
    if (!drag) return;
    if (drag.type === 'robot') {
      el.classList.remove('grabbing');
      if (Math.hypot(robot.x - drag.sx, robot.y - drag.sy) > 0.3) landed();
      else robot.lifted = false;
      stage.setCloseMode('default', 0);
    } else if (drag.moved) {
      cov.rebuild(world);
      S.mapStale = true; $('#mapBanner').hidden = false; markInset();
      stage.showFootprint(null);
    }
    drag = null;
  };
  el.addEventListener('pointerup', end);
  el.addEventListener('pointercancel', end);
  el.addEventListener('pointerleave', () => { if (!drag) stage.showFootprint(null); });
  window.addEventListener('pointermove', (e) => {
    if (e.pointerType === 'touch') return;
    const nx = e.clientX / window.innerWidth - 0.5;
    stage.view.azT = stage.reduced ? 0 : nx * 0.06;
  }, { passive: true });
}

function itemAt(p) {
  for (const it of world.items) {
    if (!it.movable) continue;
    const f = footprint(it);
    if (p.x > f.x0 - 0.04 && p.x < f.x1 + 0.04 && p.y > f.y0 - 0.04 && p.y < f.y1 + 0.04) return it;
  }
  return null;
}

// ---------------------------------------------------------------- pointer: robot camera
let hoverPart = null;
function bindClosePointer() {
  const el = $('#closeup');
  let drag = null;
  el.addEventListener('pointerdown', (e) => {
    if (e.target.closest('button')) return;
    drag = { x: e.clientX, y: e.clientY, yaw: stage.close.userYaw, pitch: stage.close.userPitch };
    if (e.pointerType !== 'touch') el.setPointerCapture(e.pointerId);
    el.classList.add('dragging');
  });
  el.addEventListener('pointermove', (e) => {
    if (drag) {
      stage.close.userYaw = drag.yaw + (e.clientX - drag.x) * 0.012;
      stage.close.userPitch = clamp(drag.pitch + (e.clientY - drag.y) * 0.006, -0.4, 0.8);
      return;
    }
    if (e.pointerType !== 'touch') setHover(stage.pickPart(e.clientX, e.clientY));
  });
  const end = () => { drag = null; el.classList.remove('dragging'); };
  el.addEventListener('pointerup', end);
  el.addEventListener('pointercancel', end);
  el.addEventListener('pointerleave', () => { if (!drag) setHover(null); });
  el.addEventListener('dblclick', () => { stage.close.userYaw = 0; stage.close.userPitch = 0; });
}

function setHover(name) {
  if (name === hoverPart) return;
  hoverPart = name;
  const box = $('#cuPart');
  if (!name || !T()['p_' + name]) { box.hidden = true; return; }
  box.querySelector('b').textContent = T()['p_' + name];
  box.querySelector('span').textContent = T()['p_' + name + '_d'];
  box.hidden = false;
  box.style.animation = 'none'; void box.offsetWidth; box.style.animation = '';
}

// ---------------------------------------------------------------- pages
// "Simulation": the robot at work. "Robot up close": the same robot standing still in a photo studio.
function setPage(page) {
  if (S.page === page || S.switching) return;
  S.switching = true;
  pressed('#pageSeg', 'page', page);
  const w = $('#world');
  w.classList.add('switching');
  setTimeout(() => {
    S.page = page;
    $('#app').dataset.page = page;
    studio.active = page === 'robot';
    studio.invalidate();
    $('#wview').style.cursor = '';
    if (page === 'robot') {
      studio.view.userYaw = 0; studio.view.userPitch = 0;
      studio.touch();
      selectPart(S.part, false);
    } else {
      endTour();
      lastSweep = { x: robot.x, y: robot.y };
    }
    requestAnimationFrame(() => requestAnimationFrame(() => {
      w.classList.remove('switching');
      syncThumbs(false);
      S.switching = false;
      markInset();
    }));
  }, reduced ? 0 : 240);
}

const OUTSIDE = new Set(['lidar', 'cover', 'bumper', 'side', 'wheel', 'body']);
// which view shows a part best when it is picked from the whole robot
const SHOW_IN = { pcb: 'open', fan: 'open', bin: 'open', cover: 'open', battery: 'apart' };
const rbLabelEls = {};

function setRbMode(mode) {
  S.rbMode = mode;
  studio.setMode(mode);
  studio.touch();
  pressed('#modeSeg', 'mode', mode);
}

function resetStudioView() {
  studio.view.userYaw = 0;
  studio.view.userPitch = 0;
  studio.view.auto = 0;
  studio.focus(null);
  studio.touch();
}

function selectPart(id, focus = true) {
  const i = Math.max(0, PARTS.findIndex((p) => p.id === id));
  const info = PARTS[i];
  S.part = info.id;
  const t = T();
  $('#rbNum').textContent = String(i + 1);
  $('#rbTag').hidden = !info.nav;
  $('#rbName').textContent = t['p_' + info.id];
  $('#rbDesc').textContent = t['p_' + info.id + '_d'];
  $('#rbMore').textContent = t['p_' + info.id + '_more'] || '';
  const specs = $('#rbSpecs');
  specs.textContent = '';
  for (const sText of (t['p_' + info.id + '_spec'] || '').split('|').filter(Boolean)) {
    const li = document.createElement('li');
    li.textContent = sText;
    specs.appendChild(li);
  }
  $('#rbSim').textContent = t['p_' + info.id + '_sim'] || '';
  const card = $('#rbCard');
  card.classList.remove('swap'); void card.offsetWidth; card.classList.add('swap');
  $$('.rb-item').forEach((b) => b.classList.toggle('on', b.dataset.part === info.id));
  $$('.rb-label').forEach((b) => b.classList.toggle('on', b.dataset.part === info.id));
  if (focus) {
    const want = SHOW_IN[info.id];
    if (want && S.rbMode === 'whole') setRbMode(want);
    studio.setDemo(info.id);
  }
  studio.focus(focus ? info.id : null);
  studio.touch();
}

function buildRobotPage() {
  const t = T();
  const list = $('#rbList'), labels = $('#rbLabels');
  list.textContent = ''; labels.textContent = '';
  PARTS.forEach((p, i) => {
    const wire = (el) => {
      el.addEventListener('pointerenter', () => studio.setHover(p.id));
      el.addEventListener('pointerleave', () => studio.setHover(null));
      el.addEventListener('click', () => { endTour(); selectPart(p.id); });
    };
    const li = document.createElement('li');
    const b = document.createElement('button');
    b.type = 'button';
    b.className = 'rb-item' + (p.nav ? ' nav' : '');
    b.dataset.part = p.id;
    const n = document.createElement('span'); n.className = 'n'; n.textContent = String(i + 1);
    const name = document.createElement('span'); name.className = 't'; name.textContent = t['p_' + p.id];
    const k = document.createElement('span'); k.className = 'k'; k.textContent = p.nav ? t.rb_nav_short : '';
    b.append(n, name, k);
    wire(b);
    li.appendChild(b);
    list.appendChild(li);
    const l = document.createElement('button');
    l.type = 'button';
    l.className = 'rb-label' + (p.nav ? ' nav' : '');
    l.dataset.part = p.id;
    const em = document.createElement('em'); em.textContent = String(i + 1);
    l.append(em, document.createTextNode(t['p_' + p.id]));
    l.style.opacity = '0';
    wire(l);
    labels.appendChild(l);
    rbLabelEls[p.id] = l;
  });
  selectPart(S.part, false);
}

// ---------------------------------------------------------------- the guided tour
const TOUR = [
  { mode: 'whole', part: null },
  { mode: 'open', part: 'cover' },
  { mode: 'whole', part: 'lidar', demo: 'lidar' },
  { mode: 'open', part: 'pcb', demo: 'pcb' },
  { mode: 'cut', part: 'wheel', demo: 'wheel' },
  { mode: 'whole', part: 'bumper', demo: 'bumper' },
  { mode: 'cut', part: null },
  { mode: 'apart', part: null },
];
function startTour() { showTourStep(0); }
function showTourStep(i) {
  S.tour = i;
  const st = TOUR[i], t = T();
  $('#tour').hidden = false;
  setRbMode(st.mode);
  if (st.part) selectPart(st.part, true);
  else { studio.focus(null); studio.setDemo(null); }
  if (st.demo) studio.setDemo(st.demo);
  $('#tourStep').textContent = fmt(t.tour_step, { i: i + 1, n: TOUR.length });
  $('#tourTitle').textContent = t['tour' + (i + 1) + '_t'];
  $('#tourText').textContent = t['tour' + (i + 1) + '_d'];
  $('#tourBar').style.width = ((i + 1) / TOUR.length) * 100 + '%';
  $('#tourBack').disabled = i === 0;
  $('#tourNext').textContent = i === TOUR.length - 1 ? t.tour_finish : t.tour_next;
  const body = $('#tourBody');
  body.classList.remove('swap'); void body.offsetWidth; body.classList.add('swap');
  markInset();
}
function endTour() {
  if (S.tour < 0) return;
  S.tour = -1;
  $('#tour').hidden = true;
  markInset();
}

// Labels sit beside their parts; overlapping ones are nudged apart and all stay inside the view.
function placeStudioLabels() {
  const e = studio.robot.state.explode;
  const rw = stage.sizeW.w, rh = stage.sizeW.h;
  const items = [];
  for (const p of studio.labelPositions()) {
    const el = rbLabelEls[p.id];
    if (!el) continue;
    const inView = p.x > -10 && p.y > -10 && p.x < rw + 10 && p.y < rh + 10;
    const narrow = rw < 560 && p.id !== S.part;
    const inner = !OUTSIDE.has(p.id) && e < 0.55 && S.rbMode !== 'open' && S.rbMode !== 'cut';
    const show = p.ok && inView && !narrow && (!inner || p.id === S.part);
    el.style.opacity = show ? '1' : '0';
    el.style.pointerEvents = show ? 'auto' : 'none';
    if (!el._w && el.offsetWidth) { el._w = el.offsetWidth; el._h = el.offsetHeight; }
    items.push({ el, x: p.x, y: p.y, w: el._w || 120, h: el._h || 30, show });
  }
  const vis = items.filter((it) => it.show);
  for (let pass = 0; pass < 4; pass++) {
    for (let i = 0; i < vis.length; i++)
      for (let j = i + 1; j < vis.length; j++) {
        const a = vis[i], b = vis[j];
        const ox = (a.w + b.w) / 2 + 6 - Math.abs(a.x - b.x);
        const oy = (a.h + b.h) / 2 + 4 - Math.abs(a.y - b.y);
        if (ox > 0 && oy > 0) {
          const d = oy / 2 + 0.5;
          if (a.y <= b.y) { a.y -= d; b.y += d; } else { a.y += d; b.y -= d; }
        }
      }
  }
  const ins = stage.inset;
  for (const it of items) {
    const x = clamp(it.x, ins.left + it.w / 2 + 6, rw - ins.right - it.w / 2 - 6);
    const y = clamp(it.y, ins.top + it.h / 2 + 6, rh - ins.bottom - it.h / 2 - 6);
    it.el.style.transform = `translate3d(${x.toFixed(1)}px, ${y.toFixed(1)}px, 0) translate(-50%, -50%)`;
  }
}

// a live readout next to the part that is doing its demo
function placeDemoChip() {
  const chip = $('#demoChip');
  const d = studio.demo;
  const t = T();
  if (!d || S.page !== 'robot') { chip.hidden = true; return; }
  let text;
  switch (d) {
    case 'wheel': text = fmt(t.demo_wheel, { n: Math.round(studio.ticks).toLocaleString('en-US') }); break;
    case 'pcb': text = fmt(t.demo_pcb, { d: studio.heading.toFixed(1) }); break;
    case 'bumper': text = fmt(t.demo_bumper, { s: studio.pose.bump ? t.demo_pressed : t.demo_released }); break;
    default: text = t['demo_' + d];
  }
  if (!text) { chip.hidden = true; return; }
  const p = studio.screenOf(d);
  if (!p || !p.ok) { chip.hidden = true; return; }
  if (chip.hidden) chip.hidden = false;
  const span = chip.querySelector('span');
  if (span.textContent !== text) span.textContent = text;
  const x = clamp(p.x, stage.inset.left + 120, stage.sizeW.w - stage.inset.right - 120);
  const y = clamp(p.y - 60, stage.inset.top + 30, stage.sizeW.h - stage.inset.bottom - 30);
  chip.style.transform = `translate3d(${x.toFixed(1)}px, ${y.toFixed(1)}px, 0) translate(-50%, -50%)`;
}

function bindStudioPointer() {
  const el = $('#wview');
  let drag = null;
  el.addEventListener('pointerdown', (e) => {
    if (S.page !== 'robot' || e.target.closest('.rb-label')) return;
    drag = { x: e.clientX, y: e.clientY, yaw: studio.view.userYaw, pitch: studio.view.userPitch, moved: false };
    if (e.pointerType !== 'touch') el.setPointerCapture(e.pointerId);
    studio.touch();
  });
  el.addEventListener('pointermove', (e) => {
    if (S.page !== 'robot') return;
    if (drag) {
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      if (Math.abs(dx) + Math.abs(dy) > 4) drag.moved = true;
      studio.view.userYaw = drag.yaw + dx * 0.008;
      studio.view.userPitch = clamp(drag.pitch + dy * 0.005, -0.3, 0.9);
      studio.touch();
      el.style.cursor = 'grabbing';
      return;
    }
    if (e.pointerType === 'touch') return;
    const name = studio.pick(e.clientX, e.clientY, $('#glWorld').getBoundingClientRect());
    studio.setHover(name);
    el.style.cursor = name ? 'pointer' : 'grab';
  });
  el.addEventListener('pointerup', (e) => {
    if (S.page !== 'robot' || !drag) return;
    if (!drag.moved) {
      const name = studio.pick(e.clientX, e.clientY, $('#glWorld').getBoundingClientRect());
      if (name) { endTour(); selectPart(name); }
    }
    drag = null;
    el.style.cursor = 'grab';
  });
  el.addEventListener('pointercancel', () => { drag = null; });
  el.addEventListener('pointerleave', () => { if (S.page === 'robot' && !drag) studio.setHover(null); });
  el.addEventListener('dblclick', () => { if (S.page === 'robot') resetStudioView(); });
}

// ---------------------------------------------------------------- glass panels over the 3D
// The flat (or the robot) is framed in whatever part of the 3D view the floating panels leave free.
let insetDirty = true, insetAt = 0;
const markInset = () => { insetDirty = true; };
window.addEventListener('resize', markInset);
window.addEventListener('scroll', markInset, { passive: true });
function measureInset(now) {
  if (!insetDirty && now - insetAt < 1000) return;
  insetDirty = false; insetAt = now;
  const wr = $('#world').getBoundingClientRect();
  const ins = { left: 0, top: 0, right: 0, bottom: 0 };
  for (const el of $$('[data-occlude]')) {
    if (!el.offsetParent) continue;
    const r = el.getBoundingClientRect();
    const x0 = Math.max(r.left, wr.left), x1 = Math.min(r.right, wr.right);
    const y0 = Math.max(r.top, wr.top), y1 = Math.min(r.bottom, wr.bottom);
    if (x1 - x0 < 4 || y1 - y0 < 4) continue;
    // a card that asks to sit on the left (the tour) pushes the picture right instead of up
    if (el.dataset.occlude === 'left' && x1 - x0 < wr.width * 0.6) ins.left = Math.max(ins.left, x1 - wr.left);
    else if (y1 - y0 > wr.height * 0.5 && x0 > wr.left + wr.width * 0.45) ins.right = Math.max(ins.right, wr.right - x0);
    else if ((y0 + y1) / 2 < wr.top + wr.height / 2) ins.top = Math.max(ins.top, y1 - wr.top);
    else ins.bottom = Math.max(ins.bottom, wr.bottom - y0);
  }
  const m = 10;
  stage.inset = { left: ins.left + m, top: ins.top + m, right: ins.right + m, bottom: ins.bottom + m };
}

// ---------------------------------------------------------------- per-frame UI
const chart = $('#errChart');
const cctx = chart.getContext('2d');
function drawChart() {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = chart.clientWidth, h = chart.clientHeight;
  if (!w) return;
  if (chart.width !== Math.round(w * dpr)) { chart.width = Math.round(w * dpr); chart.height = Math.round(h * dpr); }
  cctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  cctx.clearRect(0, 0, w, h);
  const t1 = simTime, t0 = t1 - 90;
  let maxE = 0.3;
  for (const p of history) maxE = Math.max(maxE, p.e);
  maxE = Math.ceil(maxE * 10) / 10;
  const X = (t) => ((t - t0) / 90) * w;
  const Y = (e) => h - 14 - (e / maxE) * (h - 26);
  cctx.fillStyle = 'rgba(255,79,0,0.18)';
  for (let i = 0; i < history.length; i++) {
    if (!history[i].blind) continue;
    const a = history[i].t, b = i + 1 < history.length ? history[i + 1].t : t1;
    cctx.fillRect(X(a), 0, Math.max(1, X(b) - X(a)), h - 14);
  }
  cctx.fillStyle = 'rgba(60,60,67,0.08)';
  for (let k = 1; k <= 2; k++) cctx.fillRect(0, Math.round(Y((maxE * k) / 3)), w, 1);
  if (history.length > 1) {
    cctx.fillStyle = 'rgba(20,20,20,0.06)';
    cctx.beginPath();
    cctx.moveTo(X(history[0].t), Y(0));
    for (const p of history) cctx.lineTo(X(p.t), Y(p.e));
    cctx.lineTo(X(history[history.length - 1].t), Y(0));
    cctx.closePath();
    cctx.fill();
    cctx.strokeStyle = '#141414'; cctx.lineWidth = 2; cctx.lineJoin = 'round'; cctx.lineCap = 'round';
    cctx.beginPath();
    history.forEach((p, i) => (i ? cctx.lineTo(X(p.t), Y(p.e)) : cctx.moveTo(X(p.t), Y(p.e))));
    cctx.stroke();
    const lp = history[history.length - 1];
    cctx.fillStyle = '#ff4f00';
    cctx.beginPath(); cctx.arc(X(lp.t), Y(lp.e), 3.5, 0, Math.PI * 2); cctx.fill();
  }
  cctx.fillStyle = '#6e6e73';
  cctx.font = '600 11px Figtree, system-ui, sans-serif';
  cctx.fillText(Math.round(maxE * 100) + ' ' + T().cm, 8, 14);
  cctx.fillText('0', 8, h - 4);
}

let uiAcc = 0, chartAcc = 0;
const shown = { err: 0, spread: 0, clean: 0 };
function updateUI(dt) {
  uiAcc += dt; chartAcc += dt;
  const err = Math.hypot(robot.x - belief.x, robot.y - belief.y);
  const k = 1 - Math.exp(-dt * 7);
  shown.err += (err - shown.err) * k;
  shown.spread += (pf.spread - shown.spread) * k;
  shown.clean += (cov.pct - shown.clean) * k;
  if (uiAcc < 0.05) return;
  uiAcc = 0;
  const t = T();
  $('#roOff').textContent = (shown.err * 100).toFixed(shown.err < 0.1 ? 1 : 0);
  $('#roOffBox').classList.toggle('bad', err > 0.25);
  $('#roUnsure').textContent = (shown.spread * 100).toFixed(shown.spread < 0.1 ? 1 : 0);
  $('#roClean').textContent = shown.clean.toFixed(0);
  $('#roLaserText').textContent = S.lidarOn ? t.laser_on_tag : t.laser_off_tag;
  $('#roLaser').classList.toggle('off', !S.lidarOn);
  $('#roCleanBar').style.width = shown.clean.toFixed(1) + '%';
  $('#odoLine').textContent = fmt(t.odo_line, { a: robot.distWheels.toFixed(2), b: robot.distTrue.toFixed(2) });
  $('#stNeff').textContent = Math.round(pf.neff);
  $('#stInj').textContent = pf.injected;

  if (!S.lidarOn && S.blindT >= 3 && pf.spread > 0.2) completeTask(0);
  if (S.slipT >= 10) completeTask(1);
  const prog = [
    S.tasks[0] ? 1 : !S.lidarOn ? 0.5 * Math.min(1, S.blindT / 3) + 0.5 * Math.min(1, pf.spread / 0.2) : 0,
    S.tasks[1] ? 1 : Math.min(1, S.slipT / 10),
    S.tasks[2] ? 1 : S.search ? 0.25 + 0.75 * Math.min(1, S.search.ok) : 0,
  ];
  const next = S.tasks.indexOf(false);
  for (let i = 0; i < 3; i++) {
    const card = $('#task' + (i + 1));
    card.querySelector('.tprog i').style.width = (prog[i] * 100).toFixed(1) + '%';
    card.classList.toggle('next', i === next);
    card.querySelector('.tchip-t').textContent = S.tasks[i] ? t.done : i === next ? (i === 0 ? t.chip_start : t.chip_next) : '';
  }
  const res = $('#t3res');
  if (S.search) {
    if (S.search.found != null) res.textContent = fmt(t.t3_found, { s: S.search.found.toFixed(1) });
    else res.textContent = S.lidarOn ? t.t3_search : t.t3_search_blind;
  } else res.textContent = '';

  let status;
  if (robot.lifted) status = t.status_lifted;
  else if (performance.now() - lastBump < 600) status = t.status_bump;
  else if (stage.close.mode === 'wheels') status = fmt(t.status_enc, { l: robot.ticksL, r: robot.ticksR });
  else status = S.lidarOn ? t.status_spin : t.status_retracted;
  $('#cuStatus').textContent = status;

  const it = $('#inspectText');
  if (S.sel >= 0 && pf.kv && S.lidarOn) it.textContent = fmt(t.inspect_line, { hit: inspectInfo.hits, k: inspectInfo.k, top: inspectInfo.top });
  else if (!S.lidarOn && S.inspectPt) it.textContent = t.inspect_nolaser;
  else it.textContent = t.inspect_empty;

  if (chartAcc > 0.25) { chartAcc = 0; drawChart(); }
}

// ---------------------------------------------------------------- frame loop
const inspectPts = new Float32Array(360 * 3);
let inspectInfo = null;
const vis = { lidarOn: true, spin: 0, wheelL: 0, wheelR: 0, brush: 0, side: 0, bump: 0, lifted: false, xray: 0, explode: 0, hover: null };
let last = performance.now();
const bootAt = performance.now();
S.dragHintUntil = performance.now() + 14000;

// Adaptive quality: if frames take too long on a visible page, step the effects down.
let perfAcc = 0, perfN = 0;
function watchPerf(raw) {
  if (document.hidden || raw > 0.25 || performance.now() - bootAt < 4000) { perfAcc = 0; perfN = 0; return; }
  perfAcc += raw; perfN++;
  if (perfN >= 90) {
    if (perfAcc / perfN > 0.024 && stage.lowerQuality() === 'lowfx') document.body.classList.add('lowfx');
    perfAcc = 0; perfN = 0;
  }
}

function frame(now) {
  const raw = (now - last) / 1000;
  const rdt = Math.min(0.05, raw);
  last = now;
  watchPerf(raw);

  // simulation (paused while the robot is shown up close)
  const live = S.page === 'sim';
  if (live && S.kidnapAnim) {
    const k = S.kidnapAnim;
    k.t += rdt / 0.9;
    const u = clamp(k.t, 0, 1), e = u * u * (3 - 2 * u);
    robot.x = k.from.x + (k.to.x - k.from.x) * e;
    robot.y = k.from.y + (k.to.y - k.from.y) * e;
    robot.th = wrap(k.from.th + wrap(k.to.th - k.from.th) * e);
    if (k.t >= 1) { S.kidnapAnim = null; landed(); }
  }
  if (live && S.mode === 'run') {
    const simDt = rdt * S.speed;
    const steps = Math.ceil(simDt / (1 / 120));
    for (let i = 0; i < steps; i++) physicsStep(simDt / steps);
  } else if (live && S.moveLeft > 0) {
    const d = Math.min(S.moveLeft, rdt);
    S.moveLeft -= d;
    const steps = Math.ceil(d / (1 / 120));
    for (let i = 0; i < steps; i++) physicsStep(d / steps);
    if (S.moveLeft <= 0) { S.moveLeft = 0; phaseMove(); pf.computeEstimate(); S.phase = 1; updatePhaseUI(); }
  }

  if (!robot.lifted) {
    if (Math.hypot(robot.x - lastSweep.x, robot.y - lastSweep.y) > 0.01) {
      stage.queueSweep(lastSweep.x, lastSweep.y, robot.x, robot.y, CLEAN_R);
      lastSweep = { x: robot.x, y: robot.y };
    }
  } else lastSweep = { x: robot.x, y: robot.y };
  if (dust.dirty) { dust.dirty = false; stage.markDirty('texDust'); }
  if (trailDirty) { trailDirty = false; stage.updateTrail(trail.belief); }

  const moving = live && !robot.lifted && ctrl.state !== 'done';
  vis.lidarOn = S.lidarOn;
  if (S.lidarOn && live) vis.spin += rdt * Math.PI * 2 * 1.6;
  vis.wheelL = robot.angL; vis.wheelR = robot.angR;
  if (moving) { vis.brush += rdt * 26; vis.side += rdt * 11; }
  vis.bump = performance.now() - lastBump < 120 ? 1 : 0;
  vis.lifted = robot.lifted;
  vis.xray = 0;      // never see through the shell while it drives
  vis.explode = 0;   // opening it up happens on the "Robot up close" page only
  vis.hover = hoverPart;
  stage.updateRobot(robot, vis, rdt);

  S.sel = S.inspectPt ? pf.nearest(S.inspectPt.x, S.inspectPt.y) : -1;
  if (S.sel >= 0 && pf.kv && S.lidarOn) {
    inspectInfo = pf.fitOf(S.sel, scan, df, sigma(), inspectPts);
    stage.updateInspect(inspectPts, pf.kv, pf.x[S.sel], pf.y[S.sel]);
  } else stage.updateInspect(null, 0);

  stage.updateParticles(pf, S.sel, S.weightView);
  stage.updateGhosts(belief, robot.odo, S.show.ghost);
  const lidarTurret = { x: robot.x - 0.03 * Math.cos(robot.th), y: robot.y - 0.03 * Math.sin(robot.th) };
  stage.updateLidar(S.lidarOn, hits, scanFresh, lidarTurret.x, lidarTurret.y, robot.th, vis.spin, S.allRays, S.show.rays);
  scanFresh = false;

  measureInset(now);
  stage.view.focusT = S.mode === 'step' ? 1 : 0;
  stage.render(rdt, robot, { world: onScreen.world, close: live && onScreen.close });

  const hint = $('#dragHint');
  const showHint = live && now < S.dragHintUntil && !robot.lifted;
  hint.classList.toggle('hide', !showHint);
  if (showHint) {
    const p = stage.project(robot.x, 0.12, robot.y);
    hint.style.transform = `translate3d(${p.x.toFixed(1)}px, ${p.y.toFixed(1)}px, 0) translate(-50%, calc(-100% - 14px))`;
  }

  if (!live) { placeStudioLabels(); placeDemoChip(); }
  else $('#demoChip').hidden = true;

  if (live) updateUI(rdt);
  requestAnimationFrame(frame);
}

// ---------------------------------------------------------------- boot
initThumbs();
drawPlan();
bindControls();
selectTab('try');
applyText();
$('#nParticles').value = String(Math.round((Math.log(S.n / 20) / Math.log(150)) * 100));
refreshSliderLabels();
updatePhaseUI();
document.fonts && document.fonts.ready.then(() => { paintBase(canvases.base, roomLabels()); stage.markDirty('texBase'); drawChart(); syncThumbs(false); });
setTimeout(() => syncThumbs(false), 400);
// Developer hook for measuring performance: open the page with ?debug
if (new URLSearchParams(location.search).has('debug')) window.__rv = { stage, studio, S, pf, robot };
requestAnimationFrame(frame);
