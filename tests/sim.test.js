// Headless check of the simulation core: run with `node tests/sim.test.js`.
// Prints how well the robot localizes with and without the laser.

import { World, DistField, DOCK } from '../js/world.js';
import { planRoute } from '../js/planner.js';
import { Robot } from '../js/robot.js';
import { Controller } from '../js/controller.js';
import { ParticleFilter } from '../js/filter.js';
import { Coverage } from '../js/coverage.js';
import { wrap } from '../js/util.js';
import { motionAlphas } from '../js/motion-noise.js';

function run({ lidar = true, slip = 0.06, gyro = true, seconds = 240, n = 600, kidnapAt = -1 }) {
  const world = new World();
  let t0 = performance.now();
  const df = new DistField(world.clone());
  const tDf = performance.now() - t0;
  t0 = performance.now();
  const route = planRoute(df, DOCK);
  const tPlan = performance.now() - t0;
  const robot = new Robot(world); robot.reset(DOCK);
  const ctrl = new Controller(); ctrl.setRoute(route);
  const pf = new ParticleFilter(4000); pf.init(DOCK, n);
  const cov = new Coverage(world);
  const scan = new Float32Array(360), hits = new Float32Array(720);
  let belief = { ...DOCK }, mark = { ...robot.odo };
  const dt = 1 / 120; let acc = 0, t = 0;
  const errs = []; let filterMs = 0, ticks = 0, kidnapped = false, foundAt = -1, okT = 0;
  const env = { slip, gyro };
  const alphas = motionAlphas(gyro, 1);
  while (t < seconds) {
    t += dt;
    if (kidnapAt > 0 && !kidnapped && t > kidnapAt) {
      kidnapped = true;
      robot.x = 7.5; robot.y = 4.8; robot.th = 2.0;
    }
    const cmd = ctrl.update(dt, belief);
    const r = robot.step(dt, cmd.v, cmd.w, env);
    if (r.bumped) ctrl.bump();
    belief.x += r.ods * Math.cos(belief.th + r.odth / 2);
    belief.y += r.ods * Math.sin(belief.th + r.odth / 2);
    belief.th = wrap(belief.th + r.odth);
    cov.mark(robot.x, robot.y);
    acc += dt;
    if (acc >= 0.2) {
      acc -= 0.2; ticks++;
      const a = performance.now();
      pf.move(mark, robot.odo, alphas); mark = { ...robot.odo };
      if (lidar) {
        robot.scan(scan, 0.02, hits);
        pf.weigh(scan, df, { beams: 36, sigma: 0.05, temper: 5 / 36 });
        if (pf.neff < pf.n * 0.6) pf.resample(df);
      }
      pf.computeEstimate();
      belief = { ...pf.est };
      filterMs += performance.now() - a;
      const e = Math.hypot(robot.x - belief.x, robot.y - belief.y);
      errs.push(e);
      if (kidnapped && foundAt < 0) { if (e < 0.12) { okT += 0.2; if (okT >= 1) foundAt = t - kidnapAt - 1; } else okT = 0; }
    }
    if (ctrl.state === 'done') break;
  }
  const mean = errs.reduce((a, b) => a + b, 0) / errs.length;
  const max = Math.max(...errs);
  return {
    lidar, slip, gyro, simSeconds: t.toFixed(0), routePts: route.length,
    tDf: tDf.toFixed(0) + 'ms', tPlan: tPlan.toFixed(0) + 'ms',
    meanErrCm: (mean * 100).toFixed(1), maxErrCm: (max * 100).toFixed(1),
    coverage: cov.pct.toFixed(1) + '%', bumps: robot.bumps,
    msPerTick: (filterMs / ticks).toFixed(2), state: ctrl.state,
    wheels: robot.distWheels.toFixed(1), real: robot.distTrue.toFixed(1),
    foundAfter: kidnapAt > 0 ? (foundAt < 0 ? 'never' : foundAt.toFixed(1) + 's') : '-',
  };
}

console.table([
  run({ lidar: true, seconds: 900 }),
  run({ lidar: false, seconds: 120 }),
  run({ lidar: false, gyro: false, seconds: 120 }),
  run({ lidar: true, slip: 0.25, seconds: 120 }),
  run({ lidar: true, seconds: 120, kidnapAt: 40 }),
  run({ lidar: true, seconds: 120, kidnapAt: 40, n: 1500 }),
]);
