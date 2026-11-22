// How well does the laser keep the robot localized as wheel slip rises?
// Run: node tests/slip.test.mjs
import { World, DistField, DOCK } from '../js/world.js';
import { planRoute } from '../js/planner.js';
import { Robot } from '../js/robot.js';
import { Controller } from '../js/controller.js';
import { ParticleFilter } from '../js/filter.js';
import { wrap } from '../js/util.js';
import { motionAlphas } from '../js/motion-noise.js';

const rows = [];
for (const slip of [0.0, 0.06, 0.15, 0.3]) for (const gyro of [true, false]) {
  const errs = [];
  for (let rep = 0; rep < 3; rep++) {
    const world = new World(); const df = new DistField(world.clone()); const route = planRoute(df, DOCK);
    const robot = new Robot(world); robot.reset(DOCK); const ctrl = new Controller(); ctrl.setRoute(route);
    const pf = new ParticleFilter(4000); pf.init(DOCK, 1000);
    const scan = new Float32Array(360), hits = new Float32Array(720);
    let belief = { ...DOCK }, mark = { ...robot.odo }; const dt = 1 / 120; let acc = 0, t = 0;
    while (t < 150) {
      t += dt;
      const c = ctrl.update(dt, belief); const r = robot.step(dt, c.v, c.w, { slip, gyro }); if (r.bumped) ctrl.bump();
      belief.x += r.ods * Math.cos(belief.th + r.odth / 2); belief.y += r.ods * Math.sin(belief.th + r.odth / 2); belief.th = wrap(belief.th + r.odth);
      acc += dt;
      if (acc >= 0.2) {
        acc -= 0.2;
        pf.move(mark, robot.odo, motionAlphas(gyro, 1)); mark = { ...robot.odo };
        robot.scan(scan, 0.02, hits);
        pf.weigh(scan, df, { beams: 36, sigma: 0.056, temper: 5 / 36 });
        if (pf.neff < pf.n * 0.6) pf.resample(df);
        pf.computeEstimate(); belief = { ...pf.est };
        errs.push(Math.hypot(robot.x - belief.x, robot.y - belief.y));
      }
    }
  }
  errs.sort((a, b) => a - b);
  rows.push({ slip, gyro, meanCm: (errs.reduce((a, b) => a + b, 0) / errs.length * 100).toFixed(1), p95Cm: (errs[Math.floor(errs.length * 0.95)] * 100).toFixed(1), maxCm: (errs[errs.length - 1] * 100).toFixed(0) });
}
console.table(rows);
