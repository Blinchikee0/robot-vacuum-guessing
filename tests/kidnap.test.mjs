import { World, DistField, DOCK } from '../js/world.js';
import { planRoute } from '../js/planner.js';
import { Robot } from '../js/robot.js';
import { Controller } from '../js/controller.js';
import { ParticleFilter } from '../js/filter.js';
import { wrap } from '../js/util.js';
import { motionAlphas } from '../js/motion-noise.js';
const spots = [[7.5,4.8,2.0],[6.5,2.8,0.5],[2.5,4.9,-1],[4.5,2.0,3],[1.0,2.9,1.2],[8.0,0.6,2.5]];
const rows=[];
for (const n of [300,600,1200]) for (const [kx,ky,kt] of spots) {
  const world = new World(); const df = new DistField(world.clone()); const route = planRoute(df, DOCK);
  const robot = new Robot(world); robot.reset(DOCK); const ctrl = new Controller(); ctrl.setRoute(route);
  const pf = new ParticleFilter(4000); pf.init(DOCK, n);
  const scan = new Float32Array(360), hits = new Float32Array(720);
  let belief = {...DOCK}, mark = {...robot.odo}; const dt=1/120; let acc=0,t=0, kid=false, found=-1, ok=0, wa=[];
  while (t < 90 && found < 0) { t+=dt;
    if (!kid && t>30) { kid=true; robot.x=kx; robot.y=ky; robot.th=kt; }
    const c = ctrl.update(dt, belief); const r = robot.step(dt,c.v,c.w,{slip:0.06,gyro:true}); if (r.bumped) ctrl.bump();
    belief.x += r.ods*Math.cos(belief.th+r.odth/2); belief.y += r.ods*Math.sin(belief.th+r.odth/2); belief.th = wrap(belief.th+r.odth);
    acc+=dt; if (acc>=0.2){acc-=0.2; pf.move(mark, robot.odo,motionAlphas(true, 1)); mark={...robot.odo};
      robot.scan(scan,0.02,hits); pf.weigh(scan,df,{beams:36,sigma:0.05,temper:5/36}); if(!kid) wa.push(pf.wAvg); if (pf.neff<pf.n*0.6) pf.resample(df); pf.computeEstimate(); belief={...pf.est};
      if (kid){ const e=Math.hypot(robot.x-belief.x, robot.y-belief.y); if(e<0.12){ok+=0.2; if(ok>=1) found=t-30-1;} else ok=0; } }
  }
  rows.push({n, spot:`${kx},${ky}`, found: found<0?'never':found.toFixed(1), wAvgLocalized:(wa.reduce((a,b)=>a+b,0)/wa.length).toFixed(2)});
}
console.table(rows);
