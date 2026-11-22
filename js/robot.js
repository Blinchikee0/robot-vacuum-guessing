// The REAL robot: differential drive with slipping wheels, encoders, a gyro and a lidar.
// The robot itself never reads x, y, th. It only gets encoder ticks, gyro and lidar ranges.

import { gauss, wrap, clamp } from './util.js';
import { ROBOT_R, RUG } from './world.js';

export const BASE = 0.235;     // distance between the drive wheels
export const WHEEL_R = 0.036;  // nominal wheel radius
export const TICKS = 1000;     // encoder ticks per wheel turn
export const BEAMS = 360;      // lidar returns per turn
export const LIDAR_MAX = 8;    // metres

export class Robot {
  constructor(world) {
    this.world = world;
    this.reset({ x: 0, y: 0, th: 0 });
  }

  reset(p) {
    this.x = p.x; this.y = p.y; this.th = p.th;           // truth
    this.odo = { x: p.x, y: p.y, th: p.th };              // dead reckoning
    this.angL = 0; this.angR = 0;                         // wheel angles (rad), what encoders count
    this.distTrue = 0; this.distWheels = 0;
    this.sc = 0; this.sd = 0;                             // slow random slip processes
    this.lifted = false; this.bumps = 0;
    this.calL = 0.005; this.calR = -0.004;                // real wheels are never exactly nominal
    this.gyroBias = (Math.random() * 2 - 1) * 0.0005;     // rad/s
    this.slipNow = 0;
  }

  get ticksL() { return Math.round(this.angL / (2 * Math.PI) * TICKS); }
  get ticksR() { return Math.round(this.angR / (2 * Math.PI) * TICKS); }

  onRug() { return this.x > RUG.x0 && this.x < RUG.x1 && this.y > RUG.y0 && this.y < RUG.y1; }

  // v: forward speed command, w: turn rate command. env: { slip, gyro }
  step(dt, v, w, env) {
    const vl = v - (w * BASE) / 2, vr = v + (w * BASE) / 2;
    const aL = (vl * dt) / WHEEL_R, aR = (vr * dt) / WHEEL_R;   // wheel rotation the motors produce
    this.angL += aL; this.angR += aR;

    // Slip: a slowly wandering fraction of each wheel's motion that never reaches the floor.
    const sq = Math.sqrt(dt);
    this.sc += (-this.sc * dt) / 1.2 + 0.9 * sq * gauss();
    this.sd += (-this.sd * dt) / 2.0 + 0.7 * sq * gauss();
    const base = env.slip * (this.onRug() ? 1.7 : 1) * (1 + 0.8 * Math.min(1, Math.abs(w) / 1.5));
    const common = clamp(1 + 0.5 * this.sc, 0.2, 2);
    const sL = clamp(base * common * (1 + 0.4 * this.sd), 0, 0.9);
    const sR = clamp(base * common * (1 - 0.4 * this.sd), 0, 0.9);
    this.slipNow = (sL + sR) / 2;

    const dL = aL * WHEEL_R * (1 + this.calL) * (1 - sL);
    const dR = aR * WHEEL_R * (1 + this.calR) * (1 - sR);
    const ds = (dL + dR) / 2, dth = (dR - dL) / BASE;
    const mx = ds * Math.cos(this.th + dth / 2), my = ds * Math.sin(this.th + dth / 2);

    let bumped = false;
    if (this.world.collides(this.x + mx, this.y + my, ROBOT_R)) {
      bumped = Math.abs(ds) > 1e-6;
      // scrape along the obstacle a little, like a real bumper does
      if (!this.world.collides(this.x + mx * 0.3, this.y, ROBOT_R)) this.x += mx * 0.3;
      else if (!this.world.collides(this.x, this.y + my * 0.3, ROBOT_R)) this.y += my * 0.3;
    } else {
      this.x += mx; this.y += my;
      this.distTrue += Math.abs(ds);
    }
    this.th = wrap(this.th + dth);
    if (bumped) this.bumps++;

    // Odometry: what the robot THINKS happened, from encoder counts and nominal geometry.
    const oL = aL * WHEEL_R, oR = aR * WHEEL_R;
    const ods = (oL + oR) / 2;
    const odth = env.gyro
      ? dth + this.gyroBias * dt + 0.003 * sq * gauss()   // a gyro feels the real rotation
      : (oR - oL) / BASE;                                  // without one, heading also comes from wheels
    this.odo.x += ods * Math.cos(this.odo.th + odth / 2);
    this.odo.y += ods * Math.sin(this.odo.th + odth / 2);
    this.odo.th = wrap(this.odo.th + odth);
    this.distWheels += Math.abs(ods);
    return { bumped, ods, odth };
  }

  // One full lidar turn from the TRUE pose. Ranges are relative to the robot heading.
  // out: Float32Array(360) ranges (0 = no return). hits: Float32Array(720) world points for drawing.
  scan(out, noise, hits) {
    const deg = Math.PI / 180;
    for (let i = 0; i < BEAMS; i++) {
      const a = this.th + i * deg;
      const c = Math.cos(a), s = Math.sin(a);
      const r = this.world.raycast(this.x, this.y, c, s, LIDAR_MAX);
      if (r >= LIDAR_MAX || Math.random() < 0.012) {
        out[i] = 0; hits[2 * i] = NaN; hits[2 * i + 1] = NaN;
        continue;
      }
      const m = Math.max(0.06, r + gauss() * (noise + 0.004 * r));
      out[i] = m;
      hits[2 * i] = this.x + m * c;
      hits[2 * i + 1] = this.y + m * s;
    }
  }
}
