// Drives the route using the robot's BELIEVED pose. If the belief is wrong, the real
// robot goes to the wrong place: that is how bad localization turns into bad cleaning.

import { wrap, clamp } from './util.js';

const VMAX = 0.3;   // m/s
const WMAX = 1.6;   // rad/s

export class Controller {
  constructor() { this.setRoute([]); }

  setRoute(route, i = 0) {
    this.route = route;
    this.i = i;
    this.state = route.length ? 'drive' : 'done';
    this.t = 0; this.bumpsHere = 0; this.wpT = 0; this.turnDir = 1;
  }

  get progress() { return this.route.length ? this.i / this.route.length : 1; }

  bump() {
    if (this.state === 'drive' || this.state === 'turn') {
      this.state = 'backup'; this.t = 0; this.bumpsHere++;
      this.turnDir = Math.random() < 0.5 ? -1 : 1;
    }
  }

  advance() { this.i++; this.bumpsHere = 0; this.wpT = 0; }

  update(dt, b) {
    switch (this.state) {
      case 'backup':
        this.t += dt;
        if (this.t > 0.45) { this.state = 'turn'; this.t = 0; }
        return { v: -0.12, w: 0 };
      case 'turn':
        this.t += dt;
        if (this.t > 0.4) {
          this.state = 'drive'; this.t = 0;
          if (this.bumpsHere >= 3) this.advance();
        }
        return { v: 0, w: this.turnDir * 1.4 };
      case 'drive': {
        if (this.i >= this.route.length) { this.state = 'done'; return { v: 0, w: 0 }; }
        const p = this.route[this.i];
        const dx = p.x - b.x, dy = p.y - b.y;
        const dist = Math.hypot(dx, dy);
        this.wpT += dt;
        if (dist < 0.07 || this.wpT > 20) { this.advance(); return { v: 0, w: 0 }; }
        const a = wrap(Math.atan2(dy, dx) - b.th);
        if (Math.abs(a) > 0.45) return { v: 0, w: clamp(3 * a, -WMAX, WMAX) };
        const v = VMAX * clamp(dist / 0.3, 0.3, 1) * (1 - (Math.abs(a) / 0.45) * 0.5);
        return { v, w: clamp(3.2 * a, -WMAX, WMAX) };
      }
      default:
        return { v: 0, w: 0 };
    }
  }
}
