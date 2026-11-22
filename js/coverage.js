// Tracks which parts of the REAL floor were actually cleaned.

import { W, H, ROBOT_R, DOCK, DistField } from './world.js';

const RES = 0.05;
const CLEAN_R = 0.16;

export class Coverage {
  constructor(world) {
    this.nx = Math.round(W / RES); this.ny = Math.round(H / RES);
    this.done = new Uint8Array(this.nx * this.ny);
    this.mask = new Uint8Array(this.nx * this.ny);
    this.total = 1; this.count = 0;
    this.rebuild(world);
  }

  // Cleanable = floor the robot body could physically reach with its brushes.
  rebuild(world) {
    const { nx, ny } = this;
    const df = new DistField(world, RES, 1);
    const d = new Float32Array(nx * ny);
    for (let j = 0; j < ny; j++)
      for (let i = 0; i < nx; i++) d[j * nx + i] = df.at((i + 0.5) * RES, (j + 0.5) * RES);
    const reach = new Uint8Array(nx * ny);
    const si = Math.floor(DOCK.x / RES), sj = Math.floor(DOCK.y / RES);
    const stack = [sj * nx + si];
    while (stack.length) {
      const c = stack.pop();
      if (reach[c] || d[c] < ROBOT_R + 0.005) continue;
      reach[c] = 1;
      const i = c % nx, j = (c / nx) | 0;
      if (i > 0) stack.push(c - 1);
      if (i < nx - 1) stack.push(c + 1);
      if (j > 0) stack.push(c - nx);
      if (j < ny - 1) stack.push(c + nx);
    }
    this.mask.fill(0);
    const r = 3;
    for (let c = 0; c < nx * ny; c++) {
      if (!reach[c]) continue;
      const i = c % nx, j = (c / nx) | 0;
      for (let dj = -r; dj <= r; dj++)
        for (let di = -r; di <= r; di++) {
          if (di * di + dj * dj > r * r) continue;
          const i2 = i + di, j2 = j + dj;
          if (i2 < 0 || j2 < 0 || i2 >= nx || j2 >= ny) continue;
          const n = j2 * nx + i2;
          if (d[n] >= 0.03) this.mask[n] = 1;
        }
    }
    let total = 0, count = 0;
    for (let c = 0; c < nx * ny; c++) if (this.mask[c]) { total++; if (this.done[c]) count++; }
    this.total = Math.max(1, total); this.count = count;
  }

  mark(x, y) {
    const { nx, ny } = this;
    const i0 = Math.max(0, Math.floor((x - CLEAN_R) / RES)), i1 = Math.min(nx - 1, Math.floor((x + CLEAN_R) / RES));
    const j0 = Math.max(0, Math.floor((y - CLEAN_R) / RES)), j1 = Math.min(ny - 1, Math.floor((y + CLEAN_R) / RES));
    const r2 = CLEAN_R * CLEAN_R;
    for (let j = j0; j <= j1; j++)
      for (let i = i0; i <= i1; i++) {
        const cx = (i + 0.5) * RES - x, cy = (j + 0.5) * RES - y;
        if (cx * cx + cy * cy > r2) continue;
        const c = j * nx + i;
        if (!this.done[c]) { this.done[c] = 1; if (this.mask[c]) this.count++; }
      }
  }

  reset() { this.done.fill(0); this.count = 0; }

  get pct() { return Math.min(100, (this.count / this.total) * 100); }
}

export { CLEAN_R };
