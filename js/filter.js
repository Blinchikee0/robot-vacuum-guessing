// Monte Carlo localization: the robot's position is a cloud of guesses (particles).
// One cycle = MOVE every guess by odometry + noise, SENSE with the lidar,
// WEIGH each guess by how well the scan fits the map from there, RESAMPLE.

import { gauss, wrap, clamp } from './util.js';

const Z_HIT = 0.9;    // a beam that lands near a wall in the map
const Z_RAND = 0.03;  // floor for beams that do not fit (moved furniture, noise)

export class ParticleFilter {
  constructor(cap = 4000) {
    this.cap = cap; this.n = 0;
    this.x = new Float32Array(cap); this.y = new Float32Array(cap); this.th = new Float32Array(cap);
    this.w = new Float64Array(cap); this.lp = new Float64Array(cap);
    this.bx = new Float32Array(cap); this.by = new Float32Array(cap); this.bth = new Float32Array(cap);
    this.blp = new Float64Array(cap);
    this.ca = new Float32Array(360); this.sa = new Float32Array(360); this.ra = new Float32Array(360);
    this.beamIdx = new Int32Array(360); this.kv = 0;
    this.wSlow = 0; this.wFast = 0; this.wAvg = 0;
    this.injected = 0; this.neff = 0;
    this.est = { x: 0, y: 0, th: 0 }; this.spread = 0;
  }

  init(p, n, s = 0.03) {
    this.n = n;
    for (let i = 0; i < n; i++) {
      this.x[i] = p.x + gauss() * s; this.y[i] = p.y + gauss() * s;
      this.th[i] = wrap(p.th + gauss() * 0.03);
      this.w[i] = 1 / n; this.lp[i] = 0;
    }
    this.wSlow = this.wFast = 0; this.wGood = 0; this.neff = n; this.injected = 0;
    this.computeEstimate();
  }

  resize(n) { if (n !== this.n) this.systematic(n, 0, null); }

  // Low-variance (systematic) resampling, optionally replacing some with random guesses.
  systematic(m, inject, df) {
    const n = this.n, w = this.w;
    let total = 0;
    for (let i = 0; i < n; i++) total += w[i];
    const step = total / m;
    const u = Math.random() * step;
    let c = w[0], i = 0, injected = 0;
    for (let k = 0; k < m; k++) {
      const U = u + k * step;
      while (U > c && i < n - 1) { i++; c += w[i]; }
      if (inject > 0 && df && Math.random() < inject) {
        const p = df.randomFree();
        this.bx[k] = p.x; this.by[k] = p.y; this.bth[k] = Math.random() * 2 * Math.PI - Math.PI;
        this.blp[k] = -Infinity;
        injected++;
      } else {
        this.bx[k] = this.x[i]; this.by[k] = this.y[i]; this.bth[k] = this.th[i]; this.blp[k] = this.lp[i];
      }
    }
    [this.x, this.bx] = [this.bx, this.x];
    [this.y, this.by] = [this.by, this.y];
    [this.th, this.bth] = [this.bth, this.th];
    [this.lp, this.blp] = [this.blp, this.lp];
    this.n = m;
    for (let k = 0; k < m; k++) this.w[k] = 1 / m;
    this.injected = injected;
    this.neff = m;
  }

  // MOVE: odometry motion model (rotate, translate, rotate) with noise alphas a = [a1..a4].
  move(o0, o1, a) {
    const dx = o1.x - o0.x, dy = o1.y - o0.y;
    let trans = Math.hypot(dx, dy);
    const dth = wrap(o1.th - o0.th);
    if (trans < 1e-5 && Math.abs(dth) < 1e-5) return;
    let rot1 = trans < 0.003 ? 0 : wrap(Math.atan2(dy, dx) - o0.th);
    if (Math.abs(rot1) > Math.PI / 2) { rot1 = wrap(rot1 + Math.PI); trans = -trans; }
    const rot2 = wrap(dth - rot1);
    const [a1, a2, a3, a4] = a;
    const t2 = trans * trans;
    const s1 = Math.sqrt(a1 * rot1 * rot1 + a2 * t2);
    const st = Math.sqrt(a3 * t2 + a4 * (rot1 * rot1 + rot2 * rot2));
    const s2 = Math.sqrt(a1 * rot2 * rot2 + a2 * t2);
    const { x, y, th } = this;
    for (let i = 0; i < this.n; i++) {
      const h = th[i] + rot1 + gauss() * s1;
      const t = trans + gauss() * st;
      x[i] += t * Math.cos(h);
      y[i] += t * Math.sin(h);
      th[i] = wrap(h + rot2 + gauss() * s2);
    }
  }

  // WEIGH: likelihood-field measurement model.
  // For each guess, project lidar endpoints from it and look up distance to the nearest wall in the map.
  weigh(scan, df, { beams, sigma, temper }) {
    const K = Math.min(beams, 360);
    const stepA = 360 / K;
    const off = Math.random() * stepA;
    let kv = 0;
    for (let k = 0; k < K; k++) {
      const idx = Math.floor(off + k * stepA) % 360;
      const r = scan[idx];
      if (r <= 0.05) continue;
      const a = (idx * Math.PI) / 180;
      this.beamIdx[kv] = idx; this.ca[kv] = Math.cos(a); this.sa[kv] = Math.sin(a); this.ra[kv] = r;
      kv++;
    }
    this.kv = kv;
    if (kv < 3) return;

    const inv = 1 / (2 * sigma * sigma);
    const { x, y, th, w, lp, ca, sa, ra } = this;
    const D = df.d, nxg = df.nx, nyg = df.ny, gx0 = df.x0, gy0 = df.y0, ir = 1 / df.res;
    let maxLp = -Infinity;
    for (let i = 0; i < this.n; i++) {
      const c = Math.cos(th[i]), s = Math.sin(th[i]);
      const px = x[i], py = y[i];
      let sum = 0;
      for (let k = 0; k < kv; k++) {
        const ex = px + ra[k] * (c * ca[k] - s * sa[k]);
        const ey = py + ra[k] * (s * ca[k] + c * sa[k]);
        let d = 1;
        const gi = ((ex - gx0) * ir) | 0, gj = ((ey - gy0) * ir) | 0;
        if (ex >= gx0 && ey >= gy0 && gi < nxg && gj < nyg) d = D[gj * nxg + gi];
        sum += Math.log(Z_HIT * Math.exp(-d * d * inv) + Z_RAND);
      }
      lp[i] = sum;
      if (sum > maxLp) maxLp = sum;
    }

    // How well does the cloud explain the scan on average? Used to detect "I am lost".
    let tot = 0;
    for (let i = 0; i < this.n; i++) tot += w[i];
    let wavg = 0;
    for (let i = 0; i < this.n; i++) wavg += (w[i] / tot) * Math.exp(lp[i] / kv);
    this.wAvg = wavg;
    if (!this.wSlow) { this.wSlow = wavg; this.wFast = wavg; }
    else { this.wSlow += 0.015 * (wavg - this.wSlow); this.wFast += 0.25 * (wavg - this.wFast); }
    // wGood remembers how well the scan fits when the robot is NOT lost. It forgets very slowly,
    // so a kidnapped robot keeps adding random guesses until one of them fits again.
    if (!this.wGood || this.wFast > this.wGood) this.wGood = this.wFast;
    else this.wGood += 0.002 * (this.wFast - this.wGood);

    let sw = 0;
    for (let i = 0; i < this.n; i++) { w[i] = (w[i] / tot) * Math.exp(temper * (lp[i] - maxLp)); sw += w[i]; }
    let s2 = 0;
    for (let i = 0; i < this.n; i++) { w[i] /= sw; s2 += w[i] * w[i]; }
    this.neff = 1 / s2;
  }

  // RESAMPLE, adding random guesses when the recent fit is much worse than the long-run fit.
  resample(df) {
    this.systematic(this.n, this.injectRate(), df);
  }

  // Share of guesses replaced by random ones: zero while the scan fits as well as usual.
  injectRate() {
    if (!this.wGood) return 0;
    const ratio = this.wFast / this.wGood;
    return clamp(((0.85 - ratio) / 0.85) * 0.7, 0, 0.3);
  }

  // Best guess: start at the guess that fit the last scan best, then average its neighbourhood.
  computeEstimate() {
    const { x, y, th, w, lp, n } = this;
    let gx = 0, gy = 0, tw = 0;
    for (let i = 0; i < n; i++) { gx += w[i] * x[i]; gy += w[i] * y[i]; tw += w[i]; }
    gx /= tw; gy /= tw;
    let bi = -1, bl = -Infinity;
    for (let i = 0; i < n; i++) if (lp[i] > bl) { bl = lp[i]; bi = i; }
    let cx = gx, cy = gy;
    if (bi >= 0 && bl > -Infinity && bl !== 0) { cx = x[bi]; cy = y[bi]; }
    const R2 = 0.5 * 0.5;
    let ex = cx, ey = cy, sc = 0, ss = 0;
    for (let it = 0; it < 3; it++) {
      let sx = 0, sy = 0, sww = 0; sc = 0; ss = 0;
      for (let i = 0; i < n; i++) {
        const dx = x[i] - ex, dy = y[i] - ey;
        if (dx * dx + dy * dy > R2) continue;
        sx += w[i] * x[i]; sy += w[i] * y[i]; sww += w[i];
        sc += w[i] * Math.cos(th[i]); ss += w[i] * Math.sin(th[i]);
      }
      if (sww <= 0) break;
      ex = sx / sww; ey = sy / sww;
    }
    let v = 0;
    for (let i = 0; i < n; i++) { const dx = x[i] - ex, dy = y[i] - ey; v += w[i] * (dx * dx + dy * dy); }
    this.est.x = ex; this.est.y = ey;
    this.est.th = sc || ss ? Math.atan2(ss, sc) : this.est.th;
    this.spread = Math.sqrt(v / tw);
  }

  nearest(px, py) {
    let bi = -1, bd = 0.35 * 0.35;
    for (let i = 0; i < this.n; i++) {
      const dx = this.x[i] - px, dy = this.y[i] - py;
      const d = dx * dx + dy * dy;
      if (d < bd) { bd = d; bi = i; }
    }
    return bi;
  }

  // For the "inspect a guess" panel: where would the last scan land if the robot were at guess i?
  fitOf(i, scan, df, sigma, outPts) {
    const c = Math.cos(this.th[i]), s = Math.sin(this.th[i]);
    let hits = 0;
    for (let k = 0; k < this.kv; k++) {
      const r = scan[this.beamIdx[k]];
      const ex = this.x[i] + r * (c * this.ca[k] - s * this.sa[k]);
      const ey = this.y[i] + r * (s * this.ca[k] + c * this.sa[k]);
      const ok = df.at(ex, ey) < 2 * sigma;
      if (ok) hits++;
      outPts[3 * k] = ex; outPts[3 * k + 1] = ey; outPts[3 * k + 2] = ok ? 1 : 0;
    }
    let better = 0;
    for (let j = 0; j < this.n; j++) if (this.lp[j] > this.lp[i]) better++;
    return { hits, k: this.kv, top: Math.max(1, Math.ceil((better / this.n) * 100)) };
  }
}
