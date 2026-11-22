// Floor textures drawn with Canvas 2D: the clean floor, the dust the robot removes,
// the trails, and the robot's distance map. All flat colour, no gradients.

import { W, H, ROOMS, RUG } from './world.js';
import { rng } from './util.js';

export const FX0 = -0.15, FY0 = -0.15, FW = W + 0.3, FH = H + 0.3;
export const PX_BASE = 150;   // pixels per metre for the static floor
export const PX_DYN = 100;    // pixels per metre for dust and trails

export function makeCanvas(px) {
  const c = document.createElement('canvas');
  c.width = Math.round(FW * px);
  c.height = Math.round(FH * px);
  return c;
}

export function paintBase(canvas, labels) {
  const px = canvas.width / FW;
  const ctx = canvas.getContext('2d');
  const X = (x) => (x - FX0) * px, Y = (y) => (y - FY0) * px;
  const R = rng(11);
  ctx.fillStyle = '#F1F0EC';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  for (const room of ROOMS) {
    const x0 = room.x0 - 0.06, y0 = room.y0 - 0.06, x1 = room.x1 + 0.06, y1 = room.y1 + 0.06;
    ctx.save();
    ctx.beginPath();
    ctx.rect(X(x0), Y(y0), (x1 - x0) * px, (y1 - y0) * px);
    ctx.clip();
    if (room.floor === 'wood') {
      const tones = ['#E9E2D6', '#E6DED1', '#ECE6DB', '#E3DBCE', '#E8E0D4', '#EAE4D9'];
      const pw = 0.15;
      for (let y = y0; y < y1; y += pw) {
        let x = x0 - R() * 1.4;
        while (x < x1) {
          const L = 0.9 + R() * 0.9;
          ctx.fillStyle = tones[(R() * tones.length) | 0];
          ctx.fillRect(X(x), Y(y), L * px, pw * px);
          ctx.fillStyle = '#D9D0C3';
          ctx.fillRect(X(x + L) - 0.7, Y(y), 1.4, pw * px);
          x += L;
        }
        ctx.fillStyle = '#DDD5C8';
        ctx.fillRect(X(x0), Y(y) - 0.6, (x1 - x0) * px, 1.2);
      }
    } else {
      const ts = 0.4;
      const tones = ['#EDECE8', '#EBEAE6', '#EEEDEA'];
      for (let y = room.y0; y < y1; y += ts)
        for (let x = room.x0; x < x1; x += ts) {
          ctx.fillStyle = tones[(R() * tones.length) | 0];
          ctx.fillRect(X(x), Y(y), ts * px, ts * px);
        }
      ctx.fillStyle = '#DCDBD6';
      for (let y = room.y0; y < y1; y += ts) ctx.fillRect(X(x0), Y(y) - 1, (x1 - x0) * px, 2);
      for (let x = room.x0; x < x1; x += ts) ctx.fillRect(X(x) - 1, Y(y0), 2, (y1 - y0) * px);
    }
    ctx.restore();
  }

  // Rug: three flat bands, like a woven border.
  const bands = [['#D6D0C4', 0], ['#DEDAD0', 0.07], ['#D6D0C4', 0.13], ['#E1DDD4', 0.16]];
  for (const [c, inset] of bands) {
    ctx.fillStyle = c;
    ctx.fillRect(X(RUG.x0 + inset), Y(RUG.y0 + inset), (RUG.x1 - RUG.x0 - 2 * inset) * px, (RUG.y1 - RUG.y0 - 2 * inset) * px);
  }
  ctx.fillStyle = 'rgba(160,150,135,0.10)';
  for (let y = RUG.y0 + 0.2; y < RUG.y1 - 0.2; y += 0.035) ctx.fillRect(X(RUG.x0 + 0.2), Y(y), (RUG.x1 - RUG.x0 - 0.4) * px, 1.5);

  if (labels) {
    ctx.fillStyle = '#a8a296';
    ctx.textBaseline = 'alphabetic';
    ctx.font = `800 ${Math.round(0.12 * px)}px Figtree, system-ui, sans-serif`;
    if ('letterSpacing' in ctx) ctx.letterSpacing = '0px';
    for (const room of ROOMS) {
      const t = labels[room.id];
      if (t) ctx.fillText(t, X(room.label[0]), Y(room.label[1]));
    }
  }
}

// Dust: a speckled film over the whole floor. The robot erases it where it really drives.
export class DustLayer {
  constructor() {
    this.canvas = makeCanvas(PX_DYN);
    this.ctx = this.canvas.getContext('2d');
    this.px = this.canvas.width / FW;
    this.reset();
  }

  reset() {
    const { ctx, px } = this;
    const X = (x) => (x - FX0) * px, Y = (y) => (y - FY0) * px;
    const R = rng((Math.random() * 1e9) | 0);
    ctx.globalCompositeOperation = 'source-over';
    ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    ctx.fillStyle = 'rgba(120,106,84,0.2)';
    ctx.fillRect(X(0), Y(0), W * px, H * px);
    const specks = Math.round(W * H * 650);
    for (let i = 0; i < specks; i++) {
      const a = 0.18 + R() * 0.35;
      ctx.fillStyle = `rgba(92,82,66,${a.toFixed(2)})`;
      const s = R() < 0.85 ? 1 : 2;
      ctx.fillRect(X(R() * W), Y(R() * H), s, s);
    }
    ctx.strokeStyle = 'rgba(78,72,64,0.38)';
    ctx.lineWidth = 1;
    for (let i = 0; i < 260; i++) {
      const x = R() * W, y = R() * H, a = R() * Math.PI * 2, L = 0.02 + R() * 0.05;
      ctx.beginPath();
      ctx.moveTo(X(x), Y(y));
      ctx.quadraticCurveTo(X(x + Math.cos(a) * L * 0.6 + 0.01), Y(y + Math.sin(a) * L * 0.6), X(x + Math.cos(a + 0.6) * L), Y(y + Math.sin(a + 0.6) * L));
      ctx.stroke();
    }
    for (let i = 0; i < 90; i++) {
      ctx.fillStyle = 'rgba(150,124,96,0.55)';
      ctx.fillRect(X(R() * W), Y(R() * H), 3, 2);
    }
    this.dirty = true;
  }
}

// The planned route, drawn into a floor texture when it changes.
export class PathLayer {
  constructor() {
    this.canvas = makeCanvas(PX_DYN);
    this.ctx = this.canvas.getContext('2d');
    this.px = this.canvas.width / FW;
  }

  draw({ plan, showPlan }) {
    const { ctx, px } = this;
    ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    const X = (x) => (x - FX0) * px, Y = (y) => (y - FY0) * px;
    ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    if (showPlan && plan.length) {
      ctx.setLineDash([5, 6]);
      ctx.strokeStyle = 'rgba(18,18,18,0.32)';
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      ctx.moveTo(X(plan[0].x), Y(plan[0].y));
      for (const p of plan) ctx.lineTo(X(p.x), Y(p.y));
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }
}

// The robot's map as it sees it: distance to the nearest wall in flat contour bands.
export function paintField(canvas, df) {
  const px = canvas.width / FW;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const bands = [
    [0.03, 'rgba(18,18,18,0.78)'],
    [0.10, 'rgba(255,79,0,0.62)'],
    [0.20, 'rgba(255,79,0,0.40)'],
    [0.35, 'rgba(255,79,0,0.24)'],
    [0.60, 'rgba(255,79,0,0.12)'],
  ];
  const s = 0.025;
  for (let y = 0; y < H; y += s)
    for (let x = 0; x < W; x += s) {
      const d = df.at(x + s / 2, y + s / 2);
      for (const [lim, c] of bands) {
        if (d < lim) {
          ctx.fillStyle = c;
          ctx.fillRect((x - FX0) * px, (y - FY0) * px, s * px + 0.6, s * px + 0.6);
          break;
        }
      }
    }
}
