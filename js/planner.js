// Cleaning route planner: back-and-forth lanes in every room ("boustrophedon"),
// joined by A* paths through doorways. Planned once on the robot's MAP, not the real room.

import { W, H, ROBOT_R, ROOMS, DOCK } from './world.js';

const RES = 0.05;              // planning grid, 5 cm
const LANE = 0.27;             // distance between lanes (cleaning width is 0.32)
const CLEAR = ROBOT_R + 0.05;  // keep the robot centre this far from obstacles

export function planRoute(df, start) {
  const nx = Math.round(W / RES), ny = Math.round(H / RES);
  const free = new Uint8Array(nx * ny);
  for (let j = 0; j < ny; j++)
    for (let i = 0; i < nx; i++)
      free[j * nx + i] = df.at((i + 0.5) * RES, (j + 0.5) * RES) >= CLEAR ? 1 : 0;

  const cellOf = (x, y) => {
    const i = Math.floor(x / RES), j = Math.floor(y / RES);
    return i < 0 || j < 0 || i >= nx || j >= ny ? -1 : j * nx + i;
  };
  const isFree = (x, y) => { const c = cellOf(x, y); return c >= 0 && free[c] === 1; };
  const lineFree = (a, b) => {
    const L = Math.hypot(b.x - a.x, b.y - a.y);
    const n = Math.max(1, Math.ceil(L / 0.025));
    for (let k = 0; k <= n; k++) {
      const t = k / n;
      if (!isFree(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)) return false;
    }
    return true;
  };
  const center = (c) => ({ x: ((c % nx) + 0.5) * RES, y: (((c / nx) | 0) + 0.5) * RES });
  const nearestFree = (x, y) => {
    const ci = Math.floor(x / RES), cj = Math.floor(y / RES);
    for (let r = 0; r < 40; r++) {
      let best = null, bd = Infinity;
      for (let j = cj - r; j <= cj + r; j++)
        for (let i = ci - r; i <= ci + r; i++) {
          if (i < 0 || j < 0 || i >= nx || j >= ny) continue;
          if (Math.max(Math.abs(i - ci), Math.abs(j - cj)) !== r) continue;
          if (!free[j * nx + i]) continue;
          const p = center(j * nx + i);
          const d = Math.hypot(p.x - x, p.y - y);
          if (d < bd) { bd = d; best = p; }
        }
      if (best) return best;
    }
    return { x, y };
  };

  // ---- lanes for one room ----
  function roomLanes(room) {
    const alongX = room.axis === 'x';
    const a0 = alongX ? room.x0 : room.y0, a1 = alongX ? room.x1 : room.y1;
    const b0 = alongX ? room.y0 : room.x0, b1 = alongX ? room.y1 : room.x1;
    const freeAt = (a, b) => (alongX ? isFree(a, b) : isFree(b, a));
    const fraction = (b) => {
      let f = 0, n = 0;
      for (let a = a0 + 0.025; a < a1; a += 0.05) { n++; if (freeAt(a, b)) f++; }
      return f / n;
    };
    const edgeLane = (edge, dir) => {
      for (const off of [0.235, 0.26, 0.29, 0.32, 0.36]) {
        const b = edge + dir * off;
        if (fraction(b) > 0.2) return b;
      }
      return edge + dir * 0.3;
    };
    const first = edgeLane(b0, +1), last = edgeLane(b1, -1);
    const positions = [];
    for (let b = first; b <= last + 1e-6; b += LANE) positions.push(b);
    if (positions.length && last - positions[positions.length - 1] > 0.09) positions.push(last);

    const lanes = [];
    for (const b of positions) {
      const segs = [];
      let s = null;
      const step = 0.025;
      for (let a = a0 + step / 2; a <= a1; a += step) {
        const f = freeAt(a, b);
        if (f && s === null) s = a;
        if (!f && s !== null) { if (a - s > 0.12) segs.push([s, a - step]); s = null; }
      }
      if (s !== null && a1 - s > 0.12) segs.push([s, a1 - step]);
      if (segs.length) lanes.push(segs.map(([s0, e0]) => alongX
        ? [{ x: s0, y: b }, { x: e0, y: b }]
        : [{ x: b, y: s0 }, { x: b, y: e0 }]));
    }
    return lanes;
  }

  // Order a room's lanes so the walk starts at the end nearest to where we are.
  function orderRoom(lanes, cur) {
    let best = null, bd = Infinity;
    for (const rev of [false, true]) {
      for (const d0 of [1, -1]) {
        const order = rev ? lanes.slice().reverse() : lanes;
        const pts = [];
        order.forEach((segs, k) => {
          const dir = k % 2 === 0 ? d0 : -d0;
          const list = dir > 0 ? segs : segs.slice().reverse().map(([p, q]) => [q, p]);
          for (const [p, q] of list) { pts.push(p, q); }
        });
        const d = Math.hypot(pts[0].x - cur.x, pts[0].y - cur.y);
        if (d < bd) { bd = d; best = pts; }
      }
    }
    return best;
  }

  // ---- A* on the free grid ----
  const g = new Float32Array(nx * ny);
  const came = new Int32Array(nx * ny);
  const seen = new Int32Array(nx * ny);
  const shut = new Int32Array(nx * ny);
  let gen = 0;
  const heapF = [], heapC = [];
  const push = (f, c) => {
    heapF.push(f); heapC.push(c);
    let i = heapF.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (heapF[p] <= heapF[i]) break;
      [heapF[p], heapF[i]] = [heapF[i], heapF[p]];
      [heapC[p], heapC[i]] = [heapC[i], heapC[p]];
      i = p;
    }
  };
  const pop = () => {
    const c = heapC[0];
    const lf = heapF.pop(), lc = heapC.pop();
    if (heapF.length) {
      heapF[0] = lf; heapC[0] = lc;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < heapF.length && heapF[l] < heapF[m]) m = l;
        if (r < heapF.length && heapF[r] < heapF[m]) m = r;
        if (m === i) break;
        [heapF[m], heapF[i]] = [heapF[i], heapF[m]];
        [heapC[m], heapC[i]] = [heapC[i], heapC[m]];
        i = m;
      }
    }
    return c;
  };
  function astar(a, b) {
    const s = cellOf(a.x, a.y), t = cellOf(b.x, b.y);
    if (s < 0 || t < 0) return null;
    gen++;
    heapF.length = 0; heapC.length = 0;
    const ti = t % nx, tj = (t / nx) | 0;
    const h = (c) => {
      const dx = Math.abs((c % nx) - ti), dy = Math.abs(((c / nx) | 0) - tj);
      return Math.max(dx, dy) + 0.4142 * Math.min(dx, dy);
    };
    g[s] = 0; seen[s] = gen; came[s] = -1;
    push(h(s), s);
    while (heapF.length) {
      const c = pop();
      if (shut[c] === gen) continue;
      shut[c] = gen;
      if (c === t) {
        const cells = [];
        for (let k = c; k !== -1; k = came[k]) cells.push(k);
        cells.reverse();
        const pts = [a, ...cells.slice(1, -1).map(center), b];
        const out = [];
        let i = 0;
        while (i < pts.length - 1) {
          let j = pts.length - 1;
          while (j > i + 1 && !lineFree(pts[i], pts[j])) j--;
          out.push(pts[j]);
          i = j;
        }
        return out;
      }
      const ci = c % nx, cj = (c / nx) | 0;
      for (let dj = -1; dj <= 1; dj++)
        for (let di = -1; di <= 1; di++) {
          if (!di && !dj) continue;
          const i2 = ci + di, j2 = cj + dj;
          if (i2 < 0 || j2 < 0 || i2 >= nx || j2 >= ny) continue;
          const n = j2 * nx + i2;
          if (!free[n] && n !== t) continue;
          if (di && dj && (!free[cj * nx + i2] || !free[j2 * nx + ci])) continue;
          const ng = g[c] + (di && dj ? 1.4142 : 1);
          if (seen[n] !== gen || ng < g[n]) {
            seen[n] = gen; g[n] = ng; came[n] = c;
            push(ng + h(n), n);
          }
        }
    }
    return null;
  }

  // ---- assemble ----
  const startP = nearestFree(start.x, start.y);
  const seq = [];
  let cur = startP;
  for (const room of ROOMS) {
    const lanes = roomLanes(room);
    if (!lanes.length) continue;
    const pts = orderRoom(lanes, cur);
    seq.push(...pts);
    cur = pts[pts.length - 1];
  }
  seq.push(nearestFree(DOCK.x, DOCK.y));

  const route = [];
  let a = startP;
  for (const b of seq) {
    if (Math.hypot(b.x - a.x, b.y - a.y) < 0.02) continue;
    if (lineFree(a, b)) route.push({ ...b });
    else {
      const path = astar(a, b);
      if (path) for (const q of path) route.push({ ...q });
      else route.push({ ...b });
    }
    a = b;
  }
  return route;
}
