// The apartment: walls, furniture, and geometric queries on them.
// Units are metres. x grows to the right, y grows "down" the plan (towards the viewer).

export const W = 9.0;
export const H = 6.0;
export const ROBOT_R = 0.175;           // robot body radius (35 cm robot)
export const DOCK = { x: 0.37, y: 1.75, th: 0 };
const WALL_T = 0.12;                     // interior wall thickness
const OUTER_T = 0.15;                    // outer wall thickness

export const ROOMS = [
  { id: 'living',  x0: 0,   y0: 0,   x1: 5.2, y1: 3.6, floor: 'wood', axis: 'x', label: [2.62, 3.4] },
  { id: 'bedroom', x0: 0,   y0: 3.6, x1: 4.0, y1: 6.0, floor: 'wood', axis: 'x', label: [0.2, 3.9] },
  { id: 'hall',    x0: 4.0, y0: 3.6, x1: 9.0, y1: 6.0, floor: 'tile', axis: 'x', label: [4.26, 3.9] },
  { id: 'kitchen', x0: 5.2, y0: 0,   x1: 9.0, y1: 3.6, floor: 'tile', axis: 'y', label: [5.44, 3.4] },
];

// A rug: wheels slip more on it.
export const RUG = { x0: 0.75, y0: 1.22, x1: 3.3, y1: 2.72 };

const rect = (x0, y0, x1, y1) => ({ type: 'rect', x0, y0, x1, y1 });

export function buildWalls() {
  const h = WALL_T / 2;
  return [
    // outer shell
    rect(-OUTER_T, -OUTER_T, W + OUTER_T, 0),
    rect(-OUTER_T, H, W + OUTER_T, H + OUTER_T),
    rect(-OUTER_T, 0, 0, H),
    rect(W, 0, W + OUTER_T, H),
    // living | kitchen, doorway y 1.3..2.3
    rect(5.2 - h, 0, 5.2 + h, 1.3),
    rect(5.2 - h, 2.3, 5.2 + h, 3.6 - h),
    // the long wall at y = 3.6, doorways x 2.9..3.8 and 6.3..7.2
    rect(0, 3.6 - h, 2.9, 3.6 + h),
    rect(3.8, 3.6 - h, 6.3, 3.6 + h),
    rect(7.2, 3.6 - h, W, 3.6 + h),
    // bedroom | hall, doorway y 4.4..5.2
    rect(4.0 - h, 3.6 + h, 4.0 + h, 4.4),
    rect(4.0 - h, 5.2, 4.0 + h, H),
  ];
}

// back: which side faces the wall (n = smaller y, s = larger y, w = smaller x, e = larger x)
export const FURNITURE = [
  { id: 'sofa',     kind: 'sofa',     cx: 1.55, cy: 0.5,  w: 2.4,  d: 0.9,  h: 0.42, back: 'n', movable: true },
  { id: 'armchair', kind: 'armchair', cx: 4.45, cy: 0.62, w: 0.85, d: 0.85, h: 0.42, back: 'n', movable: true },
  { id: 'media',    kind: 'media',    cx: 1.5,  cy: 3.3,  w: 1.8,  d: 0.36, h: 0.36, back: 's', movable: false },
  { id: 'coffee',   kind: 'table',    cx: 1.8,  cy: 1.95, w: 1.1,  d: 0.6,  h: 0.4,  movable: true },
  { id: 'dock',     kind: 'dock',     cx: 0.06, cy: 1.75, w: 0.12, d: 0.3,  h: 0.1,  back: 'w', movable: false },
  { id: 'counter',  kind: 'counter',  cx: 8.7,  cy: 1.77, w: 0.6,  d: 3.54, h: 0.5,  back: 'e', movable: false },
  { id: 'dining',   kind: 'table',    cx: 6.7,  cy: 1.35, w: 1.4,  d: 0.9,  h: 0.45, movable: true },
  { id: 'chair1',   kind: 'chair',    cx: 6.36, cy: 0.62, w: 0.42, d: 0.42, h: 0.45, back: 'n', movable: true },
  { id: 'chair2',   kind: 'chair',    cx: 7.04, cy: 0.62, w: 0.42, d: 0.42, h: 0.45, back: 'n', movable: true },
  { id: 'chair3',   kind: 'chair',    cx: 6.36, cy: 2.08, w: 0.42, d: 0.42, h: 0.45, back: 's', movable: true },
  { id: 'chair4',   kind: 'chair',    cx: 7.04, cy: 2.08, w: 0.42, d: 0.42, h: 0.45, back: 's', movable: true },
  { id: 'bed',      kind: 'bed',      cx: 0.95, cy: 5.05, w: 1.9,  d: 1.9,  h: 0.45, back: 'w', movable: false },
  { id: 'nightstand', kind: 'cabinet', cx: 2.16, cy: 5.73, w: 0.42, d: 0.42, h: 0.45, movable: true },
  { id: 'wardrobe', kind: 'wardrobe', cx: 3.25, cy: 5.7,  w: 1.3,  d: 0.6,  h: 0.6,  back: 's', movable: false },
  { id: 'shoes',    kind: 'cabinet',  cx: 8.8,  cy: 4.7,  w: 0.4,  d: 1.3,  h: 0.45, movable: false },
  { id: 'plant',    kind: 'plant',    cx: 4.45, cy: 5.62, r: 0.2,  h: 0.4,  movable: true },
  { id: 'basket',   kind: 'basket',   cx: 6.2,  cy: 5.68, r: 0.22, h: 0.38, movable: true },
];

// Leg layout shared by the physics and the 3D model so they always match.
export const LEGS = {
  table: { s: 0.05, inset: 0.05 },
  chair: { s: 0.03, inset: 0.025 },
};

export function footprint(it) {
  if (it.r) return { x0: it.cx - it.r, y0: it.cy - it.r, x1: it.cx + it.r, y1: it.cy + it.r };
  return { x0: it.cx - it.w / 2, y0: it.cy - it.d / 2, x1: it.cx + it.w / 2, y1: it.cy + it.d / 2 };
}

// What the lidar and the bumper can actually hit. Tables and chairs are only their legs.
export function itemObstacles(it) {
  if (it.r) return [{ type: 'circle', cx: it.cx, cy: it.cy, r: it.r }];
  const f = footprint(it);
  const legs = LEGS[it.kind];
  if (legs) {
    const { s, inset } = legs;
    const xs = [f.x0 + inset, f.x1 - inset - s];
    const ys = [f.y0 + inset, f.y1 - inset - s];
    const out = [];
    for (const x of xs) for (const y of ys) out.push(rect(x, y, x + s, y + s));
    return out;
  }
  return [rect(f.x0, f.y0, f.x1, f.y1)];
}

function distToObstacle(x, y, o) {
  if (o.type === 'rect') {
    const dx = Math.max(o.x0 - x, 0, x - o.x1);
    const dy = Math.max(o.y0 - y, 0, y - o.y1);
    return Math.sqrt(dx * dx + dy * dy);
  }
  const d = Math.hypot(x - o.cx, y - o.cy) - o.r;
  return d > 0 ? d : 0;
}

export class World {
  constructor(items) {
    this.walls = buildWalls();
    this.items = (items || FURNITURE).map((f) => ({ ...f }));
    this.rebuild();
  }

  clone() { return new World(this.items); }

  rebuild() {
    this.obstacles = this.walls.concat(this.items.flatMap(itemObstacles));
  }

  item(id) { return this.items.find((i) => i.id === id); }

  // Nearest distance from a point to anything solid.
  dist(x, y) {
    let best = Infinity;
    for (const o of this.obstacles) {
      const d = distToObstacle(x, y, o);
      if (d < best) { best = d; if (d === 0) return 0; }
    }
    return best;
  }

  collides(x, y, r) {
    for (const o of this.obstacles) if (distToObstacle(x, y, o) < r) return true;
    return false;
  }

  // Distance along a unit ray to the first obstacle, or maxR.
  raycast(x, y, dx, dy, maxR) {
    let best = maxR;
    const ix = 1 / (Math.abs(dx) < 1e-9 ? 1e-9 : dx);
    const iy = 1 / (Math.abs(dy) < 1e-9 ? 1e-9 : dy);
    for (const o of this.obstacles) {
      if (o.type === 'rect') {
        let t1 = (o.x0 - x) * ix, t2 = (o.x1 - x) * ix;
        let tmin = Math.min(t1, t2), tmax = Math.max(t1, t2);
        t1 = (o.y0 - y) * iy; t2 = (o.y1 - y) * iy;
        tmin = Math.max(tmin, Math.min(t1, t2));
        tmax = Math.min(tmax, Math.max(t1, t2));
        if (tmax >= Math.max(tmin, 0) && tmin < best) best = tmin > 0 ? tmin : 0;
      } else {
        const ox = x - o.cx, oy = y - o.cy;
        const b = ox * dx + oy * dy;
        const c = ox * ox + oy * oy - o.r * o.r;
        const disc = b * b - c;
        if (disc >= 0) {
          const sq = Math.sqrt(disc);
          let t = -b - sq;
          if (t < 0) t = -b + sq;
          if (t >= 0 && t < best) best = t;
        }
      }
    }
    return best;
  }

  // Try to move a piece of furniture. Refuses spots that hit walls or the robot.
  tryMove(id, cx, cy, robot) {
    const it = this.item(id);
    if (!it) return false;
    const probe = { ...it, cx, cy };
    const f = footprint(probe);
    if (f.x0 < 0.01 || f.y0 < 0.01 || f.x1 > W - 0.01 || f.y1 > H - 0.01) return false;
    const obs = itemObstacles(probe);
    // must not overlap walls
    for (const wall of this.walls) {
      for (const o of obs) {
        const b = o.type === 'rect' ? o : { x0: o.cx - o.r, y0: o.cy - o.r, x1: o.cx + o.r, y1: o.cy + o.r };
        if (b.x0 < wall.x1 && b.x1 > wall.x0 && b.y0 < wall.y1 && b.y1 > wall.y0) return false;
      }
    }
    if (robot) {
      for (const o of obs) if (distToObstacle(robot.x, robot.y, o) < ROBOT_R + 0.01) return false;
    }
    it.cx = cx; it.cy = cy;
    this.rebuild();
    return true;
  }
}

// Precomputed "how far is the nearest wall" for every 2.5 cm cell.
// The particle filter looks this up instead of casting rays: the likelihood-field model.
export class DistField {
  constructor(world, res = 0.025, freeClear = ROBOT_R + 0.02) {
    this.res = res;
    this.x0 = -0.2; this.y0 = -0.2;
    this.nx = Math.ceil((W + 0.4) / res);
    this.ny = Math.ceil((H + 0.4) / res);
    this.d = new Float32Array(this.nx * this.ny);
    const free = [];
    for (let j = 0; j < this.ny; j++) {
      const y = this.y0 + (j + 0.5) * res;
      for (let i = 0; i < this.nx; i++) {
        const x = this.x0 + (i + 0.5) * res;
        const d = world.dist(x, y);
        this.d[j * this.nx + i] = d;
        if (d >= freeClear && x > 0 && y > 0 && x < W && y < H) free.push(j * this.nx + i);
      }
    }
    this.free = Int32Array.from(free);
  }

  at(x, y) {
    if (x < this.x0 || y < this.y0) return 1;
    const i = ((x - this.x0) / this.res) | 0;
    const j = ((y - this.y0) / this.res) | 0;
    if (i >= this.nx || j >= this.ny) return 1;
    return this.d[j * this.nx + i];
  }

  randomFree() {
    const c = this.free[(Math.random() * this.free.length) | 0];
    const i = c % this.nx, j = (c / this.nx) | 0;
    return { x: this.x0 + (i + Math.random()) * this.res, y: this.y0 + (j + Math.random()) * this.res };
  }
}
