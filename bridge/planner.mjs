// Grid path planner over the arena. Pure functions, all units metres and radians, frontend frame.

export const CELL_M = 0.02;
export const EDGE_MARGIN_M = 0.07;  // the robot's centre cannot get closer than this to an arena wall (half its 0.125 m length plus a margin)
export const START_RELIEF_M = 0.06;  // cells this close to the robot are free: it is already there and must be able to leave
export const CLEARANCE_M = 0.1;  // robot half-diagonal (0.082) plus a margin: obstacles grow by this much

function distanceToSegment(p, a, b) {
  const dx = b.x - a.x, dy = b.y - a.y, lengthSq = dx * dx + dy * dy;
  const t = lengthSq === 0 ? 0 : Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / lengthSq));
  return Math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

function insidePolygon(p, points) {
  let inside = false;
  for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
    const a = points[i], b = points[j];
    if (a.y > p.y !== b.y > p.y && p.x < ((b.x - a.x) * (p.y - a.y)) / (b.y - a.y) + a.x) inside = !inside;
  }
  return inside;
}

// Distance from point p to the edge of an obstacle. Zero or negative means p is inside it.
export function distanceToObstacle(p, obstacle) {
  if (obstacle.shape === "circle") return Math.hypot(p.x - obstacle.x, p.y - obstacle.y) - (obstacle.radius ?? 0);
  if (obstacle.shape === "polygon" && obstacle.points?.length >= 3) {
    if (insidePolygon(p, obstacle.points)) return 0;
    return Math.min(...obstacle.points.map((a, i) => distanceToSegment(p, a, obstacle.points[(i + 1) % obstacle.points.length])));
  }
  // rect: rotate p into the rectangle's own axes, then measure to the box
  const c = Math.cos(-obstacle.yaw), s = Math.sin(-obstacle.yaw), dx = p.x - obstacle.x, dy = p.y - obstacle.y;
  const ox = Math.abs(dx * c - dy * s) - (obstacle.width ?? 0) / 2, oy = Math.abs(dx * s + dy * c) - (obstacle.length ?? 0) / 2;
  return ox <= 0 && oy <= 0 ? Math.max(ox, oy) : Math.hypot(Math.max(ox, 0), Math.max(oy, 0));
}

// A* over an 8-connected grid. Returns waypoints from start to goal, or null when no walkable path exists.
// A goal inside an obstacle (for example "go to the chocolate") is replaced by the nearest free cell.
// The start cell is always treated as free, because the robot is already there.
export function planPath(start, goal, arena, obstacles, clearance = CLEARANCE_M) {
  const cols = Math.max(1, Math.ceil(arena.width / CELL_M)), rows = Math.max(1, Math.ceil(arena.length / CELL_M));
  const centre = (i) => ({ x: ((i % cols) + 0.5) * CELL_M, y: (Math.floor(i / cols) + 0.5) * CELL_M });
  const cellOf = (p) => {
    const cx = Math.min(cols - 1, Math.max(0, Math.floor(p.x / CELL_M))), cy = Math.min(rows - 1, Math.max(0, Math.floor(p.y / CELL_M)));
    return cy * cols + cx;
  };
  const blocked = new Uint8Array(cols * rows);
  for (let i = 0; i < blocked.length; i++) {
    const p = centre(i);
    const nearWall = p.x < EDGE_MARGIN_M || p.y < EDGE_MARGIN_M || p.x > arena.width - EDGE_MARGIN_M || p.y > arena.length - EDGE_MARGIN_M;
    blocked[i] = nearWall || obstacles.some((o) => distanceToObstacle(p, o) < clearance) ? 1 : 0;
    if (Math.hypot(p.x - start.x, p.y - start.y) < START_RELIEF_M) blocked[i] = 0;
  }
  const startCell = cellOf(start);
  blocked[startCell] = 0;
  let goalCell = cellOf(goal), goalMoved = false;
  if (blocked[goalCell]) {
    let best = -1, bestDistance = Infinity;
    for (let i = 0; i < blocked.length; i++) {
      if (blocked[i]) continue;
      const p = centre(i), d = Math.hypot(p.x - goal.x, p.y - goal.y);
      if (d < bestDistance) { best = i; bestDistance = d; }
    }
    if (best < 0) return null;
    goalCell = best;
    goalMoved = true;
  }

  const cost = new Float64Array(cols * rows).fill(Infinity), from = new Int32Array(cols * rows).fill(-1);
  const heuristic = (i) => Math.hypot((i % cols) - (goalCell % cols), Math.floor(i / cols) - Math.floor(goalCell / cols));
  const open = new Set([startCell]);
  cost[startCell] = 0;
  while (open.size > 0) {
    let current = -1, best = Infinity;
    for (const i of open) {
      const f = cost[i] + heuristic(i);
      if (f < best) { best = f; current = i; }
    }
    if (current === goalCell) break;
    open.delete(current);
    const cx = current % cols, cy = Math.floor(current / cols);
    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        const nx = cx + dx, ny = cy + dy;
        if ((dx === 0 && dy === 0) || nx < 0 || ny < 0 || nx >= cols || ny >= rows) continue;
        const next = ny * cols + nx;
        // no cutting diagonally between two blocked cells
        if (blocked[next] || (dx !== 0 && dy !== 0 && (blocked[cy * cols + nx] || blocked[ny * cols + cx]))) continue;
        const candidate = cost[current] + Math.hypot(dx, dy);
        if (candidate < cost[next]) {
          cost[next] = candidate;
          from[next] = current;
          open.add(next);
        }
      }
    }
  }
  if (cost[goalCell] === Infinity) return null;

  const cells = [];
  for (let i = goalCell; i !== -1; i = from[i]) cells.unshift(i);
  const points = [{ x: start.x, y: start.y }, ...cells.slice(1, -1).map(centre), goalMoved ? centre(goalCell) : { x: goal.x, y: goal.y }];

  // Keep only the corners: drop a waypoint when the straight line past it stays clear of every obstacle.
  const clear = (a, b) => {
    const steps = Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / (CELL_M / 2));
    for (let k = 1; k < steps; k++) {
      const p = { x: a.x + ((b.x - a.x) * k) / steps, y: a.y + ((b.y - a.y) * k) / steps };
      if (cellOf(p) !== startCell && blocked[cellOf(p)]) return false;
    }
    return true;
  };
  const path = [points[0]];
  for (let i = 0; i < points.length - 1; ) {
    let j = points.length - 1;
    while (j > i + 1 && !clear(points[i], points[j])) j--;
    path.push(points[j]);
    i = j;
  }
  // The goal was in an unreachable spot (against a wall or inside an obstacle's clearance), so the path ends at the nearest reachable one.
  path.goalMoved = goalMoved;
  return path;
}
