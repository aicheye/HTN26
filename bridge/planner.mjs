// Grid path planner over the arena. Pure functions, all units metres and radians, frontend frame.
// The soft cost near obstacles, the table edge as a limit, and the rescue of a start that is too close to an
// obstacle follow Arjun's nav/planner.py.

export const CELL_M = 0.02;
// How far the robot reaches from its marker. 0.082 m is the body's half-diagonal. vision/scan.py measures the real
// reach in the camera picture, legs included, and the bridge passes it in as robotRadius.
export const ROBOT_RADIUS_M = 0.082;
const CLEARANCE_EXTRA_M = 0.018;
export const CLEARANCE_M = ROBOT_RADIUS_M + CLEARANCE_EXTRA_M;  // obstacles grow by the robot's reach plus a margin
// The strip of table that holds the corner markers is closed to the robot: no part of it may ever be there. The
// arena is the rectangle between the marker centres, and the markers are TAG_M wide and flush with the table's
// corners. The strip therefore reaches TAG_M / 2 into the arena, and the robot's centre has to stay its reach
// further in: 0.04 + 0.082 = 0.122 m from the arena's edge. On a 0.63 m arena that leaves a 0.386 m square for
// the centre. This is a hard limit for planning (below) and for walking (leavesArena).
export const TAG_M = 0.08;
export const edgeMargin = (robotRadius = ROBOT_RADIUS_M) => TAG_M / 2 + robotRadius;
export const EDGE_MARGIN_M = edgeMargin();
// A free cell costs up to 1 + SOFT_WEIGHT times its length right at a limit, falling to 1 SOFT_BAND_M further out.
// This centres paths in gaps, where a plain shortest path runs along the limit and any walking error puts the
// robot against the obstacle. The band is 5 cm because the robot's centre only has a 0.386 m square to move in:
// a wider band covered most of it and bent paths that could be straight.
const SOFT_BAND_M = 0.05;
const SOFT_WEIGHT = 3;
// A straight line replaces a bent stretch of path when it costs at most this much more. Every bend is a stop and
// a turn in place for the robot, so a slightly dearer straight line is the better trade.
const SHORTCUT_TOLERANCE = 1.15;

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

// How far a point is inside the closed strip along the arena's edge. 0 when it is outside the strip.
function edgeDepth(p, arena, margin) {
  return Math.max(0, margin - Math.min(p.x, arena.width - p.x, p.y, arena.length - p.y));
}

// True when walking with this command takes the robot into the closed strip, or deeper into it. Turning is always
// allowed, and so is walking out of the strip or along it, so that a robot that ended up there can leave.
export function leavesArena(robot, arena, command, robotRadius = ROBOT_RADIUS_M) {
  if ((command !== "forward" && command !== "backward") || !(arena?.width > 0 && arena?.length > 0)) return false;
  const sign = command === "forward" ? 1 : -1, ahead = 0.02;
  const next = { x: robot.x + sign * Math.cos(robot.yaw) * ahead, y: robot.y + sign * Math.sin(robot.yaw) * ahead };
  return edgeDepth(next, arena, edgeMargin(robotRadius)) > edgeDepth(robot, arena, edgeMargin(robotRadius)) + 1e-9;
}

// A* over an 8-connected grid. Returns waypoints from start to goal, or null when no walkable path exists.
// A goal inside an obstacle (for example "go to the chocolate") is replaced by the nearest free cell.
export function planPath(start, goal, arena, obstacles, robotRadius = ROBOT_RADIUS_M) {
  const clearance = robotRadius + CLEARANCE_EXTRA_M, margin = edgeMargin(robotRadius);
  const cols = Math.max(1, Math.ceil(arena.width / CELL_M)), rows = Math.max(1, Math.ceil(arena.length / CELL_M));
  const centre = (i) => ({ x: ((i % cols) + 0.5) * CELL_M, y: (Math.floor(i / cols) + 0.5) * CELL_M });
  const cellOf = (p) => {
    const cx = Math.min(cols - 1, Math.max(0, Math.floor(p.x / CELL_M))), cy = Math.min(rows - 1, Math.max(0, Math.floor(p.y / CELL_M)));
    return cy * cols + cx;
  };
  const blocked = new Uint8Array(cols * rows), cellCost = new Float32Array(cols * rows);
  for (let i = 0; i < blocked.length; i++) {
    const p = centre(i);
    // slack: how far the robot's centre is beyond the nearest limit, an obstacle's clearance or the table edge
    let slack = Math.min(p.x, arena.width - p.x, p.y, arena.length - p.y) - margin;
    for (const o of obstacles) slack = Math.min(slack, distanceToObstacle(p, o) - clearance);
    blocked[i] = slack < 0 ? 1 : 0;
    cellCost[i] = 1 + SOFT_WEIGHT * Math.max(0, Math.min(1, 1 - slack / SOFT_BAND_M));
  }
  const nearestFree = (to) => {
    let best = -1, bestDistance = Infinity;
    for (let i = 0; i < blocked.length; i++) {
      if (blocked[i]) continue;
      const p = centre(i), d = Math.hypot(p.x - to.x, p.y - to.y);
      if (d < bestDistance) { best = i; bestDistance = d; }
    }
    return best;
  };
  // A robot that stands closer to an obstacle or the edge than the limit first walks straight to the nearest free
  // cell. Treating only its own cell as free left it walled in by blocked neighbours, and the plan failed.
  let startCell = cellOf(start);
  const rescued = blocked[startCell] === 1;
  if (rescued) startCell = nearestFree(start);
  if (startCell < 0) return null;
  let goalCell = cellOf(goal), goalMoved = false;
  if (blocked[goalCell]) {
    goalCell = nearestFree(goal);
    goalMoved = true;
  }
  if (goalCell < 0) return null;

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
        const candidate = cost[current] + Math.hypot(dx, dy) * (cellCost[current] + cellCost[next]) / 2;
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
  const points = [{ x: start.x, y: start.y }, ...(rescued ? [centre(startCell)] : []), ...cells.slice(1, -1).map(centre),
    goalMoved ? centre(goalCell) : { x: goal.x, y: goal.y }];

  // Cost of walking the straight line from a to b, in the same units as the search. Infinity when it is blocked.
  const lineCost = (a, b) => {
    const length = Math.hypot(b.x - a.x, b.y - a.y), steps = Math.max(1, Math.ceil(length / (CELL_M / 2)));
    let total = 0;
    for (let k = 0; k <= steps; k++) {
      const cell = cellOf({ x: a.x + ((b.x - a.x) * k) / steps, y: a.y + ((b.y - a.y) * k) / steps });
      if (blocked[cell]) return Infinity;
      total += cellCost[cell];
    }
    return (total / (steps + 1)) * length / CELL_M;
  };
  // Keep only the corners: drop the waypoints between two points when the straight line between them is clear and
  // costs no more than SHORTCUT_TOLERANCE times the stretch it replaces. The first leg of a rescued start is kept.
  const first = rescued ? 1 : 0;
  const path = points.slice(0, first + 1);
  for (let i = first; i < points.length - 1; ) {
    let j = points.length - 1;
    for (; j > i + 1; j--) {
      let stretch = 0;
      for (let k = i; k < j; k++) stretch += lineCost(points[k], points[k + 1]);
      if (lineCost(points[i], points[j]) <= stretch * SHORTCUT_TOLERANCE) break;
    }
    path.push(points[j]);
    i = j;
  }
  return path;
}
