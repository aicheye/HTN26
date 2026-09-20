// Grid path planner over the arena. Pure functions, all units metres and radians, frontend frame.
// The soft cost near obstacles, the table edge as a limit, and the rescue of a start that is too close to an
// obstacle follow Arjun's nav/planner.py. The limit at the arena's edge is Angus's.

export const CELL_M = 0.02;
// How far the robot reaches from its marker. 0.082 m is the body's half-diagonal. vision/scan.py measures the real
// reach in the camera picture, legs included, and the bridge passes it in as robotRadius.
export const ROBOT_RADIUS_M = 0.082;
const CLEARANCE_EXTRA_M = 0.018;
export const CLEARANCE_M = ROBOT_RADIUS_M + CLEARANCE_EXTRA_M;  // obstacles grow by the robot's reach plus a margin
// The robot's centre cannot get closer than this to the arena's edge: half its 0.125 m length plus a margin. This
// is Angus's limit from the frontend branch, and the one limit at the edge for everything: planning (below),
// walking (leavesArena), the voice commands (src/state/voiceCommands.ts) and the line both maps draw. It does not
// grow with the measured robot radius. A stricter one, 0.122 m, which kept every part of the robot off the strip
// that holds the corner markers, left a 0.39 m square on a 0.63 m arena and few goals walkable among objects.
// With 0.07 m the robot's corners can reach 1.2 cm past the arena's edge, over the markers' strip. The table
// itself ends 4 cm past that edge.
export const EDGE_MARGIN_M = 0.07;
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

// How far a point is past the limit at the arena's edge. 0 when it is inside the limit.
function edgeDepth(p, arena) {
  return Math.max(0, EDGE_MARGIN_M - Math.min(p.x, arena.width - p.x, p.y, arena.length - p.y));
}

// True when walking with this command takes the robot's centre past the limit at the edge, or further past it.
// Turning is always allowed, and so is walking back inside or along the edge, so that a robot out there can return.
export function leavesArena(robot, arena, command) {
  if ((command !== "forward" && command !== "backward") || !(arena?.width > 0 && arena?.length > 0)) return false;
  const sign = command === "forward" ? 1 : -1, ahead = 0.02;
  const next = { x: robot.x + sign * Math.cos(robot.yaw) * ahead, y: robot.y + sign * Math.sin(robot.yaw) * ahead };
  return edgeDepth(next, arena) > edgeDepth(robot, arena) + 1e-9;
}

// The arena as a grid: which cells the robot's centre may be in, and what each free cell costs.
function buildGrid(arena, obstacles, robotRadius) {
  const clearance = robotRadius + CLEARANCE_EXTRA_M, margin = EDGE_MARGIN_M;
  const cols = Math.max(1, Math.ceil(arena.width / CELL_M)), rows = Math.max(1, Math.ceil(arena.length / CELL_M));
  const centre = (i) => ({ x: ((i % cols) + 0.5) * CELL_M, y: (Math.floor(i / cols) + 0.5) * CELL_M });
  const cellOf = (p) => {
    const cx = Math.min(cols - 1, Math.max(0, Math.floor(p.x / CELL_M))), cy = Math.min(rows - 1, Math.max(0, Math.floor(p.y / CELL_M)));
    return cy * cols + cx;
  };
  const blocked = new Uint8Array(cols * rows), cellCost = new Float32Array(cols * rows), slackOf = new Float32Array(cols * rows);
  for (let i = 0; i < blocked.length; i++) {
    const p = centre(i);
    // slack: how far the robot's centre is beyond the nearest limit, an obstacle's clearance or the table edge
    let slack = Math.min(p.x, arena.width - p.x, p.y, arena.length - p.y) - margin;
    for (const o of obstacles) slack = Math.min(slack, distanceToObstacle(p, o) - clearance);
    slackOf[i] = slack;
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
  // Free neighbours of a cell. No cutting diagonally between two blocked cells.
  const neighbours = (current) => {
    const cx = current % cols, cy = Math.floor(current / cols), out = [];
    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        const nx = cx + dx, ny = cy + dy;
        if ((dx === 0 && dy === 0) || nx < 0 || ny < 0 || nx >= cols || ny >= rows) continue;
        const next = ny * cols + nx;
        if (blocked[next] || (dx !== 0 && dy !== 0 && (blocked[cy * cols + nx] || blocked[ny * cols + cx]))) continue;
        out.push([next, Math.hypot(dx, dy)]);
      }
    }
    return out;
  };
  return { cols, rows, centre, cellOf, blocked, cellCost, slackOf, nearestFree, neighbours };
}

// The grid the planner works on, for display: per cell the cost of walking through it (1 on open floor, up to
// 1 + SOFT_WEIGHT at a limit), or -1 where the robot's centre may not be. Row 0 is y = 0.
export function costmap(arena, obstacles, robotRadius = ROBOT_RADIUS_M) {
  const grid = buildGrid(arena, obstacles, robotRadius);
  return {
    cell: CELL_M, cols: grid.cols, rows: grid.rows, clearance: robotRadius + CLEARANCE_EXTRA_M, edgeMargin: EDGE_MARGIN_M,
    cost: Array.from(grid.cellCost, (c, i) => (grid.blocked[i] ? -1 : Math.round(c * 10) / 10)),
  };
}

// Where an arm can set the robot down so that it can walk to the goal, for when no path exists from where it
// stands. Returns up to `count` points, best first, or [] when there is none: free cells that are connected to the
// goal, at least CARRY_SLACK_M clear of every limit so that a drop that is a little off still lands on free floor,
// and within `reach` of the arm's base. The ones at 0.8 of its reach come first, because those are the ones
// its kinematics most likely serve. Whether the arm really reaches a point is for the arm's own planner to say.
export const CARRY_SLACK_M = 0.03;
const PREFERRED_REACH = 0.8;

// Every free cell that can be walked to from `fromCell`, in no particular order.
function connected(grid, fromCell) {
  const seen = new Uint8Array(grid.blocked.length), queue = [fromCell], cells = [];
  seen[fromCell] = 1;
  while (queue.length > 0) {
    const current = queue.pop();
    cells.push(current);
    for (const [next] of grid.neighbours(current)) if (!seen[next]) { seen[next] = 1; queue.push(next); }
  }
  return cells;
}

export function carryTargets(goal, arena, obstacles, armBase, reach, robotRadius = ROBOT_RADIUS_M, count = 5) {
  const grid = buildGrid(arena, obstacles, robotRadius);
  let goalCell = grid.cellOf(goal);
  if (grid.blocked[goalCell]) goalCell = grid.nearestFree(goal);
  if (goalCell < 0) return [];
  const found = [];
  for (const cell of connected(grid, goalCell)) {
    const p = grid.centre(cell), distance = Math.hypot(p.x - armBase.x, p.y - armBase.y);
    if (grid.slackOf[cell] >= CARRY_SLACK_M && distance <= reach) found.push({ ...p, distance });
  }
  // Best first: the points nearest to 0.8 of the reach. For a point close to its base an arm folds up tight, and
  // at carrying height that runs into joint limits: the mock arm could not hold the robot up 0.17 m from its
  // mount, and did at 0.25 m. The very end of the reach is no better, so the search aims a little short of it.
  found.sort((a, b) => Math.abs(a.distance - PREFERRED_REACH * reach) - Math.abs(b.distance - PREFERRED_REACH * reach));
  // Spread the choices out: candidates 2 cm apart would all fail for the same reason.
  const chosen = [];
  for (const p of found) {
    if (chosen.every((q) => Math.hypot(p.x - q.x, p.y - q.y) >= 0.06)) chosen.push({ x: p.x, y: p.y });
    if (chosen.length >= count) break;
  }
  return chosen;
}

// Where the robot should walk so that the arm can pick it up: the free cell it can walk to that is PICKUP_INSET_M
// inside the arm's reach and nearest to where it stands. null when no such cell exists on its side of the obstacle.
export const PICKUP_INSET_M = 0.03;
export function pickupTarget(robot, arena, obstacles, armBase, reach, robotRadius = ROBOT_RADIUS_M) {
  const grid = buildGrid(arena, obstacles, robotRadius);
  let startCell = grid.cellOf(robot);
  if (grid.blocked[startCell]) startCell = grid.nearestFree(robot);
  if (startCell < 0) return null;
  let best = null, bestDistance = Infinity;
  for (const cell of connected(grid, startCell)) {
    const p = grid.centre(cell);
    if (Math.hypot(p.x - armBase.x, p.y - armBase.y) > reach - PICKUP_INSET_M) continue;
    const distance = Math.hypot(p.x - robot.x, p.y - robot.y);
    if (distance < bestDistance) { best = p; bestDistance = distance; }
  }
  return best;
}

// A* over an 8-connected grid. Returns waypoints from start to goal, or null when no walkable path exists.
// A goal inside an obstacle (for example "go to the chocolate") is replaced by the nearest free cell.
export function planPath(start, goal, arena, obstacles, robotRadius = ROBOT_RADIUS_M) {
  const { cols, rows, centre, cellOf, blocked, cellCost, nearestFree, neighbours } = buildGrid(arena, obstacles, robotRadius);
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
    for (const [next, step] of neighbours(current)) {
      const candidate = cost[current] + step * (cellCost[current] + cellCost[next]) / 2;
      if (candidate < cost[next]) {
        cost[next] = candidate;
        from[next] = current;
        open.add(next);
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
  // The goal was in an unreachable spot (against a wall or inside an obstacle's clearance), so the path ends at the nearest reachable one.
  path.goalMoved = goalMoved;
  return path;
}
