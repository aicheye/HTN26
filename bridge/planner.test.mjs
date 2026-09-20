import assert from "node:assert/strict";
import { CARRY_SLACK_M, CLEARANCE_M, EDGE_MARGIN_M, carryTargets, distanceToObstacle, leavesArena, planPath } from "./planner.mjs";

const arena = { width: 0.76, length: 0.6 };
// Scenes keep the robot out of the closed 0.122 m strip along the edge, so its centre lives in x 0.122 to 0.638
// and y 0.122 to 0.478 of this arena.
const wall = { shape: "rect", x: 0.38, y: 0.15, yaw: 0, width: 0.04, length: 0.26 };
const clearOf = (path, obstacles) => {
  for (let i = 0; i < path.length - 1; i++) {
    for (let k = 0; k <= 20; k++) {
      const p = { x: path[i].x + ((path[i + 1].x - path[i].x) * k) / 20, y: path[i].y + ((path[i + 1].y - path[i].y) * k) / 20 };
      for (const o of obstacles) assert.ok(distanceToObstacle(p, o) > CLEARANCE_M - 0.03, `path passes ${distanceToObstacle(p, o).toFixed(3)} m from an obstacle`);
    }
  }
};

let path = planPath({ x: 0.15, y: 0.15 }, { x: 0.6, y: 0.45 }, arena, []);
assert.equal(path.length, 2);
console.log("PASS  empty arena: straight line");

path = planPath({ x: 0.15, y: 0.15 }, { x: 0.62, y: 0.15 }, arena, [wall]);
assert.ok(path.length > 2 && Math.max(...path.map((p) => p.y)) > 0.37);
clearOf(path, [wall]);
console.log(`PASS  wall with a gap: goes around in ${path.length} waypoints`);

const fullWall = { ...wall, y: 0.3, length: 0.6 };
assert.equal(planPath({ x: 0.15, y: 0.15 }, { x: 0.62, y: 0.15 }, arena, [fullWall]), null);
console.log("PASS  wall across the whole arena: no path");

const chocolate = { shape: "rect", x: 0.5, y: 0.35, yaw: 0, width: 0.06, length: 0.06 };
path = planPath({ x: 0.15, y: 0.15 }, { x: 0.5, y: 0.35 }, arena, [chocolate]);
const end = path.at(-1), gap = distanceToObstacle(end, chocolate);
assert.ok(gap >= CLEARANCE_M - 0.02 && gap < CLEARANCE_M + 0.04, `stops ${gap.toFixed(3)} m from the object`);
console.log("PASS  goal inside an object: stops next to it");

// One of each shape. With the closed strip and the clearance, the robot's centre has little room in this arena:
// the three are small and apart so that a way through exists.
const tilted = { shape: "rect", x: 0.38, y: 0.2, yaw: Math.PI / 4, width: 0.1, length: 0.03 };
const polygon = { shape: "polygon", x: 0.62, y: 0.47, yaw: 0, points: [{ x: 0.6, y: 0.45 }, { x: 0.66, y: 0.45 }, { x: 0.63, y: 0.5 }] };
const circle = { shape: "circle", x: 0.14, y: 0.14, yaw: 0, radius: 0.02 };
path = planPath({ x: 0.2, y: 0.3 }, { x: 0.6, y: 0.2 }, arena, [tilted, polygon, circle]);
assert.ok(path.length > 2, "the rotated box is in the straight line");
clearOf(path, [tilted, polygon, circle]);
console.log("PASS  rotated rect, polygon and circle are all avoided");

const along = (path, check) => {
  for (let i = 0; i < path.length - 1; i++) {
    for (let k = 0; k <= 20; k++) check({ x: path[i].x + ((path[i + 1].x - path[i].x) * k) / 20, y: path[i].y + ((path[i + 1].y - path[i].y) * k) / 20 });
  }
};

// A box in the middle of the bottom half. The short way round is below it, along the table's edge.
const square = { width: 0.63, length: 0.63 };
const low = { shape: "rect", x: 0.315, y: 0.2, yaw: 0, width: 0.06, length: 0.06 };
path = planPath({ x: 0.13, y: 0.2 }, { x: 0.5, y: 0.2 }, square, [low]);
along(path, (p) => assert.ok(Math.min(p.x, square.width - p.x, p.y, square.length - p.y) >= EDGE_MARGIN_M - 0.02, `path is ${p.y.toFixed(3)} m from the edge`));
clearOf(path, [low]);
assert.ok(Math.max(...path.map((p) => p.y)) > 0.3, "goes round the far side, where there is room");
console.log("PASS  table edge: the path keeps the robot on the table and takes the roomy side");

// Two boxes with a 0.3 m gap. A shortest path would run along one box's clearance limit.
const left = { shape: "rect", x: 0.15, y: 0.315, yaw: 0, width: 0.3, length: 0.06 }, right = { shape: "rect", x: 0.615, y: 0.315, yaw: 0, width: 0.03, length: 0.06 };
path = planPath({ x: 0.33, y: 0.1 }, { x: 0.57, y: 0.55 }, square, [left, right]);
let crossing = null;
along(path, (p) => { if (crossing === null && p.y >= 0.315) crossing = p.x; });
assert.ok(Math.abs(crossing - 0.45) < 0.06, `crosses the gap at x = ${crossing.toFixed(3)}, the middle is 0.45`);
console.log(`PASS  gap between two boxes: crosses at x = ${crossing.toFixed(2)}, near the middle (0.45)`);

// The robot stands 4 cm from a box, inside its clearance. It first steps away, then goes round.
const beside = { shape: "rect", x: 0.3, y: 0.3, yaw: 0, width: 0.1, length: 0.1 };
path = planPath({ x: 0.21, y: 0.3 }, { x: 0.5, y: 0.3 }, square, [beside]);
assert.ok(path && path.length >= 3, "a start inside the clearance still gets a path");
assert.ok(distanceToObstacle(path[1], beside) >= CLEARANCE_M - 0.02, "the first leg leads out of the clearance");
console.log(`PASS  start inside an obstacle's clearance: steps out first, ${path.length} waypoints`);

// The closed strip along the edge: 0.122 m for the robot's centre. Walking into it or deeper is refused, and
// walking out of it, along it, and turning are allowed.
assert.ok(Math.abs(EDGE_MARGIN_M - 0.122) < 1e-9);
const east = 0, west = Math.PI, north = Math.PI / 2;
assert.equal(leavesArena({ x: 0.5, y: 0.3, yaw: east }, square, "forward"), true, "0.5 + 0.02 crosses 0.63 - 0.122 = 0.508");
assert.equal(leavesArena({ x: 0.4, y: 0.3, yaw: east }, square, "forward"), false);
assert.equal(leavesArena({ x: 0.55, y: 0.3, yaw: east }, square, "forward"), true, "already inside the strip, going deeper");
assert.equal(leavesArena({ x: 0.55, y: 0.3, yaw: west }, square, "forward"), false, "walking out of the strip");
assert.equal(leavesArena({ x: 0.55, y: 0.3, yaw: west }, square, "backward"), true, "backing deeper into the strip");
assert.equal(leavesArena({ x: 0.55, y: 0.3, yaw: north }, square, "forward"), false, "walking along the strip");
assert.equal(leavesArena({ x: 0.55, y: 0.3, yaw: east }, square, "left"), false, "turning is always allowed");
for (const from of [{ x: 0.2, y: 0.2 }, { x: 0.03, y: 0.03 }, { x: 0.6, y: 0.31 }]) {
  path = planPath(from, { x: 0.62, y: 0.62 }, square, []);
  path.slice(from.x === 0.2 ? 0 : 1).forEach((p) => assert.ok(Math.min(p.x, square.width - p.x, p.y, square.length - p.y) >= EDGE_MARGIN_M - 0.011, `waypoint ${JSON.stringify(p)} is in the strip`));
}
console.log("PASS  closed strip along the edge: no waypoint in it, no walking into it, walking out allowed");

// From Angus's branch: the navigator reports when the goal had to be moved, and a robot at the wall can leave.
const nearWall = planPath({ x: 0.3, y: 0.3 }, { x: 0.755, y: 0.3 }, arena, []);
assert.ok(nearWall.goalMoved && nearWall.at(-1).x <= arena.width - EDGE_MARGIN_M + 0.02, "goal against a wall is pulled back to reachable floor");
assert.equal(planPath({ x: 0.3, y: 0.3 }, { x: 0.5, y: 0.3 }, arena, []).goalMoved, false);
console.log("PASS  goal in the closed strip: moved to the nearest reachable spot");

const escape = planPath({ x: 0.03, y: 0.3 }, { x: 0.5, y: 0.3 }, arena, []);
assert.ok(escape && escape.at(-1).x === 0.5, "a robot already touching the wall can still leave");
console.log("PASS  robot starting inside the closed strip can plan out of it");

// A wall across the whole arena, the robot on its left, the goal and the arm's base on its right. No path exists,
// so the arm is offered places to set the robot down: on the goal's side, clear of every limit, within its reach.
const barrier = { shape: "rect", x: 0.3, y: 0.3, yaw: 0, width: 0.04, length: 0.6 };
const armBase = { x: 0.7, y: 0.1 }, target = { x: 0.55, y: 0.35 };
assert.equal(planPath({ x: 0.15, y: 0.3 }, target, arena, [barrier]), null);
const drops = carryTargets(target, arena, [barrier], armBase, 0.3);
assert.ok(drops.length >= 2 && drops.length <= 5, `${drops.length} drop points`);
for (const drop of drops) {
  assert.ok(drop.x > 0.3 + 0.02 + CLEARANCE_M + CARRY_SLACK_M - 0.011, `drop ${JSON.stringify(drop)} is on the goal's side, clear of the wall`);
  assert.ok(Math.hypot(drop.x - armBase.x, drop.y - armBase.y) <= 0.3, "within the arm's reach");
  assert.ok(planPath(drop, target, arena, [barrier]), "the goal can be walked to from the drop point");
}
assert.ok(Math.hypot(drops[0].x - armBase.x, drops[0].y - armBase.y) <= Math.hypot(drops.at(-1).x - armBase.x, drops.at(-1).y - armBase.y), "nearest to the arm first");
assert.deepEqual(carryTargets({ x: 0.15, y: 0.3 }, arena, [barrier], armBase, 0.3), [], "nothing on the far side of the wall is within reach");
console.log(`PASS  fully blocked: ${drops.length} places for the arm to set the robot down, none when the goal's side is out of reach`);
