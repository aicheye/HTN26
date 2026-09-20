import assert from "node:assert/strict";
import { CLEARANCE_M, EDGE_MARGIN_M, distanceToObstacle, planPath } from "./planner.mjs";

const arena = { width: 0.76, length: 0.6 };
const wall = { shape: "rect", x: 0.38, y: 0.2, yaw: 0, width: 0.04, length: 0.4 };
const clearOf = (path, obstacles) => {
  for (let i = 0; i < path.length - 1; i++) {
    for (let k = 0; k <= 20; k++) {
      const p = { x: path[i].x + ((path[i + 1].x - path[i].x) * k) / 20, y: path[i].y + ((path[i + 1].y - path[i].y) * k) / 20 };
      for (const o of obstacles) assert.ok(distanceToObstacle(p, o) > CLEARANCE_M - 0.03, `path passes ${distanceToObstacle(p, o).toFixed(3)} m from an obstacle`);
    }
  }
};

let path = planPath({ x: 0.1, y: 0.1 }, { x: 0.6, y: 0.5 }, arena, []);
assert.equal(path.length, 2);
console.log("PASS  empty arena: straight line");

path = planPath({ x: 0.1, y: 0.1 }, { x: 0.66, y: 0.1 }, arena, [wall]);
assert.ok(path.length > 2 && Math.max(...path.map((p) => p.y)) > 0.4);
clearOf(path, [wall]);
console.log(`PASS  wall with a gap: goes around in ${path.length} waypoints`);

const fullWall = { ...wall, y: 0.3, length: 0.6 };
assert.equal(planPath({ x: 0.1, y: 0.1 }, { x: 0.66, y: 0.1 }, arena, [fullWall]), null);
console.log("PASS  wall across the whole arena: no path");

const chocolate = { shape: "rect", x: 0.6, y: 0.4, yaw: 0, width: 0.06, length: 0.06 };
path = planPath({ x: 0.1, y: 0.1 }, { x: 0.6, y: 0.4 }, arena, [chocolate]);
const end = path.at(-1), gap = distanceToObstacle(end, chocolate);
assert.ok(gap >= CLEARANCE_M - 0.02 && gap < CLEARANCE_M + 0.04, `stops ${gap.toFixed(3)} m from the object`);
console.log("PASS  goal inside an object: stops next to it");

const tilted = { shape: "rect", x: 0.38, y: 0.3, yaw: Math.PI / 4, width: 0.3, length: 0.04 };
const polygon = { shape: "polygon", x: 0.2, y: 0.4, yaw: 0, points: [{ x: 0.15, y: 0.35 }, { x: 0.25, y: 0.35 }, { x: 0.2, y: 0.45 }] };
// wall margin plus obstacle clearance leaves no gap in the small arena, so this scene uses a larger one
path = planPath({ x: 0.1, y: 0.1 }, { x: 0.9, y: 0.7 }, { width: 1, length: 0.8 }, [tilted, polygon, { shape: "circle", x: 0.6, y: 0.3, yaw: 0, radius: 0.05 }]);
clearOf(path, [tilted, polygon]);
console.log("PASS  rotated rect, polygon and circle are all avoided");

const nearWall = planPath({ x: 0.3, y: 0.3 }, { x: 0.755, y: 0.3 }, arena, []);
assert.ok(nearWall.goalMoved && nearWall.at(-1).x <= arena.width - EDGE_MARGIN_M + 0.02, "goal against a wall is pulled back to reachable floor");
assert.equal(planPath({ x: 0.3, y: 0.3 }, { x: 0.5, y: 0.3 }, arena, []).goalMoved, false);
console.log("PASS  goal in the wall margin: moved to the nearest reachable spot");

const escape = planPath({ x: 0.03, y: 0.3 }, { x: 0.5, y: 0.3 }, arena, []);
assert.ok(escape && escape.at(-1).x === 0.5, "a robot already touching the wall can still leave");
console.log("PASS  robot starting inside the wall margin can plan out of it");
