import type { Point, Robot } from "../types/world";

/** Where the robot has been, for the trail both maps draw. A point is added for every centimetre it moves. Kept
 *  outside React state: it changes on every frame, and the maps redraw on every frame anyway. */
const MAX_POINTS = 800;
const STEP_M = 0.01;
const points: Point[] = [];
let enabled = true;
try { enabled = localStorage.getItem("trail") !== "off"; } catch { /* no storage: keep the default */ }

export function recordTrail(robot: Robot | undefined) {
  if (!robot?.tracking) return;
  const last = points[points.length - 1];
  if (last && Math.hypot(robot.x - last.x, robot.y - last.y) < STEP_M) return;
  points.push({ x: robot.x, y: robot.y });
  if (points.length > MAX_POINTS) points.shift();
}

export const trail = (): readonly Point[] => (enabled ? points : []);
export const trailEnabled = () => enabled;
export function clearTrail() { points.length = 0; }
export function setTrailEnabled(on: boolean) {
  enabled = on;
  try { localStorage.setItem("trail", on ? "on" : "off"); } catch { /* no storage */ }
}
