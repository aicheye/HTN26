import type { Obstacle, Point, Robot } from "../types/world";

export const FLOOR = "#e2e8f0";
export const FLOOR_EDGE = "#cbd5e1";
export const GRID = "#ffffff";
export const OBSTACLE = "#475569";
export const OBSTACLE_DANGER = "#b91c1c";
export const CHASSIS = "#111111";
export const CHASSIS_EDGE = "#000000";
export const GOAL = "#ef4444";
export const SHADOW = "rgba(15, 23, 42, 0.3)";

export const TILE_M = 0.3; // floor tile size in meters

/** Proximity at which an obstacle is flagged as a danger, in meters. */
export const DANGER_M = 0.12;

/** Body and leg proportions derived from the reported footprint. */
export function robotDims(footprint: Robot["footprint"]) {
  const L = footprint.length;
  const W = footprint.width;
  const bodyL = L * 0.84;
  const bodyW = W * 0.58;
  const legL = L * 0.24;
  return {
    L,
    W,
    bodyL,
    bodyW,
    legL,
    hipX: bodyL * 0.3,
    legInner: bodyW * 0.3,
    legSpan: W / 2 - bodyW * 0.3,
  };
}

/** Hip positions and gait phase offsets for a diagonal-pair trot. */
export const LEG_LAYOUT: [front: 1 | -1, left: 1 | -1, phase: number][] = [
  [1, -1, 0],
  [1, 1, Math.PI],
  [-1, -1, Math.PI],
  [-1, 1, 0],
];

export const GAIT_RATE = 9; // radians per second
export const GAIT_STRIDE = 0.45; // fraction of leg length

export function isWalking(mode: Robot["mode"]) {
  return mode === "moving" || mode === "turning";
}

/** Rough distance from a point to an obstacle, used only for the danger highlight. */
export function distanceTo(o: Obstacle, p: Point): number {
  if (o.shape === "circle") {
    return Math.max(0, Math.hypot(p.x - o.x, p.y - o.y) - (o.radius ?? 0));
  }
  if (o.shape === "polygon" && o.points?.length) {
    return Math.max(
      0,
      Math.min(...o.points.map((q) => Math.hypot(p.x - q.x, p.y - q.y))) - 0.05,
    );
  }
  const dx = p.x - o.x;
  const dy = p.y - o.y;
  const c = Math.cos(-o.yaw);
  const s = Math.sin(-o.yaw);
  const lx = Math.abs(dx * c - dy * s) - (o.width ?? 0) / 2;
  const ly = Math.abs(dx * s + dy * c) - (o.length ?? 0) / 2;
  return Math.hypot(Math.max(lx, 0), Math.max(ly, 0));
}

export function isDanger(o: Obstacle, robots: Robot[]): boolean {
  const tracked = robots.find((r) => r.tracking);
  return tracked ? distanceTo(o, tracked) < DANGER_M : false;
}
