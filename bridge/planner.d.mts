// Types for planner.mjs, so that the web UI's Mock source can run the same planner the bridge does.
type Point = { x: number; y: number };
type Arena = { width: number; length: number };
type Obstacle = { shape: string; x: number; y: number; yaw: number; width?: number; length?: number; radius?: number; points?: Point[] };
export const CELL_M: number;
export const ROBOT_RADIUS_M: number;
export const CLEARANCE_M: number;
export const EDGE_MARGIN_M: number;
export const CARRY_SLACK_M: number;
export const PICKUP_INSET_M: number;
export function distanceToObstacle(p: Point, obstacle: Obstacle): number;
export function leavesArena(robot: Point & { yaw: number }, arena: Arena, command: string): boolean;
export function costmap(arena: Arena, obstacles: Obstacle[], robotRadius?: number):
  { cell: number; cols: number; rows: number; clearance: number; edgeMargin: number; cost: number[] };
export function carryTargets(goal: Point, arena: Arena, obstacles: Obstacle[], armBase: Point, reach: number, robotRadius?: number, count?: number): Point[];
export function pickupTarget(robot: Point, arena: Arena, obstacles: Obstacle[], armBase: Point, reach: number, robotRadius?: number): Point | null;
export function planPath(start: Point, goal: Point, arena: Arena, obstacles: Obstacle[], robotRadius?: number): (Point[] & { goalMoved?: boolean }) | null;
