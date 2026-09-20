// Types for navigator.mjs, so that the web UI's Mock source can run the same navigator the bridge does.
type Point = { x: number; y: number };
type Pose = Point & { yaw: number; tracking: boolean };
type Arena = { width: number; length: number };
export type NavigatorStatus = {
  state: "idle" | "navigating" | "recovering" | "calibrating" | "carrying" | "done" | "failed";
  detail: string; recoveries: number; motion: Record<string, number>; command: string; waypoints: number; carries: number; via?: Point;
};
export class Navigator {
  constructor(send: (command: string) => void, motion?: Record<string, number>);
  state: NavigatorStatus["state"];
  detail: string;
  goal: Point | null;
  path: Point[];
  via: Point | null;
  robotRadius: number | undefined;
  arm: { base: Point; reach: number } | null;
  requestCarry: ((goal: Point, drops: Point[]) => boolean) | null;
  status(): NavigatorStatus;
  start(goal: Point): void;
  cancel(): void;
  carried(ok: boolean, reason?: string): void;
  step(robot: Pose | undefined, arena: Arena, obstacles: unknown[], now?: number): void;
}
