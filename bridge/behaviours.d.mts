// Types for behaviours.mjs, so that the web UI's Mock source can run the same behaviours the bridge does.
type Point = { x: number; y: number };
type Hooks = { goto: (target: Point) => void; face: (name: string) => void; pose: (name: string) => void; ignore?: (obstacle: any) => boolean };
export type BehaviourStatus = {
  moods: boolean; curious: boolean; diary: { at: number; text: string; kind: "table" | "robot" }[];
  tour: { visited: number; total: number } | null; errand: { label: string; kind: "curious" | "tour" } | null;
};
export class Behaviours {
  constructor(hooks: Hooks, options?: { moods?: boolean; curious?: boolean; diary?: boolean });
  status(): BehaviourStatus;
  configure(options: { moods?: boolean; curious?: boolean; diary?: boolean }): void;
  startTour(objects: (Point & { id: string })[], robot: Point, now: number): boolean;
  interrupt(): void;
  step(world: { robot: (Point & { tracking: boolean }) | undefined; obstacles: unknown[]; mission: { state: string; detail?: string } }, now?: number): void;
}
export function isArmDetection(obstacle: unknown, armBase: Point | null, radius?: number): boolean;
