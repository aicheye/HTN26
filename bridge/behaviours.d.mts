// Types for behaviours.mjs, so that the web UI's Mock source can run the same behaviours the bridge does.
type Point = { x: number; y: number };
type Hooks = { goto: (target: Point) => void; ignore?: (obstacle: any) => boolean };
export type BehaviourStatus = {
  curious: boolean; diary: { at: number; text: string; kind: "table" | "robot" }[];
  errand: { label: string; looking: boolean } | null;
};
export class Behaviours {
  constructor(hooks: Hooks, options?: { curious?: boolean; diary?: boolean });
  status(): BehaviourStatus;
  configure(options: { curious?: boolean; diary?: boolean }): void;
  interrupt(): void;
  step(world: { robot: (Point & { tracking: boolean }) | undefined; obstacles: unknown[]; mission: { state: string; detail?: string; via?: Point } }, now?: number): void;
}
export function isArmDetection(obstacle: unknown, armBase: Point | null, radius?: number): boolean;
