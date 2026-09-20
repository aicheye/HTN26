import { POSES, type Obstacle, type Point, type PoseName, type WorldState } from "../types/world";
import { distanceTo } from "../components/mapShared";
import type { ConnectionStatus } from "../sources/StateSource";

export const VOICE_PULSE_MS = 500;
export const VOICE_MAX_AGE_MS = 2000;
export type VoiceIntent =
  | { type: "forward" | "backward"; durationMs: number; distanceCm?: number }
  | { type: "left" | "right"; durationMs: number; angleDeg?: number }
  | { type: "stop" }
  | { type: "pose"; pose: PoseName }
  | { type: "goto"; name: string; relation?: GotoRelation };
export type GotoRelation = "at" | "past";
export const MAX_PLAN_STEPS = 4;
export const MAX_WALK_CM = 60;
export const MIN_TURN_DEG = 5;
export const MAX_TURN_DEG = 360;
const EDGE_MARGIN_M = 0.07; // matches the bridge planner: the robot's centre cannot get closer to a wall than this
export type Landmark = { name: string } & (
  | { kind: "point"; point: Point }
  | { kind: "obstacle"; obstacleId: string }
);
/**
 * The four table corners, named as they appear on the map and in the default 3D view: x grows to the right and y grows
 * away from the viewer, so "top" is the far side and "bottom" the near side.
 */
export const CORNERS = {
  "corner top left": { right: false, top: true, label: "top left" },
  "corner top right": { right: true, top: true, label: "top right" },
  "corner bottom left": { right: false, top: false, label: "bottom left" },
  "corner bottom right": { right: true, top: false, label: "bottom right" },
} as const;
export type TargetResult = { target: Point; error?: never } | { error: string; target?: never };

const UNSUPPORTED = /\b(?:not|no|never|dont|don't|cannot|can't|then|and|or|after|before|until|unless|if)\b/;

export function normalizeVoiceName(name: string): string {
  return name.toLowerCase().trim().replace(/[-_\s]+/g, " ");
}

export function parseVoiceCommand(transcript: string): VoiceIntent | null {
  let text = transcript.toLowerCase().replace(/’/g, "'").trim().replace(/[.!]+$/, "").trim();
  if (!text || UNSUPPORTED.test(text)) return null;
  text = text.replace(/^sesame[,\s]+/, "").replace(/^please\s+/, "").replace(/\s+please$/, "").trim();
  if (/^(?:stop|halt|freeze|emergency stop)$/.test(text)) return { type: "stop" };
  if (/^(?:(?:go|move) )?forwards?$/.test(text)) return { type: "forward", durationMs: VOICE_PULSE_MS };
  if (/^(?:(?:go|move) )?backwards?$|^back up$|^reverse$/.test(text)) return { type: "backward", durationMs: VOICE_PULSE_MS };
  const turn = text.match(/^(?:turn )?(left|right)$/);
  if (turn) return { type: turn[1] as "left" | "right", durationMs: VOICE_PULSE_MS };
  const pose = text.replace(/^(?:do (?:a )?|pose )/, "");
  if (POSES.includes(pose as PoseName)) return { type: "pose", pose: pose as PoseName };
  const past = text.match(/^(?:go|move|get|walk) (?:past|beyond|over|across|to the other side of) (?:the )?([a-z0-9]+(?:[ -][a-z0-9]+)*)$/)?.[1];
  if (past && past.length <= 40) return { type: "goto", name: normalizeVoiceName(past), relation: "past" };
  const destination = text.match(/^(?:go to|navigate to) ([a-z0-9]+(?:[ -][a-z0-9]+)*)$/)?.[1];
  if (destination && destination.length <= 40) return { type: "goto", name: normalizeVoiceName(destination) };
  return null;
}

export function validateLandmarkName(name: string, existing: readonly { name: string }[]): string | null {
  const normalized = normalizeVoiceName(name);
  const parsed = parseVoiceCommand(`go to ${normalized}`);
  if (!parsed || parsed.type !== "goto" || parsed.name !== normalized) return "Use 1–40 letters, numbers or spaces, without command words such as ‘then’.";
  return existing.some((item) => normalizeVoiceName(item.name) === normalized) ? "That name already exists. Choose a unique name." : null;
}

/** With `armAssist`, the arm carrying this robot over an obstacle is part of the navigation, not a reason to abort it. */
export function voiceReadiness(state: WorldState | null, robotId: string | null, status: ConnectionStatus, receivedAt: number, now = Date.now(), armAssist = false): string | null {
  if (status !== "live") return "Connect to a live data source before sending voice actions.";
  if (!state || !Number.isFinite(receivedAt) || now - receivedAt > VOICE_MAX_AGE_MS || now < receivedAt) return "Waiting for a fresh world frame.";
  if (state.calibration?.ok === false) return "Arena calibration is not ready.";
  const robot = state.robots.find((r) => r.id === robotId);
  if (!robot?.tracking || robot.mode === "lost") return "The selected robot is not currently tracked.";
  if (![robot.x, robot.y, robot.footprint.width, robot.footprint.length].every(Number.isFinite) || robot.footprint.width <= 0 || robot.footprint.length <= 0) return "Robot position or footprint is invalid.";
  const armOwned = armAssist && (!state.arm?.targetRobotId || state.arm.targetRobotId === robotId);
  if (state.arm && state.arm.mode !== "idle" && !armOwned) return "Wait for the arm to finish before sending a voice action.";
  return null;
}

function obstacleRadius(obstacle: Obstacle): number {
  if (![obstacle.x, obstacle.y, obstacle.yaw].every(Number.isFinite)) return NaN;
  if (obstacle.shape === "circle") return (obstacle.radius ?? 0) > 0 ? obstacle.radius! : NaN;
  if (obstacle.shape === "rect") return (obstacle.width ?? 0) > 0 && (obstacle.length ?? 0) > 0 ? Math.hypot(obstacle.width!, obstacle.length!) / 2 : NaN;
  if (!obstacle.points || obstacle.points.length < 3 || !obstacle.points.every((p) => Number.isFinite(p.x) && Number.isFinite(p.y))) return NaN;
  return Math.max(...obstacle.points.map((p) => Math.hypot(p.x - obstacle.x, p.y - obstacle.y)));
}

export function resolveVoiceTarget(name: string, landmarks: readonly Landmark[], state: WorldState, robotId: string, relation: GotoRelation = "at"): TargetResult {
  const key = normalizeVoiceName(name);
  const matches: Landmark[] = [
    ...landmarks.filter((item) => normalizeVoiceName(item.name) === key),
    ...state.obstacles.filter((o) => normalizeVoiceName(o.id) === key).map((o) => ({ name: o.id, kind: "obstacle" as const, obstacleId: o.id })),
  ];
  const corner = matches.length ? undefined : (CORNERS as Record<string, (typeof CORNERS)[keyof typeof CORNERS]>)[key];
  if (!matches.length && !corner) return { error: `Unknown destination: ${name}. That object is not currently detected.` };
  if (matches.length > 1) return { error: `Ambiguous destination: ${name}. Use a unique name.` };
  const robot = state.robots.find((r) => r.id === robotId);
  if (!robot?.tracking) return { error: "The selected robot is not tracked." };
  const clearance = Math.hypot(robot.footprint.width, robot.footprint.length) / 2 + 0.03;
  if (![clearance, state.arena.width, state.arena.length].every(Number.isFinite) || clearance <= 0.03 || state.obstacles.some((o) => !Number.isFinite(obstacleRadius(o)))) return { error: "Cannot verify clearance with incomplete scene geometry." };
  const isClear = (p: Point) => Number.isFinite(p.x) && Number.isFinite(p.y)
    && p.x >= clearance && p.y >= clearance && p.x <= state.arena.width - clearance && p.y <= state.arena.length - clearance
    && state.obstacles.every((o) => distanceTo(o, p) >= clearance)
    && state.robots.every((r) => r.id === robotId || !r.tracking || Math.hypot(p.x - r.x, p.y - r.y) >= clearance + Math.hypot(r.footprint.width, r.footprint.length) / 2);
  if (corner) {
    const inset = clearance + 0.001;
    const point = { x: corner.right ? state.arena.width - inset : inset, y: corner.top ? state.arena.length - inset : inset };
    return isClear(point) ? { target: point } : { error: `The ${corner.label} corner is blocked by an object or the other robot.` };
  }
  const match = matches[0];
  if (match.kind === "point") return isClear(match.point) ? { target: { ...match.point } } : { error: "That waypoint is blocked or lacks clearance from the arena edge." };
  const obstacles = state.obstacles.filter((o) => o.id === match.obstacleId);
  if (obstacles.length !== 1) return { error: "That obstacle is no longer uniquely visible. Rename a current obstacle." };
  const obstacle = obstacles[0];
  const radius = obstacleRadius(obstacle) + clearance + 0.01;
  const near = Math.atan2(robot.y - obstacle.y, robot.x - obstacle.x);
  // "past" lands on the far side: the ring point opposite the robot, or the clear point closest to it.
  const far = near + Math.PI;
  const candidates = Array.from({ length: 32 }, (_, i) => ({
    x: obstacle.x + Math.cos(near + i * Math.PI / 16) * radius,
    y: obstacle.y + Math.sin(near + i * Math.PI / 16) * radius,
    angle: near + i * Math.PI / 16,
  })).filter(isClear);
  const offFar = (angle: number) => Math.abs(Math.atan2(Math.sin(angle - far), Math.cos(angle - far)));
  candidates.sort(relation === "past" ? (a, b) => offFar(a.angle) - offFar(b.angle)
    : (a, b) => Math.hypot(a.x - robot.x, a.y - robot.y) - Math.hypot(b.x - robot.x, b.y - robot.y));
  if (!candidates.length) return { error: `No clear ${relation === "past" ? "point past" : "approach point beside"} that obstacle.` };
  if (relation === "past" && offFar(candidates[0].angle) > Math.PI / 2) return { error: "There is no clear space on the far side of that obstacle." };
  return { target: { x: candidates[0].x, y: candidates[0].y } };
}

/** Checks a measured walk before it starts, from the robot's pose at that moment. Turns are always safe in place. */
export function checkVoiceWalk(intent: VoiceIntent, state: WorldState, robotId: string): string | null {
  if ((intent.type !== "forward" && intent.type !== "backward") || !intent.distanceCm) return null;
  const robot = state.robots.find((r) => r.id === robotId);
  if (!robot) return "The selected robot is not tracked.";
  const heading = robot.yaw + (intent.type === "backward" ? Math.PI : 0);
  const end = { x: robot.x + Math.cos(heading) * intent.distanceCm / 100, y: robot.y + Math.sin(heading) * intent.distanceCm / 100 };
  if (end.x < EDGE_MARGIN_M || end.y < EDGE_MARGIN_M || end.x > state.arena.width - EDGE_MARGIN_M || end.y > state.arena.length - EDGE_MARGIN_M) {
    return `Walking ${intent.distanceCm} cm ${intent.type} would take the robot off the table. Try a shorter distance or turn first.`;
  }
  const clearance = Math.hypot(robot.footprint.width, robot.footprint.length) / 2 + 0.03;
  for (const obstacle of state.obstacles) {
    const closest = distanceTo(obstacle, robot);
    if (!Number.isFinite(closest)) return "Cannot verify the path with incomplete scene geometry.";
    // Only refuse when the walk gets closer than the clearance, so a robot already beside an object can still back away.
    const steps = Math.ceil(intent.distanceCm / 2);
    for (let i = 1; i <= steps; i++) {
      const p = { x: robot.x + (end.x - robot.x) * i / steps, y: robot.y + (end.y - robot.y) * i / steps };
      if (distanceTo(obstacle, p) < Math.min(clearance, closest) - 0.005) return `That walk would run into ${obstacle.id}.`;
    }
  }
  return null;
}
