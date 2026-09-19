/**
 * Wire schema shared between the camera/backend and this frontend.
 * Coordinates: meters, origin at the arena's bottom-left corner,
 * +x right, +y forward/up-screen, yaw in radians CCW from +x.
 * See docs/HANDOFF.md.
 */

export const SCHEMA_VERSION = 1 as const;

export type Point = { x: number; y: number };

export type RobotMode = "idle" | "moving" | "turning" | "lost";

/** One-shot animations supported by the Sesame firmware (movement-sequences.h). */
export const POSES = [
  "rest",
  "stand",
  "wave",
  "dance",
  "swim",
  "point",
  "pushup",
  "bow",
  "cute",
  "freaky",
  "worm",
  "shake",
  "shrug",
  "dead",
  "crab",
] as const;
export type PoseName = (typeof POSES)[number];

/** OLED faces exposed by the firmware. Talk variants are `talk_<emotion>`. */
export const FACES = [
  "default",
  "idle",
  "walk",
  "rest",
  "stand",
  "dance",
  "wave",
  "happy",
  "sad",
  "angry",
  "surprised",
  "sleepy",
  "love",
  "excited",
  "confused",
  "thinking",
] as const;
export type FaceName = (typeof FACES)[number] | (string & {});

export type Robot = {
  id: string; // app label, e.g. "sesame-1"
  tagId: number; // AprilTag number on the robot
  x: number;
  y: number; // center, meters
  yaw: number; // radians
  footprint: { width: number; length: number };
  tracking: boolean; // false = tag currently not detected
  lastSeen: number; // ms since epoch of last detection
  confidence?: number; // 0 to 1
  mode: RobotMode;
  face?: FaceName; // mirrors /api/status currentFace
  pose?: PoseName; // one-shot animation currently playing
};

export type Obstacle = {
  id: string; // app label, e.g. "obstacle-2"
  source: "tag" | "manual" | "cv";
  shape: "rect" | "circle" | "polygon";
  x: number;
  y: number; // center, meters
  yaw: number; // radians (0 for circles)
  width?: number;
  length?: number; // rect
  radius?: number; // circle
  points?: Point[]; // polygon, world coords — traced contour for cv detections
  height?: number; // vertical size, 3D only
  tagId?: number; // only if source = "tag"
  confidence?: number; // 0 to 1, detector score
};

export type WorldState = {
  schemaVersion: 1;
  seq: number; // increments every frame
  timestamp: number; // capture time, ms since epoch

  arena: {
    width: number; // x extent, meters
    length: number; // y extent, meters
    cornerTagIds?: number[]; // fixed calibration tags
  };
  calibration?: { ok: boolean; reprojectionError?: number };
  cameraFeedUrl?: string; // optional MJPEG/video for the toggle layer

  robots: Robot[];
  obstacles: Obstacle[];
  goal?: Point; // where the user clicked
  path?: Point[]; // planner output
};

export type CommandType =
  | "forward"
  | "backward"
  | "left"
  | "right"
  | "stop"
  | "goto"
  | "pose"
  | "face";

export type Command = {
  id: string; // unique, used to match the ack
  ts: number;
  robotId: string;
  type: CommandType;
  speed?: number; // 0 to 1
  durationMs?: number;
  target?: Point; // required when type = "goto"
  pose?: PoseName; // required when type = "pose"
  face?: FaceName; // required when type = "face", optional on any other command
};

export type Ack = { commandId: string; ok: boolean; error?: string };

/** Every WebSocket message uses this wrapper. */
export type Envelope =
  | { type: "state"; data: WorldState } // backend -> frontend
  | { type: "command"; data: Command } // frontend -> backend
  | { type: "ack"; data: Ack }; // backend -> frontend
