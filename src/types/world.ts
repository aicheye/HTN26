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

export type SesameJoint = "R1" | "R2" | "L1" | "L2" | "R4" | "R3" | "L3" | "L4";

export type Robot = {
  joints?: Partial<Record<SesameJoint, number>>;
  jointSource?: "commanded" | "measured";
  shellColor?: string;
  z?: number;
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
  color?: string;
  colorSource?: "detector" | "camera";
  heightSource?: "measured" | "estimated";
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
  label?: string; // what the camera thinks it is, e.g. "green box"
  // Top-down photo of the object, transparent outside its outline. Image x runs along `width` (the yaw
  // direction) and the top row is the far side along `length`. The URL changes when the picture changes.
  textureUrl?: string;
};

/**
 * SO-101 6-DOF arm (waist, shoulder, elbow, wrist pitch, wrist roll, gripper),
 * bolted to one edge of the table. Angles are radians, matching the joint
 * conventions in the arm's URDF, so real hardware can publish these directly.
 */
export type ArmJointAngles = {
  waist: number; // joint 1, yaw about the vertical pedestal axis
  shoulder: number; // joint 2, pitch
  elbow: number; // joint 3, pitch
  wristPitch: number; // joint 4
  wristRoll: number; // joint 5
  gripper: number; // joint 6, 0 = closed .. ~1.2 = open
};

export type ArmMode =
  | "idle"
  | "reaching"
  | "grasping"
  | "lifting"
  | "carrying"
  | "placing"
  | "releasing"
  | "returning";

export type ArmState = {
  /** Fixed mount pose in arena coordinates; which table edge it's clamped to. */
  mount: { x: number; y: number; yaw: number; side: "north" | "south" | "east" | "west" };
  joints: ArmJointAngles;
  mode: ArmMode;
  targetRobotId?: string; // robot currently being assisted, if any
};

export type WorldState = {
  simulation?: {
    scenario: string;
    status: "ready" | "running" | "complete" | "blocked";
    message: string;
    testGoal: Point;
  };
  schemaVersion: 1;
  seq: number; // increments every frame
  timestamp: number; // capture time, ms since epoch

  arena: {
    surface?: "wood" | "grid";
    tagSize?: number;
    border?: number;
    width: number; // x extent, meters
    length: number; // y extent, meters
    cornerTagIds?: number[]; // fixed calibration tags
    // Set by the bridge when the strip of table that holds the corner tags is closed to the robot: the robot's
    // centre stays this far inside the arena. Both maps draw the strip and this limit.
    edgeMargin?: number;
  };
  calibration?: { ok: boolean; reprojectionError?: number };
  cameraFeedUrl?: string; // optional MJPEG/video for the toggle layer

  robots: Robot[];
  obstacles: Obstacle[];
  goal?: Point; // where the user clicked
  path?: Point[]; // planner output
  arm?: ArmState; // pick-and-place assist arm, if the rig has one
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
