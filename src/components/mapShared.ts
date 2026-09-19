import type { ArmJointAngles, ArmState, Obstacle, Point, Robot } from "../types/world";

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

/**
 * SO-101 arm geometry, in meters. Proportions follow the arm's own URDF joint
 * chain (onshape-to-robot export), scaled up from the real ~30 cm hobby arm so
 * it can service a useful chunk of the arena from one table-edge mount. It is
 * deliberately not arena-spanning: a folded rest pose only reads as "folded" if
 * the links are comparable to the robot it picks up.
 */
export const ARM_LINK = {
  pedestal: 0.1, // base plate + motor stack height, up to the waist axis
  plate: 0.018, // mounting plate thickness
  upperArm: 0.45, // shoulder joint -> elbow joint
  lowerArm: 0.42, // elbow joint -> wrist joint
  wrist: 0.15, // wrist joint -> gripper roll joint
  gripper: 0.1, // gripper body, roll joint -> jaw pivot
  jaw: 0.065, // finger length, visual only
};

/** Height of the shoulder/waist axis above the floor. */
export const ARM_MOUNT_HEIGHT = ARM_LINK.pedestal + ARM_LINK.plate;

/** Effective 2-link reach lengths used for IK: elbow bends once, wrist just levels the gripper. */
export const ARM_REACH_L1 = ARM_LINK.upperArm;
export const ARM_REACH_L2 = ARM_LINK.lowerArm + ARM_LINK.wrist + ARM_LINK.gripper;

/** Furthest floor point the gripper can touch, measured from the mount. */
export const ARM_MAX_REACH = ARM_REACH_L1 + ARM_REACH_L2 - 0.05;

/** Joint limits copied from the URDF <limit> tags, radians. */
export const ARM_LIMITS: Record<keyof ArmJointAngles, [number, number]> = {
  waist: [-1.91986, 1.91986],
  shoulder: [-1.74533, 1.74533],
  elbow: [-1.74533, 1.5708],
  wristPitch: [-1.65806, 1.65806],
  wristRoll: [-2.74385, 2.84121],
  gripper: [-0.174533, 1.74533],
};

/**
 * Stowed pose. The URDF elbow only travels +90/-100 degrees, so the forearm
 * can't lie parallel against the upper arm; the tightest available tuck folds
 * it back over the base and curls the wrist down so the gripper parks beside
 * the pedestal instead of reaching out across the table.
 */
export const ARM_REST_POSE: ArmJointAngles = {
  waist: 0,
  shoulder: -1.3,
  elbow: -1.74533,
  wristPitch: -1.65806,
  wristRoll: 0,
  gripper: 0.05,
};

export const ARM_COLOR = "#f97316";
export const ARM_JOINT_COLOR = "#292524";

export function clampArmJoint(name: keyof ArmJointAngles, value: number): number {
  const [lo, hi] = ARM_LIMITS[name];
  return Math.max(lo, Math.min(hi, value));
}

type ArmMount = ArmState["mount"];

/**
 * 2-link planar IK: waist yaws to face the target, then shoulder/elbow solve
 * for the horizontal distance + height via the law of cosines (elbow bends
 * once; wristPitch afterward just levels the gripper). Since ARM_REACH_L1/L2
 * cover the whole arena, this reaches any (x, y, z) the mock or a real
 * controller asks for.
 */
export function solveArmIK(
  mount: ArmMount,
  target: Point,
  targetZ: number,
): Pick<ArmJointAngles, "waist" | "shoulder" | "elbow" | "wristPitch"> {
  const dx = target.x - mount.x;
  const dy = target.y - mount.y;
  const reachXY = Math.hypot(dx, dy);
  const waist = wrapAngle(Math.atan2(dy, dx) - mount.yaw);

  const height = targetZ - ARM_MOUNT_HEIGHT;
  const L1 = ARM_REACH_L1;
  const L2 = ARM_REACH_L2;
  const raw = Math.hypot(reachXY, height);
  const d = Math.min(Math.max(raw, Math.abs(L1 - L2) + 0.01), L1 + L2 - 0.01);

  const cosElbow = clampRatio((L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2));
  const elbow = -(Math.PI - Math.acos(cosElbow));

  const cosShoulderOffset = clampRatio((L1 * L1 + d * d - L2 * L2) / (2 * L1 * d));
  const shoulder = Math.atan2(height, reachXY) + Math.acos(cosShoulderOffset);

  const wristPitch = -(shoulder + elbow);

  return { waist, shoulder, elbow, wristPitch };
}

function clampRatio(v: number) {
  return Math.max(-1, Math.min(1, v));
}

function wrapAngle(a: number) {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

/** Obstacles taller than this can't be stepped over and need the arm's help. */
export const ARM_ASSIST_HEIGHT_M = 0.05;

/** Whether a floor point is inside the arm's working envelope. */
export function isWithinArmReach(mount: ArmState["mount"], p: Point): boolean {
  return Math.hypot(p.x - mount.x, p.y - mount.y) <= ARM_MAX_REACH;
}

export function armTargetPoint(arm: ArmState, robots: Robot[]): Robot | undefined {
  return robots.find((r) => r.id === arm.targetRobotId);
}

/** How high the carried robot rises off the floor, meters (visual only). */
export const ARM_CARRY_LIFT_M = 0.06;

const CARRY_MODES = new Set(["lifting", "carrying", "placing"]);

/** True while this robot is the one being lifted/carried/placed by the arm. */
export function isCarried(arm: ArmState | undefined, robotId: string): boolean {
  return !!arm && arm.targetRobotId === robotId && CARRY_MODES.has(arm.mode);
}

export function isDanger(o: Obstacle, robots: Robot[]): boolean {
  const tracked = robots.find((r) => r.tracking);
  return tracked ? distanceTo(o, tracked) < DANGER_M : false;
}
