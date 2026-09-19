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
 * SO-101 arm geometry, in meters - joint-to-joint distances taken directly from
 * the arm's own URDF (onshape-to-robot export), not scaled up for the arena.
 */
export const ARM_LINK = {
  pedestal: 0.085, // base mounting plate + motor stack height, up to the waist axis
  plate: 0.01, // mounting plate thickness
  shoulderRise: 0.065, // waist axis -> shoulder pitch axis (the vertical riser)
  upperArm: 0.116, // shoulder joint -> elbow joint
  lowerArm: 0.135, // elbow joint -> wrist joint
  wrist: 0.064, // wrist joint -> gripper roll joint
  gripper: 0.07, // gripper body, roll joint -> jaw pivot
  jaw: 0.028, // finger length, visual only
};

/** Height of the shoulder pitch axis above the floor. */
export const ARM_MOUNT_HEIGHT =
  ARM_LINK.pedestal + ARM_LINK.plate + ARM_LINK.shoulderRise;

/**
 * Effective 2-link reach lengths used for IK. The wrist is held straight
 * (wristPitch 0) while reaching so this rigid L2 length is exactly where the
 * jaws end up, not just an approximation.
 */
export const ARM_REACH_L1 = ARM_LINK.upperArm;
export const ARM_REACH_L2 =
  ARM_LINK.lowerArm + ARM_LINK.wrist + ARM_LINK.gripper + ARM_LINK.jaw / 2;

/** Furthest floor point the gripper can touch, measured from the mount. */
export const ARM_MAX_REACH = ARM_REACH_L1 + ARM_REACH_L2 - 0.05;

/** Joint limits. Shoulder/elbow are widened past the URDF spec so the rest pose can fold a full 180 degrees. */
export const ARM_LIMITS: Record<keyof ArmJointAngles, [number, number]> = {
  waist: [-1.91986, 1.91986],
  shoulder: [-Math.PI, Math.PI],
  elbow: [-Math.PI, Math.PI],
  wristPitch: [-1.65806, 1.65806],
  wristRoll: [-2.74385, 2.84121],
  gripper: [-0.174533, 1.74533],
};

/**
 * Stowed pose: the upper arm lies horizontal pointing away from the arena,
 * then the elbow turns a full 180 degrees so the forearm lies horizontal
 * pointing back the other way, over the arena.
 */
export const ARM_REST_POSE: ArmJointAngles = {
  waist: 0,
  shoulder: Math.PI,
  elbow: Math.PI,
  wristPitch: 0,
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
 * for the horizontal distance + height via the law of cosines. wristPitch is
 * held at 0 so the wrist+gripper stay a rigid extension of the forearm -
 * that rigid length is exactly ARM_REACH_L2, so the jaws land on the target
 * instead of drifting off it the way a "leveling" wrist bend would.
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
  // the other elbow branch (positive, paired with a subtracted shoulder
  // offset below) makes the elbow arc up and over on the way to a target,
  // instead of dipping the whole upper arm down through the floor first
  const elbow = Math.PI - Math.acos(cosElbow);

  const cosShoulderOffset = clampRatio((L1 * L1 + d * d - L2 * L2) / (2 * L1 * d));
  // atan2 takes -height (not height): the render's dir() treats a negative
  // pitch as "up", the opposite of the usual math convention, so the sign
  // has to flip here or every reach would aim for the mirrored height.
  const shoulder = Math.atan2(-height, reachXY) - Math.acos(cosShoulderOffset);

  return { waist, shoulder, elbow, wristPitch: 0 };
}

/**
 * Forward kinematics for the gripper tip, mirroring ArmModel's chain exactly
 * (wrist+gripper continue straight past the forearm since wristRoll doesn't
 * change pointing direction). Used to find where the gripper currently is so
 * a reach can smoothly move it in a straight Cartesian line instead of
 * interpolating joint angles, which can swing the tip through the floor.
 */
export function armTipPosition(
  mount: ArmMount,
  joints: Pick<ArmJointAngles, "waist" | "shoulder" | "elbow" | "wristPitch">,
): Point & { z: number } {
  const { waist, shoulder, elbow, wristPitch } = joints;
  const forearmAbs = shoulder + elbow;
  const wristAbs = forearmAbs + wristPitch;
  const dir = (a: number): [number, number] => [Math.cos(a), -Math.sin(a)];

  let lx = 0;
  let lz = ARM_MOUNT_HEIGHT;
  const [dx1, dz1] = dir(shoulder);
  lx += ARM_LINK.upperArm * dx1;
  lz += ARM_LINK.upperArm * dz1;
  const [dx2, dz2] = dir(forearmAbs);
  lx += ARM_LINK.lowerArm * dx2;
  lz += ARM_LINK.lowerArm * dz2;
  const [dx3, dz3] = dir(wristAbs);
  const tail = ARM_LINK.wrist + ARM_LINK.gripper + ARM_LINK.jaw;
  lx += tail * dx3;
  lz += tail * dz3;

  const totalYaw = mount.yaw + waist;
  return {
    x: mount.x + lx * Math.cos(totalYaw),
    y: mount.y + lx * Math.sin(totalYaw),
    z: lz,
  };
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

/** How high the gripper hovers while carrying a robot over the map, meters. */
export const ARM_CARRY_LIFT_M = 0.28;

/** Transit height while reaching: swings over the arena before descending onto the target. */
export const ARM_TRANSIT_HEIGHT_M = 0.32;

const CARRY_MODES = new Set(["lifting", "carrying", "placing"]);

/** True while this robot is the one being lifted/carried/placed by the arm. */
export function isCarried(arm: ArmState | undefined, robotId: string): boolean {
  return !!arm && arm.targetRobotId === robotId && CARRY_MODES.has(arm.mode);
}

export function isDanger(o: Obstacle, robots: Robot[]): boolean {
  const tracked = robots.find((r) => r.tracking);
  return tracked ? distanceTo(o, tracked) < DANGER_M : false;
}
