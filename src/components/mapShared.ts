import type { ArmJointAngles, ArmState, Obstacle, Point, Robot } from "../types/world";
import { ARM_HOME, ARM_URDF_LIMITS } from "../robot/geometry";

/** True when the obstacle is drawn by its traced contour: polygons, and photographed objects, whose photo is
 *  transparent outside that contour. */
export function hasContour(o: Obstacle): boolean {
  return (o.shape === "polygon" || !!o.textureUrl) && (o.points?.length ?? 0) >= 3;
}

export function obstacleOutline(o: Obstacle): Point[] {
  let points: Point[];
  if (hasContour(o)) points = o.points!;
  else if (o.shape === "circle") points = Array.from({ length: 48 }, (_, i) => ({
    x: o.x + Math.cos(i * Math.PI / 24) * (o.radius ?? 0.1),
    y: o.y + Math.sin(i * Math.PI / 24) * (o.radius ?? 0.1),
  }));
  else points = [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([x, y]) => ({
    x: o.x + x * (o.width ?? 0.2) / 2 * Math.cos(o.yaw) - y * (o.length ?? 0.2) / 2 * Math.sin(o.yaw),
    y: o.y + x * (o.width ?? 0.2) / 2 * Math.sin(o.yaw) + y * (o.length ?? 0.2) / 2 * Math.cos(o.yaw),
  }));
  return [...points, points[0]];
}

export function obstacleColor(o: Obstacle): string {
  return /^#[0-9a-f]{6}$/i.test(o.color ?? "") ? o.color! : OBSTACLE;
}

export function obstacleHeight(o: Obstacle): number {
  if (Number.isFinite(o.height) && o.height! >= 0) return Math.max(o.height!, 0.001);
  // The camera does not measure height. A photographed object is drawn 0.4 times as tall as its short side,
  // between 1 and 4 cm, so that it stands out from the floor. The dashed outline still marks the height as a guess.
  if (o.textureUrl) return Math.min(0.04, Math.max(0.01, 0.4 * Math.min(o.width ?? 0.05, o.length ?? 0.05)));
  return 0.004;
}

export const FLOOR = "#e2e8f0";
export const FLOOR_EDGE = "#cbd5e1";
export const GRID = "#ffffff";
export const OBSTACLE = "#475569";
export const OBSTACLE_DANGER = "#b91c1c";
export const CHASSIS = "#111111";
export const CHASSIS_EDGE = "#000000";
export const ROBOT_FITTING = "#f1f5f9";
export const ROBOT_FITTING_INSET = "#bac6d1";
export const GOAL = "#ef4444";
export const SHADOW = "rgba(15, 23, 42, 0.3)";

export const TILE_M = 0.1; // floor tile size in meters

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
    let inside = false;
    let nearest = Infinity;
    for (let i = 0, j = o.points.length - 1; i < o.points.length; j = i++) {
      const a = o.points[j], b = o.points[i];
      const dx = b.x - a.x, dy = b.y - a.y;
      const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy || 1)));
      nearest = Math.min(nearest, Math.hypot(p.x - a.x - t * dx, p.y - a.y - t * dy));
      if ((a.y > p.y) !== (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    return inside ? 0 : nearest;
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
  pedestal: 0.06, // base mounting plate + motor stack height, up to the waist axis
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

/** Joint limits from the supplied SO-101 URDF, without widening the mechanical range. */
export const ARM_LIMITS: Record<keyof ArmJointAngles, [number, number]> = ARM_URDF_LIMITS;

/**
 * Illustrative stowed pose within the URDF limits, used only when no arm pose is supplied.
 * This is not a measurement of the photographed arm's joint angles.
 */
export const ARM_REST_POSE: ArmJointAngles = ARM_HOME;

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
