import { Euler, Matrix4, Quaternion, Vector3 } from "three";
import type { ArmJointAngles, ArmState, Robot, SesameJoint } from "../types/world";

export type Vec3 = [number, number, number];
export type Solid = { matrix: Matrix4; size: Vec3; color: string; name?: string } & (
  { kind: "box" | "sphere" | "capsule" | "cylinder" } | { kind: "prism"; outline: [number, number][] }
);
export const ARM_URDF_LIMITS: Record<keyof ArmJointAngles, [number, number]> = {
  waist: [-1.91986, 1.91986], shoulder: [-1.74533, 1.74533], elbow: [-1.74533, 1.5708],
  wristPitch: [-1.65806, 1.65806], wristRoll: [-2.74385, 2.84121], gripper: [-0.174533, 1.74533],
};
export const ARM_HOME: ArmJointAngles = { waist: 0, shoulder: -1.3, elbow: 1.4, wristPitch: 0, wristRoll: 0, gripper: 0.05 };
export const SESAME_HOME: Record<SesameJoint, number> = {
  R1: 3 * Math.PI / 4, R2: Math.PI / 4, L1: Math.PI / 4, L2: 3 * Math.PI / 4,
  R4: 0, R3: Math.PI, L3: 0, L4: Math.PI,
};

export function origin(xyz: Vec3, rpy: Vec3 = [0, 0, 0]): Matrix4 {
  return new Matrix4().compose(new Vector3(...xyz), new Quaternion().setFromEuler(new Euler(...rpy, "ZYX")), new Vector3(1, 1, 1));
}

function joint(parent: Matrix4, xyz: Vec3, rpy: Vec3, angle: number, sign = 1) {
  return parent.clone().multiply(origin(xyz, rpy)).multiply(new Matrix4().makeRotationZ(angle * sign));
}

function box(matrix: Matrix4, size: Vec3, color: string): Solid {
  return { kind: "box", matrix, size, color };
}

function ball(point: Vector3, radius: number, color: string): Solid {
  return { kind: "sphere", matrix: origin(point.toArray()), size: [radius, radius, radius], color };
}

function beam(a: Vector3, b: Vector3, radius: number, color: string): Solid {
  const delta = b.clone().sub(a);
  const matrix = new Matrix4().compose(a.clone().add(b).multiplyScalar(0.5),
    new Quaternion().setFromUnitVectors(new Vector3(0, 1, 0), delta.clone().normalize()), new Vector3(1, 1, 1));
  return { kind: "capsule", matrix, size: [radius, delta.length(), radius], color };
}

const CLAW_JAW_ORIGIN: Vec3 = [0.0202, 0.0188, -0.0234];
const CLAW_FIXED_PAD: Vec3 = [-0.0104, -0.0921274, 0];
const CLAW_MOVING_PAD: Vec3 = [-0.0182, -0.0687274, 0];
const CLAW_PAD_SIZE: Vec3 = [0.005, 0.012, 0.016];
const CLAW_MOVING_PLANE = 0.0188;

export function armFrames(joints: ArmJointAngles): Matrix4[] {
  const q = (name: keyof ArmJointAngles) => Math.max(ARM_URDF_LIMITS[name][0], Math.min(ARM_URDF_LIMITS[name][1], joints[name]));
  const base = origin([0.163038, 0.168068, -0.0324817]);
  const shoulder = joint(base, [-0.124202, -0.168068, 0.0948817], [Math.PI, 0, -Math.PI], q("waist"));
  const upper = joint(shoulder, [-0.0303992, -0.0182778, -0.0542], [-Math.PI / 2, -Math.PI / 2, 0], q("shoulder"));
  const lower = joint(upper, [-0.11257, -0.028, 0], [0, 0, Math.PI / 2], q("elbow"));
  const wrist = joint(lower, [-0.1349, 0.0052, 0], [0, 0, -Math.PI / 2], q("wristPitch"));
  const gripper = joint(wrist, [0, -0.0611, 0.0181], [Math.PI / 2, 0.0486795, Math.PI], q("wristRoll"));
  const jaw = joint(gripper, CLAW_JAW_ORIGIN, [Math.PI / 2, -5.24284e-8, 0], q("gripper"));
  return [base, shoulder, upper, lower, wrist, gripper, jaw];
}

export function armGeometry(joints: ArmJointAngles): Solid[] {
  const frames = armFrames(joints);
  const orange = "#ef831c", motor = "#26282b";
  const points = frames.slice(1, 6).map((m) => new Vector3().setFromMatrixPosition(m));
  const solids: Solid[] = [
    box(origin([0.01, 0, 0.008]), [0.09, 0.07, 0.016], orange),
    box(origin([-0.026, 0, -0.015]), [0.016, 0.05, 0.046], motor),
    box(origin([-0.01, 0, -0.034]), [0.048, 0.05, 0.012], motor),
    beam(new Vector3(points[0].x, points[0].y, 0.016), points[0], 0.018, orange),
  ];
  points.forEach((p, i) => {
    solids.push(box(frames[i + 1].clone().multiply(origin([0, 0, 0.009])), [0.04, 0.025, 0.035], motor));
    if (i === 2 || i === 3) {
      const offset = new Vector3(0, 0, 1).transformDirection(frames[i]).multiplyScalar(0.018);
      for (const side of [-1, 1]) {
        const a = points[i - 1].clone().addScaledVector(offset, side);
        const b = p.clone().addScaledVector(offset, side);
        solids.push(beam(a, b, 0.007, orange), beam(points[i - 1], a, 0.008, orange), beam(p, b, 0.008, orange));
      }
    } else if (i > 0) solids.push(beam(points[i - 1], p, 0.013, orange));
  });
  solids.push(...clawGeometry(joints, frames));
  return solids;
}

const FIXED_FINGER: [number, number][] = [
  [-0.012, -0.031], [-0.027, -0.043], [-0.033, -0.070], [-0.030, -0.099],
  [-0.0129, -0.099], [-0.0129, -0.085], [-0.019, -0.085], [-0.022, -0.069],
  [-0.017, -0.047], [-0.005, -0.039],
];
const MOVING_FINGER: [number, number][] = [
  [-0.010, 0.006], [0.010, 0.006], [0.020, -0.031], [0.021, -0.055],
  [0.011, -0.076], [-0.0157, -0.076], [-0.0157, -0.062], [0.002, -0.062],
  [0.008, -0.051], [0.007, -0.032], [-0.010, -0.008],
];

export function clawGeometry(joints: ArmJointAngles, frames = armFrames(joints)): Solid[] {
  const roll = frames[5], jaw = frames[6];
  const orange = "#ef831c", rubber = "#24272a";
  const moving = jaw.clone().multiply(origin([0, 0, CLAW_MOVING_PLANE]));
  const fixed = roll.clone().multiply(origin([0, 0, 0], [Math.PI / 2, 0, 0]));
  const brace = beam(new Vector3().setFromMatrixPosition(roll), new Vector3().setFromMatrixPosition(jaw), 0.009, orange);
  const solids: Solid[] = [
    { ...box(roll.clone().multiply(origin([0.0077, 0.0001, -0.0234])), [0.032, 0.045, 0.04], rubber), name: "palm" },
    { ...box(roll.clone().multiply(origin([0.0077, 0.0001, -0.004])), [0.041, 0.049, 0.008], orange), name: "palm-collar" },
    { ...brace, kind: "box", size: [0.018, brace.size[1], 0.018], name: "jaw-brace" },
    { kind: "prism", name: "fixed-finger", matrix: fixed, size: [0, 0, 0.014], color: orange, outline: FIXED_FINGER },
    { kind: "prism", name: "moving-finger", matrix: moving, size: [0, 0, 0.014], color: orange, outline: MOVING_FINGER },
    { ...box(fixed.clone().multiply(origin(CLAW_FIXED_PAD)), CLAW_PAD_SIZE, rubber), name: "fixed-pad" },
    { ...box(moving.clone().multiply(origin(CLAW_MOVING_PAD)), CLAW_PAD_SIZE, rubber), name: "moving-pad" },
    { kind: "cylinder", name: "jaw-hinge", matrix: moving.clone().multiply(origin([0, 0, 0], [Math.PI / 2, 0, 0])), size: [0.010, 0.022, 0.010], color: rubber },
    { kind: "cylinder", name: "hinge-pin", matrix: moving.clone().multiply(origin([0, 0, 0], [Math.PI / 2, 0, 0])), size: [0.0035, 0.025, 0.0035], color: "#a9b0b8" },
  ];
  for (let i = 0; i < 3; i++) {
    const y = -0.096 + i * 0.004;
    solids.push(box(fixed.clone().multiply(origin([-0.0075, y, 0])), [0.0015, 0.0012, 0.015], "#454b51"));
    solids.push(box(moving.clone().multiply(origin([-0.0211, y + 0.0234, 0])), [0.0015, 0.0012, 0.015], "#454b51"));
  }
  return solids;
}

export function urdfArmTip(mount: ArmState["mount"], joints: ArmJointAngles) {
  const p = new Vector3(-0.0079, -0.000218121, -0.0981274).applyMatrix4(armFrames(joints)[5])
    .applyMatrix4(origin([mount.x, mount.y, 0], [0, 0, mount.yaw]));
  return { x: p.x, y: p.y, z: p.z };
}

export function armGripPosition(mount: ArmState["mount"], joints: ArmJointAngles) {
  const frames = armFrames(joints);
  const fixed = new Vector3(CLAW_FIXED_PAD[0] + CLAW_PAD_SIZE[0] / 2, -CLAW_FIXED_PAD[2], CLAW_FIXED_PAD[1]).applyMatrix4(frames[5]);
  const moving = new Vector3(CLAW_MOVING_PAD[0] - CLAW_PAD_SIZE[0] / 2, CLAW_MOVING_PAD[1], CLAW_MOVING_PAD[2] + CLAW_MOVING_PLANE).applyMatrix4(frames[6]);
  return fixed.add(moving).multiplyScalar(0.5).applyMatrix4(origin([mount.x, mount.y, 0], [0, 0, mount.yaw]));
}

type HandleGrip = { yaw: number; opening: number };

function alignHandleGrip(mount: ArmState["mount"], q: ArmJointAngles, grip: HandleGrip, referenceRoll: number) {
  const frame = origin([mount.x, mount.y, 0], [0, 0, mount.yaw]).multiply(armFrames({ ...q, wristRoll: 0 })[5]);
  const rail = new Vector3(Math.cos(grip.yaw), Math.sin(grip.yaw), 0);
  const x = new Vector3(1, 0, 0).transformDirection(frame), y = new Vector3(0, 1, 0).transformDirection(frame);
  const angle = Math.atan2(-rail.dot(x), rail.dot(y));
  const rolls = [-2, -1, 0, 1, 2].map((i) => angle + i * Math.PI)
    .filter((v) => v >= ARM_URDF_LIMITS.wristRoll[0] && v <= ARM_URDF_LIMITS.wristRoll[1]);
  q.wristRoll = rolls.sort((a, b) => Math.abs(a - referenceRoll) - Math.abs(b - referenceRoll))[0];
  const normal = x.multiplyScalar(Math.cos(q.wristRoll)).addScaledVector(y, Math.sin(q.wristRoll));
  const width = (SESAME_HANDLE_THICKNESS + 2 * PRISM_BEVEL) * Math.abs(normal.dot(new Vector3(-rail.y, rail.x, 0)))
    + (SESAME_HANDLE_RAIL_HEIGHT + 2 * PRISM_BEVEL) * Math.abs(normal.z);
  let lo = ARM_URDF_LIMITS.gripper[0], hi = 0.2;
  for (let i = 0; i < 20; i++) {
    const mid = (lo + hi) / 2;
    const gap = CLAW_JAW_ORIGIN[0] - CLAW_FIXED_PAD[0] - CLAW_PAD_SIZE[0] / 2
      + (CLAW_MOVING_PAD[0] - CLAW_PAD_SIZE[0] / 2) * Math.cos(mid) - CLAW_MOVING_PAD[1] * Math.sin(mid);
    if (gap < width) lo = mid;
    else hi = mid;
  }
  const closed = (lo + hi) / 2;
  q.gripper = closed + (0.9 - closed) * Math.max(0, Math.min(1, grip.opening));
}

export function solveUrdfArmIK(mount: ArmState["mount"], target: { x: number; y: number }, z: number, seed = ARM_HOME, grip?: HandleGrip) {
  const q = { ...seed };
  const names = ["waist", "shoulder", "elbow", "wristPitch"] as const;
  const goal = new Vector3(target.x, target.y, z);
  const position = (joints: ArmJointAngles) => {
    if (!grip) return urdfArmTip(mount, joints);
    alignHandleGrip(mount, joints, grip, seed.wristRoll);
    return armGripPosition(mount, joints);
  };
  for (let step = 0; step < 320; step++) {
    const p = position(q);
    const error = goal.clone().sub(new Vector3(p.x, p.y, p.z));
    if (error.length() < (grip ? 0.0004 : 0.001)) break;
    for (const name of names) {
      const current = position(q);
      const residual = goal.clone().sub(new Vector3(current.x, current.y, current.z));
      const h = 0.0001;
      const direction = q[name] + h > ARM_URDF_LIMITS[name][1] ? -h : h;
      const next = position({ ...q, [name]: q[name] + direction });
      const derivative = new Vector3(next.x - current.x, next.y - current.y, next.z - current.z).divideScalar(direction);
      const delta = Math.max(-0.12, Math.min(0.12, derivative.dot(residual) / (derivative.lengthSq() + 0.004) * 0.4));
      q[name] = Math.max(ARM_URDF_LIMITS[name][0], Math.min(ARM_URDF_LIMITS[name][1], q[name] + delta));
    }
  }
  if (grip) alignHandleGrip(mount, q, grip, seed.wristRoll);
  return q;
}

export function displayRobot(robot: Robot, seconds: number, carried = false): Robot {
  if (carried || (robot.z ?? 0) > 0.001 || (robot.mode !== "moving" && robot.mode !== "turning") || Object.keys(robot.joints ?? {}).length) return robot;
  const phase = seconds * 9;
  const joints = { ...SESAME_HOME };
  const legs: [SesameJoint, SesameJoint, number, number][] = [
    ["R1", "R3", 0, -1], ["L2", "L4", 0, -1],
    ["R2", "R4", Math.PI, 1], ["L1", "L3", Math.PI, 1],
  ];
  legs.forEach(([hip, knee, offset, sign]) => {
    joints[hip] += Math.sin(phase + offset) * 0.23;
    joints[knee] += Math.max(0, Math.cos(phase + offset)) * 0.7 * sign;
  });
  return { ...robot, joints };
}

const SESAME_SHELL: [number, number][] = [
  [-0.041, -0.0105], [-0.035, -0.0165], [0.037, -0.0165], [0.043, -0.0105],
  [0.043, 0.0185], [0.028, 0.0455], [-0.029, 0.0455], [-0.041, 0.0315],
];
export const PRISM_BEVEL = 0.0006;
const SESAME_HANDLE_THICKNESS = 0.004;
const SESAME_HANDLE_RAIL_HEIGHT = 0.007;
const SESAME_HANDLE_GRIP = { x: -0.002, z: 0.0735 };
const SESAME_HANDLE: [number, number][] = [
  [-0.030, 0.028], [-0.030, 0.068], [-0.024, 0.077], [0.019, 0.077],
  [0.027, 0.068], [0.027, 0.028], [0.020, 0.028], [0.020, 0.065],
  [0.015, 0.070], [-0.019, 0.070], [-0.023, 0.065], [-0.023, 0.028],
];
const SESAME_SHIN: [number, number][] = [
  [-0.009, -0.021], [0.009, -0.021], [0.009, -0.002], [0.005, 0.021],
  [-0.005, 0.021], [-0.009, 0.012], [-0.003, 0.012], [0.002, -0.003], [-0.009, -0.003],
];

export function sesameGeometry(robot: Pick<Robot, "joints" | "shellColor" | "tracking">): { solids: Solid[]; baseHeight: number } {
  const shell = robot.tracking ? robot.shellColor ?? "#202226" : "#88929c";
  const motor = robot.tracking ? "#303238" : "#77828c";
  const q = (name: SesameJoint) => {
    const lo = name === "R1" || name === "L2" ? Math.PI / 4 : 0;
    const hi = name === "R2" || name === "L1" ? 3 * Math.PI / 4 : Math.PI;
    const value = robot.joints?.[name];
    return Math.max(lo, Math.min(hi, value !== undefined && Number.isFinite(value) ? value : SESAME_HOME[name]));
  };
  const layouts: [SesameJoint, SesameJoint, Vec3, number, number][] = [
    ["R1", "R3", [0.0240966, -0.0206276, 0.0179057], Math.PI / 4, -1],
    ["R2", "R4", [-0.0243034, -0.0206276, 0.0179057], -Math.PI / 4, 1],
    ["L1", "L3", [0.0240966, 0.0301724, 0.0179057], 3 * Math.PI / 4, 1],
    ["L2", "L4", [-0.0243034, 0.0301724, 0.0179057], -3 * Math.PI / 4, -1],
  ];
  const solids: Solid[] = [
    { kind: "prism", name: "shell", matrix: origin([0, 0.004, 0], [Math.PI / 2, 0, 0]),
      size: [0.084, 0.062, 0.068], color: shell, outline: SESAME_SHELL },
    { ...box(origin([0.001, 0.004, -0.012]), [0.078, 0.063, 0.012], motor), name: "chassis" },
  ];
  for (const side of [-1, 1]) {
    solids.push({ kind: "prism", name: side === 1 ? "left-handle" : "right-handle",
      matrix: origin([0, 0.004 + side * 0.034, 0], [Math.PI / 2, 0, 0]),
      size: [0.057, 0.049, SESAME_HANDLE_THICKNESS], color: shell, outline: SESAME_HANDLE });
  }
  let minZ = -0.018;
  for (const [hipName, kneeName, xyz, yaw, side] of layouts) {
    const hip = joint(new Matrix4(), xyz, [Math.PI, 0, yaw], q(hipName), -1);
    const knee = joint(hip, [-0.0207104, side * 0.0313751, 0.01779],
      [1.57079632557, 0, side === 1 ? 2.35619448904 : 0.785398164546], q(kneeName), -1);
    const a = new Vector3().setFromMatrixPosition(hip);
    const b = new Vector3().setFromMatrixPosition(knee);
    const foot = new Vector3(-0.0016, side * 0.04006, -0.016).applyMatrix4(knee);
    const upper = beam(a, b, 0.01, shell);
    const lower = beam(b, foot, 0.006, shell);
    solids.push(
      { ...box(upper.matrix, [0.017, upper.size[1], 0.012], shell), name: `${hipName}-bracket` },
      { ...box(hip, [0.023, 0.012, 0.023], motor), name: `${hipName}-servo` },
      { ...box(knee, [0.023, 0.024, 0.012], motor), name: `${kneeName}-servo` },
      { ...box(knee.clone().multiply(origin([0, 0, -0.007])), [0.020, 0.021, 0.002], "#515157"), name: `${kneeName}-cover` },
      { kind: "prism", name: `${kneeName}-shin`, matrix: lower.matrix, size: [0.018, 0.042, 0.007], color: shell, outline: SESAME_SHIN },
      { ...ball(foot, 0.007, "#191b1e"), name: `${kneeName}-foot` },
    );
    minZ = Math.min(minZ, foot.z - 0.007, a.z - 0.016, b.z - 0.017);
  }
  const baseHeight = -minZ;
  solids.forEach((s) => s.matrix.premultiply(origin([0, 0, baseHeight])));
  return { solids, baseHeight };
}

export function sesameTopView(robot: Pick<Robot, "joints" | "shellColor" | "tracking">, legSpread = 1) {
  const { solids } = sesameGeometry(robot);
  const parts = new Map(solids.map((s) => [s.name, s]));
  const point = (name: string) => new Vector3().setFromMatrixPosition(parts.get(name)!.matrix);
  return {
    shell: projectedOutline(parts.get("shell")!),
    handles: solids.filter((s) => s.name?.endsWith("-handle")).map(projectedOutline),
    legs: [["R1", "R3"], ["R2", "R4"], ["L1", "L3"], ["L2", "L4"]].map(([hipName, kneeName]) => {
      const hip = point(`${hipName}-servo`);
      const inset = (p: Vector3) => new Vector3(hip.x + (p.x - hip.x) * legSpread, hip.y + (p.y - hip.y) * legSpread, p.z);
      return { hip, knee: inset(point(`${kneeName}-servo`)), foot: inset(point(`${kneeName}-foot`)) };
    }),
  };
}

export function sesameHandleTargets(robot: Pick<Robot, "joints" | "shellColor" | "tracking" | "yaw">) {
  return sesameGeometry(robot).solids.filter((s) => s.name?.endsWith("-handle")).map((s) => ({
    name: s.name!,
    offset: new Vector3(SESAME_HANDLE_GRIP.x, SESAME_HANDLE_GRIP.z, 0).applyMatrix4(s.matrix)
      .applyMatrix4(origin([0, 0, 0], [0, 0, robot.yaw])),
  }));
}

export function projectedFaces(solid: Solid): Vector3[][] {
  if (solid.kind !== "prism") return [projectedOutline(solid)];
  const rings = [-1, 1].map((side) => solid.outline.map(([x, y]) =>
    new Vector3(x, y, side * solid.size[2] / 2).applyMatrix4(solid.matrix)));
  const faces = [...rings];
  for (let i = 0; i < solid.outline.length; i++) {
    const j = (i + 1) % solid.outline.length;
    faces.push([rings[0][i], rings[0][j], rings[1][j], rings[1][i]]);
  }
  return faces.sort((a, b) => a.reduce((sum, p) => sum + p.z, 0) / a.length - b.reduce((sum, p) => sum + p.z, 0) / b.length);
}

export function projectedOutline(solid: Solid): Vector3[] {
  const { size, matrix, kind } = solid;
  const points: Vector3[] = [];
  if (solid.kind === "prism") {
    for (const [x, y] of solid.outline) for (const z of [-size[2] / 2, size[2] / 2]) {
      points.push(new Vector3(x, y, z).applyMatrix4(matrix));
    }
  } else if (kind === "box") {
    for (const x of [-1, 1]) for (const y of [-1, 1]) for (const z of [-1, 1]) {
      points.push(new Vector3(x * size[0] / 2, y * size[1] / 2, z * size[2] / 2).applyMatrix4(matrix));
    }
  } else if (kind === "cylinder") {
    for (const y of [-size[1] / 2, size[1] / 2]) for (let i = 0; i < 24; i++) {
      const a = i * Math.PI / 12;
      points.push(new Vector3(Math.cos(a) * size[0], y, Math.sin(a) * size[0]).applyMatrix4(matrix));
    }
  } else {
    const ends = kind === "sphere" ? [0] : [-size[1] / 2, size[1] / 2];
    for (const y of ends) {
      const center = new Vector3(0, y, 0).applyMatrix4(matrix);
      for (let i = 0; i < 16; i++) points.push(new Vector3(center.x + Math.cos(i * Math.PI / 8) * size[0], center.y + Math.sin(i * Math.PI / 8) * size[0], center.z));
    }
  }
  points.sort((a, b) => a.x - b.x || a.y - b.y);
  const cross = (a: Vector3, b: Vector3, c: Vector3) => (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  const half = (pts: Vector3[]) => {
    const hull: Vector3[] = [];
    for (const p of pts) {
      while (hull.length >= 2 && cross(hull[hull.length - 2], hull[hull.length - 1], p) <= 0) hull.pop();
      hull.push(p);
    }
    hull.pop();
    return hull;
  };
  return [...half(points), ...half([...points].reverse())];
}
