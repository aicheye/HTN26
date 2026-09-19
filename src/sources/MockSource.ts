import { cloneSampleWorldState } from "../data/sampleWorldState";
import {
  ARM_ASSIST_HEIGHT_M,
  ARM_CARRY_LIFT_M,
  ARM_MAX_REACH,
  ARM_REST_POSE,
  ARM_TRANSIT_HEIGHT_M,
  armTipPosition,
  clampArmJoint,
  isWithinArmReach,
  solveArmIK,
} from "../components/mapShared";
import type {
  Ack,
  ArmMode,
  Command,
  Obstacle,
  Point,
  Robot,
  WorldState,
} from "../types/world";
import type { ConnectionStatus, StateSource } from "./StateSource";

const TICK_MS = 50; // ~20 Hz
const LINEAR_SPEED = 0.45; // m/s at speed = 1
const ANGULAR_SPEED = 1.6; // rad/s at speed = 1
const GOTO_ARRIVE_RADIUS = 0.06; // meters
const POSE_DURATION_MS = 2_500; // one-shot animations block movement while playing

/** How long the robot has to be blocked by a too-tall obstacle before the arm helps. */
const ARM_STUCK_TRIGGER_MS = 900;

/** Duration of each phase in the pick-and-carry sequence, ms. */
const ARM_PHASE_MS: Partial<Record<ArmMode, number>> = {
  reaching: 900,
  grasping: 500,
  lifting: 500,
  carrying: 1400,
  placing: 500,
  releasing: 400,
  returning: 900,
};

const ARM_NEXT: Partial<Record<ArmMode, ArmMode>> = {
  reaching: "grasping",
  grasping: "lifting",
  lifting: "carrying",
  carrying: "placing",
  placing: "releasing",
  releasing: "returning",
  returning: "idle",
};

type ActiveCommand = {
  command: Command;
  expiresAt: number | null;
};

/** Browser-only fake backend: simulates detections and robot motion from the sample arena. */
export class MockSource implements StateSource {
  private state: WorldState;
  private truth: { x: number; y: number; yaw: number };
  private bias = { x: 0, y: 0, yaw: 0 };
  private active: ActiveCommand | null = null;
  private poseUntil = 0;
  private timer: number | null = null;
  private lastTick = 0;

  // pick-and-carry assist state
  private stuckMs = 0;
  private armElapsed = 0;
  private armFrom: Point | null = null;
  private armTo: Point | null = null;
  private lastBlockedBy: Obstacle | null = null;
  // smoothed gripper target used while reaching/carrying, so the tip travels
  // in a straight Cartesian line (and floor height) instead of swinging
  // through the floor when joint angles are interpolated directly
  private armPoint: Point = { x: 0, y: 0 };
  private armZ = 0;

  private stateSubs = new Set<(s: WorldState) => void>();
  private ackSubs = new Set<(a: Ack) => void>();
  private statusSubs = new Set<(s: ConnectionStatus) => void>();

  constructor() {
    this.state = cloneSampleWorldState();
    const r = this.robot();
    this.truth = { x: r.x, y: r.y, yaw: r.yaw };
  }

  subscribe(cb: (s: WorldState) => void) {
    this.stateSubs.add(cb);
    cb(this.state);
    this.start();
    return () => {
      this.stateSubs.delete(cb);
      if (this.stateSubs.size === 0) this.stop();
    };
  }

  onAck(cb: (a: Ack) => void) {
    this.ackSubs.add(cb);
    return () => this.ackSubs.delete(cb);
  }

  onStatus(cb: (s: ConnectionStatus) => void) {
    this.statusSubs.add(cb);
    cb(this.timer === null ? "connecting" : "live");
    return () => this.statusSubs.delete(cb);
  }

  start() {
    if (this.timer !== null) return;
    this.lastTick = performance.now();
    this.timer = window.setInterval(() => this.tick(), TICK_MS);
    this.statusSubs.forEach((cb) => cb("live"));
  }

  stop() {
    if (this.timer === null) return;
    window.clearInterval(this.timer);
    this.timer = null;
    this.statusSubs.forEach((cb) => cb("closed"));
  }

  sendCommand(c: Command) {
    const robot = this.state.robots.find((r) => r.id === c.robotId);
    if (!robot) {
      this.emitAck({ commandId: c.id, ok: false, error: "unknown robotId" });
      return;
    }
    if (c.type === "goto" && !c.target) {
      this.emitAck({ commandId: c.id, ok: false, error: "goto needs target" });
      return;
    }

    if (c.type === "stop") {
      this.active = null;
      this.state.goal = undefined;
      this.state.path = undefined;
    } else if (c.type === "face") {
      if (c.face) robot.face = c.face;
    } else if (c.type === "pose") {
      if (!c.pose) {
        this.emitAck({ commandId: c.id, ok: false, error: "pose needs a name" });
        return;
      }
      this.active = null;
      robot.pose = c.pose;
      robot.face = c.face ?? c.pose;
      this.poseUntil = Date.now() + POSE_DURATION_MS;
    } else if (c.type === "goto" && c.target) {
      this.active = { command: c, expiresAt: null };
      this.state.goal = { ...c.target };
      this.state.path = [{ x: this.truth.x, y: this.truth.y }, { ...c.target }];
    } else {
      this.active = {
        command: c,
        expiresAt: c.durationMs ? Date.now() + c.durationMs : null,
      };
      robot.face = c.face ?? "walk";
    }
    this.emitAck({ commandId: c.id, ok: true });
  }

  private emitAck(a: Ack) {
    this.ackSubs.forEach((cb) => cb(a));
  }

  private robot(): Robot {
    return this.state.robots[0];
  }

  private tick() {
    const now = performance.now();
    const dt = Math.min((now - this.lastTick) / 1000, 0.2);
    this.lastTick = now;

    const cmd = this.active?.command;
    if (this.active?.expiresAt && Date.now() > this.active.expiresAt) {
      this.active = null;
    }

    const robot = this.robot();
    const posing = Date.now() < this.poseUntil;
    if (!posing && robot.pose) {
      robot.pose = undefined;
      robot.face = "idle";
    }

    const arm = this.state.arm;
    const assisting = !!arm && arm.mode !== "idle";

    let mode: Robot["mode"] = "idle";
    if (assisting) {
      mode = "idle";
    } else if (cmd && !posing) {
      mode = this.step(cmd, dt);
      this.trackStuck(cmd, dt);
    } else {
      this.stuckMs = 0;
    }

    if (arm) this.advanceArm(arm, dt);

    // Reported pose = truth + a slowly drifting estimate error, not per-frame hash.
    this.bias.x = this.bias.x * 0.94 + gauss(0.004) * 0.06;
    this.bias.y = this.bias.y * 0.94 + gauss(0.004) * 0.06;
    this.bias.yaw = this.bias.yaw * 0.94 + gauss(0.01) * 0.06;
    robot.x = this.truth.x + this.bias.x;
    robot.y = this.truth.y + this.bias.y;
    robot.yaw = this.truth.yaw + this.bias.yaw;
    robot.lastSeen = Date.now();
    robot.confidence = 0.93;
    robot.mode = mode;
    robot.tracking = true;

    this.state.seq += 1;
    this.state.timestamp = Date.now();
    this.state = { ...this.state, robots: [{ ...robot }] };
    this.stateSubs.forEach((cb) => cb(this.state));
  }

  /** Advances ground truth for one command, returns the resulting mode. */
  private step(cmd: Command, dt: number): Robot["mode"] {
    const speed = clamp(cmd.speed ?? 1, 0, 1);

    if (cmd.type === "left" || cmd.type === "right") {
      const dir = cmd.type === "left" ? 1 : -1;
      this.truth.yaw = wrapAngle(this.truth.yaw + dir * ANGULAR_SPEED * speed * dt);
      return "turning";
    }

    if (cmd.type === "goto" && cmd.target) {
      const dx = cmd.target.x - this.truth.x;
      const dy = cmd.target.y - this.truth.y;
      const dist = Math.hypot(dx, dy);
      if (dist < GOTO_ARRIVE_RADIUS) {
        this.active = null;
        this.state.goal = undefined;
        this.state.path = undefined;
        return "idle";
      }
      const desired = Math.atan2(dy, dx);
      const err = wrapAngle(desired - this.truth.yaw);
      if (Math.abs(err) > 0.12) {
        this.truth.yaw = wrapAngle(
          this.truth.yaw + Math.sign(err) * Math.min(ANGULAR_SPEED * dt, Math.abs(err)),
        );
        this.state.path = [{ x: this.truth.x, y: this.truth.y }, { ...cmd.target }];
        return "turning";
      }
      this.advance(Math.min(LINEAR_SPEED * dt, dist));
      this.state.path = [{ x: this.truth.x, y: this.truth.y }, { ...cmd.target }];
      return "moving";
    }

    if (cmd.type === "forward" || cmd.type === "backward") {
      const dir = cmd.type === "forward" ? 1 : -1;
      this.advance(dir * LINEAR_SPEED * speed * dt);
      return "moving";
    }

    return "idle";
  }

  private advance(distance: number) {
    const nx = this.truth.x + Math.cos(this.truth.yaw) * distance;
    const ny = this.truth.y + Math.sin(this.truth.yaw) * distance;
    const { width, length } = this.state.arena;
    const r = this.robot();
    const margin = Math.max(r.footprint.width, r.footprint.length) / 2;

    const cx = clamp(nx, margin, width - margin);
    const cy = clamp(ny, margin, length - margin);
    const blocker = this.findBlocker(cx, cy, margin);
    this.lastBlockedBy = blocker ?? null;
    if (blocker) return;
    this.truth.x = cx;
    this.truth.y = cy;
  }

  private findBlocker(x: number, y: number, radius: number): Obstacle | undefined {
    return this.state.obstacles.find((o) => circleHitsObstacle(x, y, radius, o));
  }

  /** Counts how long a drive command has been blocked by an obstacle too tall to climb. */
  private trackStuck(cmd: Command, dt: number) {
    const blocker = this.lastBlockedBy;
    const drivingInto = cmd.type === "forward" || cmd.type === "backward" || cmd.type === "goto";
    if (!blocker || !drivingInto || (blocker.height ?? 0) <= ARM_ASSIST_HEIGHT_M) {
      this.stuckMs = 0;
      return;
    }
    this.stuckMs += dt * 1000;
    if (this.stuckMs >= ARM_STUCK_TRIGGER_MS && this.state.arm?.mode === "idle") {
      this.triggerArmAssist(blocker);
    }
  }

  /** Kicks off the reach -> grasp -> lift -> carry -> place -> release -> return sequence. */
  private triggerArmAssist(obstacle: Obstacle) {
    const arm = this.state.arm;
    if (!arm) return;
    const robot = this.robot();
    this.stuckMs = 0;

    // the arm only serves its own working envelope; outside it the robot is on its own
    if (!isWithinArmReach(arm.mount, this.truth)) return;

    const margin = Math.max(robot.footprint.width, robot.footprint.length) / 2;
    const { width, length } = this.state.arena;
    const goal = this.state.goal;

    let drop: Point | null = null;

    // Get as close to the robot's actual destination as the arm can physically
    // reach: walk the mount->goal ray in from the goal (or from the edge of the
    // reach envelope, if the goal itself is farther out) toward the mount,
    // stopping at the first clear spot. That's exactly the goal when it's
    // reachable and unobstructed, otherwise the nearest point to it that isn't.
    if (goal) {
      const dx = goal.x - arm.mount.x;
      const dy = goal.y - arm.mount.y;
      const goalDist = Math.hypot(dx, dy);
      const dir = goalDist > 1e-6 ? { x: dx / goalDist, y: dy / goalDist } : { x: 1, y: 0 };
      const startR = Math.min(goalDist, ARM_MAX_REACH - margin - 0.01);
      for (let r = startR; r > margin; r -= 0.02) {
        const p = {
          x: clamp(arm.mount.x + dir.x * r, margin, width - margin),
          y: clamp(arm.mount.y + dir.y * r, margin, length - margin),
        };
        if (!this.findBlocker(p.x, p.y, margin)) {
          drop = p;
          break;
        }
      }
    }

    if (!drop) {
      // no goal (plain drive command), or the whole reach envelope toward the
      // goal was blocked: fall back to searching outward from the obstacle,
      // biased toward the goal (or the direction the robot was already
      // heading) so the drop still continues the trip where it can
      const toward = goal
        ? normalize({ x: goal.x - obstacle.x, y: goal.y - obstacle.y })
        : normalize({ x: obstacle.x - this.truth.x, y: obstacle.y - this.truth.y });
      const side = { x: -toward.y, y: toward.x };
      const directions = [toward, side, { x: -side.x, y: -side.y }, { x: -toward.x, y: -toward.y }];
      const base = obstacleRadius(obstacle) + margin + 0.09;
      for (const testDir of directions) {
        for (let i = 0; i < 14; i++) {
          const c = base + i * 0.05;
          const p = {
            x: clamp(obstacle.x + testDir.x * c, margin, width - margin),
            y: clamp(obstacle.y + testDir.y * c, margin, length - margin),
          };
          if (!this.findBlocker(p.x, p.y, margin) && isWithinArmReach(arm.mount, p)) {
            drop = p;
            break;
          }
        }
        if (drop) break;
      }
    }
    if (!drop) return;

    this.armFrom = { x: this.truth.x, y: this.truth.y };
    this.armTo = drop;
    this.armElapsed = 0;
    // start the Cartesian smoothing from wherever the gripper currently sits
    const tip = armTipPosition(arm.mount, arm.joints);
    this.armPoint = { x: tip.x, y: tip.y };
    this.armZ = tip.z;
    arm.mode = "reaching";
    arm.targetRobotId = robot.id;
  }

  /** Advances the arm's phase timer, joint pose, and (while carrying) the robot's position. */
  private advanceArm(arm: NonNullable<WorldState["arm"]>, dt: number) {
    if (arm.mode === "idle") return;
    this.armElapsed += dt * 1000;

    if (arm.mode === "carrying" && this.armFrom && this.armTo) {
      const dur = ARM_PHASE_MS.carrying ?? 1;
      const t = easeInOut(clamp(this.armElapsed / dur, 0, 1));
      this.truth.x = lerp(this.armFrom.x, this.armTo.x, t);
      this.truth.y = lerp(this.armFrom.y, this.armTo.y, t);
    }

    const k = 1 - Math.exp(-8 * dt);

    // move the gripper's target point in a straight Cartesian line (and re-solve IK each
    // frame) so the tip's height changes monotonically and can't dip through the floor -
    // "returning" aims at the rest pose's own tip position, not its (very folded) angles,
    // since interpolating those angles directly swings the tip below the floor partway through
    const { point, z, gripper } = this.armPhaseTarget(arm);
    this.armPoint.x += (point.x - this.armPoint.x) * k;
    this.armPoint.y += (point.y - this.armPoint.y) * k;
    this.armZ += (z - this.armZ) * k;
    const ik = solveArmIK(arm.mount, this.armPoint, this.armZ);
    arm.joints.waist = clampArmJoint("waist", ik.waist);
    arm.joints.shoulder = clampArmJoint("shoulder", ik.shoulder);
    arm.joints.elbow = clampArmJoint("elbow", ik.elbow);
    arm.joints.wristPitch = clampArmJoint("wristPitch", ik.wristPitch);
    arm.joints.wristRoll = 0;
    arm.joints.gripper += (gripper - arm.joints.gripper) * k;

    const dur = ARM_PHASE_MS[arm.mode];
    if (dur !== undefined && this.armElapsed >= dur) {
      this.armElapsed = 0;
      const next = ARM_NEXT[arm.mode] ?? "idle";
      arm.mode = next;
      if (next === "idle") {
        // snap into the exact folded rest shape now that the tip has arrived at its position
        arm.joints = { ...ARM_REST_POSE, waist: 0 };
        arm.targetRobotId = undefined;
        this.armFrom = null;
        this.armTo = null;
        // the rescue is done; drop the command and goal so it doesn't march back in
        this.active = null;
        this.state.goal = undefined;
        this.state.path = undefined;
      }
    }
  }

  /** Where the gripper should be for the current phase, and how open. */
  private armPhaseTarget(arm: NonNullable<WorldState["arm"]>): {
    point: Point;
    z: number;
    gripper: number;
  } {
    if (arm.mode === "returning") {
      // aim at the rest pose's own tip position (not its very-folded angles) so the
      // Cartesian smoothing below can't swing the tip below the floor on the way there
      const tip = armTipPosition(arm.mount, { ...ARM_REST_POSE, waist: 0 });
      return { point: { x: tip.x, y: tip.y }, z: tip.z, gripper: 0.1 };
    }

    const point =
      arm.mode === "carrying"
        ? { x: this.truth.x, y: this.truth.y }
        : arm.mode === "placing" || arm.mode === "releasing"
          ? (this.armTo ?? this.truth)
          : (this.armFrom ?? this.truth);

    const z =
      arm.mode === "reaching"
        ? ARM_TRANSIT_HEIGHT_M
        : arm.mode === "lifting" || arm.mode === "carrying"
          ? ARM_CARRY_LIFT_M
          : arm.mode === "placing"
            ? ARM_CARRY_LIFT_M *
              (1 - easeInOut(clamp(this.armElapsed / (ARM_PHASE_MS.placing ?? 1), 0, 1)))
            : 0;

    const gripper =
      arm.mode === "reaching" || arm.mode === "releasing" ? 0.6 : arm.mode === "idle" ? 0.1 : 0;

    return { point, z, gripper };
  }
}

function circleHitsObstacle(x: number, y: number, radius: number, o: Obstacle): boolean {
  if (o.shape === "circle") {
    return Math.hypot(x - o.x, y - o.y) < (o.radius ?? 0) + radius;
  }
  if (o.shape === "polygon" && o.points?.length) {
    return pointInPolygon({ x, y }, o.points);
  }
  // rect: test in the obstacle's local frame
  const dx = x - o.x;
  const dy = y - o.y;
  const c = Math.cos(-o.yaw);
  const s = Math.sin(-o.yaw);
  const lx = dx * c - dy * s;
  const ly = dx * s + dy * c;
  const hw = (o.width ?? 0) / 2 + radius;
  const hl = (o.length ?? 0) / 2 + radius;
  return Math.abs(lx) < hw && Math.abs(ly) < hl;
}

function pointInPolygon(p: Point, poly: Point[]): boolean {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const a = poly[i];
    const b = poly[j];
    const intersects =
      a.y > p.y !== b.y > p.y &&
      p.x < ((b.x - a.x) * (p.y - a.y)) / (b.y - a.y) + a.x;
    if (intersects) inside = !inside;
  }
  return inside;
}

function clamp(v: number, lo: number, hi: number) {
  return Math.max(lo, Math.min(hi, v));
}

function wrapAngle(a: number) {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

function gauss(sigma: number) {
  const u = 1 - Math.random();
  const v = Math.random();
  return sigma * Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

function lerp(a: number, b: number, t: number) {
  return a + (b - a) * t;
}

function easeInOut(t: number) {
  return t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
}

function normalize(v: Point): Point {
  const len = Math.hypot(v.x, v.y);
  return len > 1e-6 ? { x: v.x / len, y: v.y / len } : { x: 1, y: 0 };
}

/** Rough bounding radius used to place the robot clear of the obstacle it got stuck on. */
function obstacleRadius(o: Obstacle): number {
  if (o.shape === "circle") return o.radius ?? 0.1;
  if (o.shape === "polygon" && o.points?.length) {
    return Math.max(...o.points.map((p) => Math.hypot(p.x - o.x, p.y - o.y)));
  }
  return Math.hypot((o.width ?? 0.2) / 2, (o.length ?? 0.2) / 2);
}
