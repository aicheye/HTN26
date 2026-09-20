import { makeMockScenario, type MockScenarioId } from "../data/mockScenarios";
import { armGripPosition as armTipPosition, solveUrdfArmIK as solveArmIK, sesameHandleTargets } from "../robot/geometry";
import {
  ARM_ASSIST_HEIGHT_M,
  ARM_MAX_REACH,
  ARM_REST_POSE,
  clampArmJoint,
  distanceTo,
  isWithinArmReach,
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
// The Mock tab walks its robot with the bridge's own navigator and planner, so what it shows is what the real
// bridge would do: the same paths, limits, stuck recovery, walk to the arm's reach, and carry request.
import { Navigator } from "../../bridge/navigator.mjs";
import { EDGE_MARGIN_M } from "../../bridge/planner.mjs";
import { Behaviours } from "../../bridge/behaviours.mjs";

const TICK_MS = 50; // ~20 Hz
const LINEAR_SPEED = 0.45; // m/s at speed = 1
const ANGULAR_SPEED = 1.6; // rad/s at speed = 1
const STEER_CURVE_RAD_PER_M = 2.5;
const POSE_DURATION_MS = 2_500; // one-shot animations block movement while playing

/** How long the robot has to be blocked by a too-tall obstacle before the arm helps. */
const ARM_STUCK_TRIGGER_MS = 900;

/** Duration of each phase in the pick-and-carry sequence, ms. */
const ARM_PHASE_MS: Partial<Record<ArmMode, number>> = {
  reaching: 1300,
  grasping: 1500,
  lifting: 1100,
  carrying: 2200,
  placing: 1100,
  releasing: 600,
  returning: 1400,
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
  private carryZ = 0.2;
  private handleOffset = { x: 0, y: 0, z: 0 };
  private gripOpening = 1;
  private gripYaw = 0;
  private phaseStart: Point & { z: number } = { x: 0, y: 0, z: 0 };
  private paused = false;

  // goto is the bridge's navigator. It sends walking commands, which land in navDrive and move the mock robot. It
  // runs on the mock's own clock, which advances with the ticks, so that tests with a fake clock work.
  private navDrive = "";
  private navSteer = 0;
  private navClockMs = 0;
  private navigator = this.makeNavigator();
  // What Spidey does by itself, from the same module as the bridge: curiosity, and the diary of the table.
  private behaviours: Behaviours = this.makeBehaviours();

  private stateSubs = new Set<(s: WorldState) => void>();
  private ackSubs = new Set<(a: Ack) => void>();
  private statusSubs = new Set<(s: ConnectionStatus) => void>();

  constructor() {
    this.state = makeMockScenario("barrier");
    const r = this.robot();
    this.truth = { x: r.x, y: r.y, yaw: r.yaw };
  }

  private makeNavigator() {
    const navigator = new Navigator((command, steer) => { this.navDrive = command === "stop" ? "" : command; this.navSteer = steer; });
    // An arm request from the navigator: the first drop point that the arm's kinematics can serve is used.
    navigator.requestCarry = (_goal, drops) => drops.some((drop) => this.beginCarry(drop) === null);
    return navigator;
  }

  private makeBehaviours(): Behaviours {
    return new Behaviours({
      goto: (target) => {
        const command: Command = { id: `play-${Date.now()}`, ts: Date.now(), robotId: this.robot().id, type: "goto", target, speed: 0.3 };
        this.active = { command, expiresAt: null };
        this.state.goal = { ...target };
        this.paused = false;
        this.navDrive = "";
        this.navigator.start({ ...target });
      },
    }, this.behaviours ? { curious: this.behaviours.status().curious } : undefined);
  }

  /** Drops a small box at a random free spot, for the curiosity and diary features. */
  addRandomObject() {
    const { width, length } = this.state.arena, colours = [["#d5a332", "yellow box"], ["#3f9b5c", "green box"], ["#bc4a41", "red box"], ["#7c5cc4", "purple box"]];
    for (let attempt = 0; attempt < 40; attempt++) {
      const x = 0.12 + Math.random() * (width - 0.24), y = 0.12 + Math.random() * (length - 0.24);
      if (Math.hypot(x - this.truth.x, y - this.truth.y) < 0.2 || this.state.obstacles.some((o) => distanceTo(o, { x, y }) < 0.12)) continue;
      const [color, label] = colours[this.state.obstacles.length % colours.length];
      this.state.obstacles = [...this.state.obstacles, { id: `dropped-${Date.now()}`, label, source: "cv", shape: "rect", x, y, yaw: Math.random() * Math.PI,
        width: 0.05, length: 0.04, height: 0.03, color, confidence: 0.9 }];
      return;
    }
  }

  resetScenario(id: MockScenarioId) {
    this.navigator = this.makeNavigator();
    this.behaviours = this.makeBehaviours();
    this.navDrive = "";
    this.state = makeMockScenario(id);
    const r = this.robot();
    this.truth = { x: r.x, y: r.y, yaw: r.yaw };
    this.bias = { x: 0, y: 0, yaw: 0 };
    this.active = null;
    this.poseUntil = 0;
    this.stuckMs = 0;
    this.armElapsed = 0;
    this.armFrom = null;
    this.armTo = null;
    this.handleOffset = { x: 0, y: 0, z: 0 };
    this.gripOpening = 1;
    this.gripYaw = 0;
    this.lastBlockedBy = null;
    this.paused = false;
    this.lastTick = performance.now();
    this.stateSubs.forEach((cb) => cb(this.state));
  }

  runScenarioTest() {
    this.resetScenario((this.state.simulation?.scenario ?? "barrier") as MockScenarioId);
    const simulation = this.state.simulation!;
    simulation.status = "running";
    simulation.message = "Walking toward the test goal.";
    this.sendCommand({ id: `mock-test-${Date.now()}`, ts: Date.now(), robotId: this.robot().id,
      type: "goto", target: simulation.testGoal, speed: 0.3 });
  }

  private grabHeight() {
    return this.handleOffset.z;
  }

  private blockTest(message: string) {
    if (this.navigator.state === "carrying") this.navigator.carried(false, message);
    this.navDrive = "";
    this.active = null;
    this.paused = true;
    if (this.state.simulation) Object.assign(this.state.simulation, { status: "blocked", message });
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

    if (this.paused && this.state.arm?.mode !== "idle" && c.type !== "stop") {
      this.emitAck({ commandId: c.id, ok: false, error: "Reset the mock scene before resuming a paused lift" });
      return;
    }
    if (c.type === "play") {
      this.behaviours.configure(c.play ?? {});
      if (c.play?.curious === false && this.behaviours.status().errand) { this.behaviours.interrupt(); this.navigator.cancel(); this.navDrive = ""; this.active = null; }
      this.emitAck({ commandId: c.id, ok: true });
      return;
    }
    this.behaviours.interrupt();  // any other command is the user taking over
    if (c.type !== "goto") { this.navigator.cancel(); this.navDrive = ""; }
    if (c.type === "stop") {
      this.active = null;
      this.paused = true;
      this.state.goal = undefined;
      this.state.path = undefined;
      if (this.state.simulation) Object.assign(this.state.simulation, { status: "ready", message: "Stopped. Reset or run the test again." });
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
      this.navDrive = "";
      this.navigator.start({ ...c.target });
    } else {
      this.active = {
        command: c,
        expiresAt: c.durationMs ? Date.now() + c.durationMs : null,
      };
      robot.face = c.face ?? "walk";
    }
    if (c.type !== "stop") this.paused = false;
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
    this.navClockMs += dt * 1000;

    if (this.active?.expiresAt && Date.now() > this.active.expiresAt) {
      this.active = null;
    }
    const cmd = this.active?.command;

    const robot = this.robot();
    const posing = Date.now() < this.poseUntil;
    if (!posing && robot.pose) {
      robot.pose = undefined;
      robot.face = "idle";
    }

    const arm = this.state.arm;
    const assisting = !!arm && arm.mode !== "idle";

    let mode: Robot["mode"] = "idle";
    if (assisting || this.paused) {
      mode = "idle";
    } else if (cmd && !posing) {
      mode = this.step(cmd, dt);
      this.trackStuck(cmd, dt);
    } else {
      this.stuckMs = 0;
    }

    if (arm && !this.paused) this.advanceArm(arm, dt);

    // Walking pose includes slowly drifting detector noise; attachment uses the exact gripper pose.
    this.bias.x = this.bias.x * 0.94 + gauss(0.004) * 0.06;
    this.bias.y = this.bias.y * 0.94 + gauss(0.004) * 0.06;
    this.bias.yaw = this.bias.yaw * 0.94 + gauss(0.01) * 0.06;
    robot.x = this.truth.x + (assisting ? 0 : this.bias.x);
    robot.y = this.truth.y + (assisting ? 0 : this.bias.y);
    robot.yaw = this.truth.yaw + (assisting ? 0 : this.bias.yaw);
    robot.lastSeen = Date.now();
    robot.confidence = 0.93;
    robot.mode = mode;
    robot.tracking = true;

    if (!this.paused) this.behaviours.step({ robot: { x: robot.x, y: robot.y, tracking: true }, obstacles: this.state.obstacles, mission: this.navigator.status() });
    this.state.play = this.behaviours.status();
    // What the telemetry panel shows for the live bridge, from the same navigator.
    this.state.mission = { ...this.navigator.status(), robotRadius: this.navigator.robotRadius, drive: "mock", robotConnected: true };
    this.state.arena = { ...this.state.arena, edgeMargin: EDGE_MARGIN_M };

    this.state.seq += 1;
    this.state.timestamp = Date.now();
    this.state = { ...this.state, robots: [{ ...robot }] };
    this.stateSubs.forEach((cb) => cb(this.state));
  }

  /** Advances ground truth for one command, returns the resulting mode. */
  private step(cmd: Command, dt: number): Robot["mode"] {
    const speed = clamp(cmd.speed ?? 1, 0, 1);
    this.lastBlockedBy = null;

    if (cmd.type === "left" || cmd.type === "right") {
      const dir = cmd.type === "left" ? 1 : -1;
      this.truth.yaw = wrapAngle(this.truth.yaw + dir * ANGULAR_SPEED * speed * dt);
      return "turning";
    }

    if (cmd.type === "goto" && cmd.target) return this.stepNavigator(speed, dt);

    if (cmd.type === "forward" || cmd.type === "backward") {
      const dir = cmd.type === "forward" ? 1 : -1;
      this.advance(dir * LINEAR_SPEED * speed * dt);
      return "moving";
    }

    return "idle";
  }

  /** One tick of the bridge's navigator, and of the walking command it has chosen. */
  private stepNavigator(speed: number, dt: number): Robot["mode"] {
    const robot = this.robot(), arm = this.state.arm;
    const radius = Math.hypot(robot.footprint.width, robot.footprint.length) / 2;
    this.navigator.robotRadius = radius;
    // 4 cm inside the arm's furthest reach, because a grip at full stretch fails the joint limits.
    this.navigator.arm = arm ? { base: arm.mount, reach: ARM_MAX_REACH - 0.04 } : null;
    this.navigator.step({ x: robot.x, y: robot.y, yaw: robot.yaw, tracking: true }, this.state.arena, this.state.obstacles, this.navClockMs);

    const status = this.navigator.status();
    this.state.path = this.navigator.path.length ? [{ x: this.truth.x, y: this.truth.y }, ...this.navigator.path] : undefined;
    if (this.state.simulation?.status === "running" && status.detail) this.state.simulation.message = status.detail;
    if (status.state === "done" || status.state === "failed") {
      this.active = null;
      this.navDrive = "";
      this.state.goal = undefined;
      this.state.path = undefined;
      if (status.state === "failed") this.blockTest(status.detail);
      else if (this.state.simulation?.status === "running") Object.assign(this.state.simulation, { status: "complete", message: "Reached the test goal." });
      return "idle";
    }
    if (this.navDrive === "left" || this.navDrive === "right") {
      this.truth.yaw = wrapAngle(this.truth.yaw + (this.navDrive === "left" ? 1 : -1) * ANGULAR_SPEED * speed * dt);
      return "turning";
    }
    if (this.navDrive === "forward" || this.navDrive === "backward") {
      const distance = (this.navDrive === "forward" ? 1 : -1) * LINEAR_SPEED * speed * dt;
      // Full steer curves the mock's walk with a 0.4 m radius. The real robot's value has not been measured.
      if (this.navDrive === "forward") this.truth.yaw = wrapAngle(this.truth.yaw + this.navSteer * STEER_CURVE_RAD_PER_M * distance);
      this.advance(distance);
      return "moving";
    }
    return "idle";
  }

  private advance(distance: number) {
    const nx = this.truth.x + Math.cos(this.truth.yaw) * distance;
    const ny = this.truth.y + Math.sin(this.truth.yaw) * distance;
    const { width, length } = this.state.arena;
    const r = this.robot();
    const margin = Math.hypot(r.footprint.width, r.footprint.length) / 2;

    // The mock's table ends where the planner's limit for the robot's centre is, less half a centimetre. With the
    // robot's half-diagonal (9 cm) here and 7 cm in the planner, a goal on the limit could not be reached: the mock
    // robot pushed against an invisible wall and the navigator called it stuck.
    const edge = Math.min(margin, EDGE_MARGIN_M) - 0.005;
    const cx = clamp(nx, edge, width - edge);
    const cy = clamp(ny, edge, length - edge);
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
    if (!isWithinArmReach(arm.mount, this.truth)) {
      this.blockTest("Obstacle detected outside the arm's reach. No rescue attempted.");
      return;
    }

    const margin = Math.hypot(robot.footprint.width, robot.footprint.length) / 2;
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
    if (!drop) {
      this.blockTest("No clear landing point inside the arm's reach.");
      return;
    }

    const error = this.beginCarry(drop);
    if (error) this.blockTest(error);
  }

  /** Starts the pick-and-carry sequence to a drop point. Returns null when it has started, or why it cannot. */
  private beginCarry(drop: Point): string | null {
    const arm = this.state.arm;
    if (!arm) return "There is no arm.";
    if (arm.mode !== "idle") return "The arm is busy.";
    if (!isWithinArmReach(arm.mount, this.truth)) return "Sesame is outside the arm's reach.";
    const robot = this.robot();
    const margin = Math.hypot(robot.footprint.width, robot.footprint.length) / 2;
    const route = Array.from({ length: 13 }, (_, i) => ({ x: lerp(this.truth.x, drop!.x, i / 12), y: lerp(this.truth.y, drop!.y, i / 12) }));
    const crossed = this.state.obstacles.filter((o) => route.some((p) => distanceTo(o, p) < margin));
    const clearance = Math.max(0.06, ...crossed.map((o) => o.height ?? 0)) + 0.03;
    this.gripYaw = this.truth.yaw;
    const handles = sesameHandleTargets({ ...robot, yaw: this.gripYaw }).sort((a, b) =>
      Math.hypot(this.truth.x + a.offset.x - arm.mount.x, this.truth.y + a.offset.y - arm.mount.y)
      - Math.hypot(this.truth.x + b.offset.x - arm.mount.x, this.truth.y + b.offset.y - arm.mount.y));
    const handle = handles.find(({ offset }) => {
      let seed = arm.joints;
      const from = { x: this.truth.x + offset.x, y: this.truth.y + offset.y };
      const to = { x: drop!.x + offset.x, y: drop!.y + offset.y };
      const waypoints = [{ ...from, z: offset.z + 0.035, opening: 1 }, { ...from, z: offset.z, opening: 1 },
        { ...from, z: offset.z, opening: 0 },
        ...route.map((p) => ({ x: p.x + offset.x, y: p.y + offset.y, z: offset.z + clearance, opening: 0 })),
        { ...to, z: offset.z, opening: 0 }, { ...to, z: offset.z, opening: 1 }, { ...to, z: offset.z + 0.035, opening: 1 }];
      return waypoints.every((target) => {
        // Clamped to the joint limits exactly as the running sequence clamps them. Unclamped, a drop point that
        // needs a joint past its limit passed this check and then failed mid-carry.
        const ik = solveArmIK(arm.mount, target, target.z, seed, { yaw: this.gripYaw, opening: target.opening });
        seed = { waist: clampArmJoint("waist", ik.waist), shoulder: clampArmJoint("shoulder", ik.shoulder), elbow: clampArmJoint("elbow", ik.elbow),
          wristPitch: clampArmJoint("wristPitch", ik.wristPitch), wristRoll: clampArmJoint("wristRoll", ik.wristRoll), gripper: clampArmJoint("gripper", ik.gripper) };
        const reached = armTipPosition(arm.mount, seed);
        // Stricter than the 1.5 mm the running sequence demands of each phase. At 2 mm, a drop point could pass
        // here and then fail mid-carry, which depended on the heading Sesame happened to arrive with.
        return Math.hypot(reached.x - target.x, reached.y - target.y, reached.z - target.z) < 0.001;
      });
    });
    if (!handle) return "Handle pickup or clearance path is outside the URDF joint limits. Move the obstacle closer.";
    this.handleOffset = handle.offset;
    this.armFrom = { x: this.truth.x + handle.offset.x, y: this.truth.y + handle.offset.y };
    this.armTo = { x: drop.x + handle.offset.x, y: drop.y + handle.offset.y };
    this.carryZ = this.grabHeight() + clearance;
    this.gripOpening = clamp((arm.joints.gripper + 0.05) / 0.95, 0, 1);
    this.armElapsed = 0;
    // start the Cartesian smoothing from wherever the gripper currently sits
    const tip = armTipPosition(arm.mount, arm.joints);
    this.armPoint = { x: tip.x, y: tip.y };
    this.armZ = tip.z;
    this.phaseStart = tip;
    arm.mode = "reaching";
    arm.targetRobotId = robot.id;
    if (this.state.simulation) this.state.simulation.message = "Obstacle detected. Arm reaching for Sesame.";
    return null;
  }

  /** Advances the arm's phase timer, joint pose, and (while carrying) the robot's position. */
  private advanceArm(arm: NonNullable<WorldState["arm"]>, dt: number) {
    if (arm.mode === "idle") return;
    this.armElapsed += dt * 1000;
    const dur = ARM_PHASE_MS[arm.mode] ?? 1000;
    const t = easeInOut(clamp(this.armElapsed / (arm.mode === "grasping" ? dur * 0.6 : dur), 0, 1));
    const k = 1 - Math.exp(-8 * dt);

    // Move the tool along a time-parameterised Cartesian segment using the URDF chain.
    // Only advance phases once the actual tool reaches the requested endpoint.
    // Returning also follows a tool-space segment rather than swinging through the floor.
    // The carried robot is attached to the solved tool position, never a separate animation.
    const { point, z, gripper } = this.armPhaseTarget(arm);
    this.armPoint = { x: lerp(this.phaseStart.x, point.x, t), y: lerp(this.phaseStart.y, point.y, t) };
    this.armZ = lerp(this.phaseStart.z, z, t);
    if (arm.mode === "returning") {
      const retreatZ = this.phaseStart.z + 0.035;
      const u = easeInOut(clamp((this.armElapsed / dur - 0.35) / 0.65, 0, 1));
      this.armPoint = { x: lerp(this.phaseStart.x, point.x, u), y: lerp(this.phaseStart.y, point.y, u) };
      this.armZ = this.armElapsed < dur * 0.35
        ? lerp(this.phaseStart.z, retreatZ, easeInOut(this.armElapsed / (dur * 0.35))) : lerp(retreatZ, z, u);
    }
    const current = armTipPosition(arm.mount, arm.joints);
    const aligned = Math.hypot(current.x - point.x, current.y - point.y, current.z - z) < 0.0015;
    const opening = arm.mode === "grasping" && (!aligned || this.armElapsed < dur * 0.6) ? 1 : gripper;
    this.gripOpening += (opening - this.gripOpening) * k;
    const ik = solveArmIK(arm.mount, this.armPoint, this.armZ, arm.joints, { yaw: this.gripYaw, opening: this.gripOpening });
    arm.joints.waist = clampArmJoint("waist", ik.waist);
    arm.joints.shoulder = clampArmJoint("shoulder", ik.shoulder);
    arm.joints.elbow = clampArmJoint("elbow", ik.elbow);
    arm.joints.wristPitch = clampArmJoint("wristPitch", ik.wristPitch);
    arm.joints.wristRoll = clampArmJoint("wristRoll", ik.wristRoll);
    arm.joints.gripper = clampArmJoint("gripper", ik.gripper);
    const tip = armTipPosition(arm.mount, arm.joints);
    if (["lifting", "carrying", "placing"].includes(arm.mode)) {
      this.truth.x = tip.x - this.handleOffset.x;
      this.truth.y = tip.y - this.handleOffset.y;
      this.robot().z = Math.max(0, tip.z - this.grabHeight());
    }
    const error = Math.hypot(tip.x - point.x, tip.y - point.y, tip.z - z);
    if (this.armElapsed > dur + 4000 && (error > 0.0015 || Math.abs(this.gripOpening - gripper) >= 0.005)) {
      this.blockTest(`Arm could not reach the ${arm.mode} endpoint within joint limits.`);
      return;
    }
    if (this.armElapsed >= dur && error < 0.0015 && Math.abs(this.gripOpening - gripper) < 0.005) {
      this.armElapsed = 0;
      this.phaseStart = tip;
      const next = ARM_NEXT[arm.mode] ?? "idle";
      if (next === "releasing") this.robot().z = 0;
      arm.mode = next;
      if (this.state.simulation) this.state.simulation.message = `Arm: ${next}.`;
      if (next === "idle") {
        // Retain the solved resting pose rather than snapping between IK branches.
        arm.targetRobotId = undefined;
        this.armFrom = null;
        this.armTo = null;
        const command = this.active?.command;
        if (command?.type === "goto" && command.target) {
          this.navigator.carried(true);   // it plans the rest of the way from where the arm set the robot down
          this.state.path = [{ x: this.truth.x, y: this.truth.y }, { ...command.target }];  // until that plan exists
          if (this.state.simulation) this.state.simulation.message = "Carry complete. Continuing toward the destination.";
        } else {
          // The rescue is done; clear its command so the robot stays at the landing point.
          this.active = null;
          this.state.goal = undefined;
          this.state.path = undefined;
          if (this.state.simulation) Object.assign(this.state.simulation, { status: "complete", message: "Carry complete: Sesame placed beyond the obstacle." });
        }
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
      return { point: { x: tip.x, y: tip.y }, z: tip.z, gripper: 1 };
    }

    const point = ["carrying", "placing", "releasing"].includes(arm.mode)
      ? (this.armTo ?? this.truth) : (this.armFrom ?? this.truth);
    const z = arm.mode === "reaching" ? this.grabHeight() + 0.035
      : arm.mode === "lifting" || arm.mode === "carrying" ? this.carryZ : this.grabHeight();
    const gripper = arm.mode === "reaching" || arm.mode === "releasing" ? 1 : 0;

    return { point, z, gripper };
  }
}

function circleHitsObstacle(x: number, y: number, radius: number, o: Obstacle): boolean {
  if (o.shape === "circle") {
    return Math.hypot(x - o.x, y - o.y) < (o.radius ?? 0) + radius;
  }
  if (o.shape === "polygon" && o.points?.length) {
    return pointInPolygon({ x, y }, o.points) || distanceTo(o, { x, y }) < radius;
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
