// Drives the Sesame robot to a goal using only the firmware's gaits (forward, backward, left, right, stop)
// and the camera pose. All units metres, radians, milliseconds. step() is called about 10 times per second.
//
// The firmware finishes a gait with a stand pose every time the command changes, so each switch between
// turning and walking costs time. The controller therefore switches as rarely as it can:
//  - it starts a turn only when the heading error is large, and ends it only when the error is small
//  - it ends a turn or a walk early by the distance the robot keeps moving after "stop" (measured by calibrate())
//  - it walks backward to a close target behind it, which is faster than turning around
//  - it aims slightly against the robot's measured sideways drift
import { planPath } from "./planner.mjs";

export const DEFAULT_MOTION = {
  walkSpeed: 0.04,      // m/s
  turnRate: 0.5,        // rad/s
  veer: 0,              // rad of heading change per metre walked, positive = drifts counter-clockwise
  turnStopLead: 0.15,   // rad the robot keeps turning after "stop" is sent
  walkStopLead: 0.01,   // m the robot keeps walking after "stop" is sent
};

const TURN_START = 0.45;        // start turning in place above this heading error
const TURN_DONE = 0.1;          // a turn is finished when the predicted error is below this
const WAYPOINT_REACHED = 0.06;
const GOAL_REACHED = 0.04;
const REVERSE_ANGLE = 2.4;      // target is behind the robot
const REVERSE_DISTANCE = 0.15;  // and this close: walk backward
const REPLAN_MS = 1000;
const RESEND_MS = 2000;
const STUCK_WINDOW_MS = 3000;
const STUCK_MIN_MOVE = 0.01;
const STUCK_MIN_TURN = 0.09;
const RECOVER_BACK_MS = 1200;
const RECOVER_TURN_MS = 1200;
const MAX_RECOVERIES = 3;
const MAX_FAILED_PLANS = 3;
const LOST_TIMEOUT_MS = 10000;
const NO_PROGRESS_MS = 30000;   // give up when the remaining route has not shrunk by this much in this long
const NO_PROGRESS_MIN = 0.03;

const wrap = (angle) => Math.atan2(Math.sin(angle), Math.cos(angle));

export class Navigator {
  constructor(send, motion = {}) {
    this.send = send;  // (command string) => void
    this.motion = { ...DEFAULT_MOTION, ...motion };
    this.robotRadius = undefined;  // metres, set by the bridge once the camera has measured the robot
    this.state = "idle";  // idle, navigating, recovering, calibrating, done, failed
    this.detail = "";
    this.goal = null;
    this.path = [];
    this.drive = "";
    this.recoveries = 0;
  }

  status() {
    return { state: this.state, detail: this.detail, recoveries: this.recoveries, motion: this.motion };
  }

  start(goal) {
    Object.assign(this, { goal, path: [], state: "navigating", detail: "", recoveries: 0, failedPlans: 0, plannedAt: 0, lostSince: 0, progress: null, watch: null });
  }

  cancel() {
    if (this.state === "navigating" || this.state === "recovering") this.setDrive("stop");
    Object.assign(this, { goal: null, path: [], state: "idle", detail: "", drive: "" });
    this.calibration = null;
  }

  finish(state, detail) {
    this.setDrive("stop");
    Object.assign(this, { state, detail, goal: null, path: [], drive: "" });
  }

  setDrive(command, now = Date.now()) {
    if (command === this.drive && now - this.sentAt < RESEND_MS) return;
    if (command !== this.drive) this.progress = null;
    this.send(command);
    this.drive = command;
    this.sentAt = now;
  }

  step(robot, arena, obstacles, now = Date.now()) {
    if (this.state === "calibrating") return this.stepCalibration(robot, now);
    if (this.state !== "navigating" && this.state !== "recovering") return;
    if (!robot?.tracking) {
      this.setDrive("stop", now);
      this.lostSince ||= now;
      if (now - this.lostSince > LOST_TIMEOUT_MS) this.finish("failed", "robot not visible to the camera");
      return;
    }
    this.lostSince = 0;
    if (this.state === "recovering") return this.stepRecovery(now);
    if (this.isStuck(robot, now)) return this.startRecovery(now);

    if (now - this.plannedAt > REPLAN_MS || this.path.length === 0) {
      const planned = planPath(robot, this.goal, arena, obstacles, this.robotRadius);
      this.plannedAt = now;
      if (!planned) {
        if (++this.failedPlans >= MAX_FAILED_PLANS) this.finish("failed", "no walkable path to the goal");
        else this.setDrive("stop", now);
        return;
      }
      this.failedPlans = 0;
      this.path = planned.slice(1);
      this.detail = planned.goalMoved ? "goal is against a wall or obstacle; heading for the nearest reachable spot" : "";
    }

    // Advance past waypoints that are reached, counting the distance the robot still covers after a stop.
    while (this.path.length > 0) {
      const last = this.path.length === 1, target = this.path[0];
      const reach = (last ? GOAL_REACHED : WAYPOINT_REACHED) + (this.drive === "forward" || this.drive === "backward" ? this.motion.walkStopLead : 0);
      if (Math.hypot(target.x - robot.x, target.y - robot.y) > reach) break;
      this.path.shift();
    }
    if (this.path.length === 0) return this.finish("done", "");

    // Fail instead of walking forever when the route to the goal stops getting shorter.
    let remaining = Math.hypot(this.path[0].x - robot.x, this.path[0].y - robot.y);
    for (let i = 1; i < this.path.length; i++) remaining += Math.hypot(this.path[i].x - this.path[i - 1].x, this.path[i].y - this.path[i - 1].y);
    if (!this.watch || remaining < this.watch.best - NO_PROGRESS_MIN) this.watch = { best: remaining, at: now };
    else if (now - this.watch.at > NO_PROGRESS_MS) return this.finish("failed", "not making progress toward the goal");

    const target = this.path[0], distance = Math.hypot(target.x - robot.x, target.y - robot.y);
    // Aim against the drift the robot will pick up over the next stretch of walking.
    const aim = Math.atan2(target.y - robot.y, target.x - robot.x) - (this.motion.veer * Math.min(distance, 0.3)) / 2;
    const error = wrap(aim - robot.yaw);
    const behind = Math.abs(error) > REVERSE_ANGLE && distance < REVERSE_DISTANCE;

    if (this.drive === "left" || this.drive === "right") {
      // Still turning: stop early by the amount the robot keeps turning after the command.
      const remaining = Math.abs(error) - this.motion.turnStopLead;
      const overshot = (this.drive === "left") !== (error > 0);
      if (remaining > TURN_DONE && !overshot) return this.setDrive(this.drive, now);
    }
    if (behind) return this.setDrive("backward", now);
    if (Math.abs(error) > TURN_START) return this.setDrive(error > 0 ? "left" : "right", now);
    this.setDrive("forward", now);
  }

  // The robot is stuck when a movement command has been active for a while and the camera sees no movement.
  isStuck(robot, now) {
    if (!["forward", "backward", "left", "right"].includes(this.drive)) return false;
    this.progress ??= { since: now, x: robot.x, y: robot.y, turned: 0, yaw: robot.yaw };
    this.progress.turned += Math.abs(wrap(robot.yaw - this.progress.yaw));
    this.progress.yaw = robot.yaw;
    if (now - this.progress.since < STUCK_WINDOW_MS) return false;
    const moved = Math.hypot(robot.x - this.progress.x, robot.y - this.progress.y);
    const turning = this.drive === "left" || this.drive === "right";
    const stuck = turning ? this.progress.turned < STUCK_MIN_TURN : moved < STUCK_MIN_MOVE;
    this.progress = null;
    return stuck;
  }

  startRecovery(now) {
    if (this.recoveries >= MAX_RECOVERIES) return this.finish("failed", `stuck, gave up after ${MAX_RECOVERIES} recoveries`);
    this.recoveries++;
    this.state = "recovering";
    this.detail = `stuck while driving ${this.drive}, recovery ${this.recoveries} of ${MAX_RECOVERIES}`;
    this.recoverUntil = now + RECOVER_BACK_MS + RECOVER_TURN_MS;
    // Back away from whatever blocked a forward walk, then turn, alternating sides between attempts.
    this.recoverFirst = this.drive === "backward" ? "forward" : "backward";
    this.recoverTurn = this.recoveries % 2 ? "left" : "right";
  }

  stepRecovery(now) {
    if (now >= this.recoverUntil) {
      Object.assign(this, { state: "navigating", path: [], plannedAt: 0, progress: null });
      return this.setDrive("stop", now);
    }
    this.setDrive(this.recoverUntil - now > RECOVER_TURN_MS ? this.recoverFirst : this.recoverTurn, now);
  }

  // Measures the robot's real motion with the camera: walk, stop, turn left, stop, turn right, stop.
  // Needs about 30 cm of free floor ahead of the robot. Resolves with the measured motion model.
  calibrate(seconds = 5) {
    const phases = ["forward", "left", "right"].flatMap((command) => [{ command, ms: seconds * 1000 }, { command: "stop", ms: 2000, after: command }]);
    this.calibration = { phases, index: -1, results: {} };
    Object.assign(this, { state: "calibrating", detail: "", goal: null, path: [] });
    return new Promise((resolve, reject) => Object.assign(this.calibration, { resolve, reject }));
  }

  stepCalibration(robot, now) {
    const c = this.calibration;
    if (!robot?.tracking) {
      c.reject(new Error("robot not visible to the camera"));
      return this.finish("failed", "calibration aborted: robot not visible to the camera");
    }
    if (c.phase) {  // accumulate turning as small wrapped steps, so more than half a turn is measured correctly
      c.turned += wrap(robot.yaw - c.yaw);
      c.yaw = robot.yaw;
    }
    if (c.phase && now < c.until) return;

    if (c.phase) {
      const moved = Math.hypot(robot.x - c.from.x, robot.y - c.from.y), seconds = c.phase.ms / 1000;
      if (c.phase.command === "forward") Object.assign(c.results, { walkSpeed: moved / seconds, veer: moved > 0.02 ? c.turned / moved : 0 });
      if (c.phase.command === "left") c.results.turnRateLeft = c.turned / seconds;
      if (c.phase.command === "right") c.results.turnRateRight = -c.turned / seconds;
      if (c.phase.after === "forward") c.results.walkStopLead = moved;
      if (c.phase.after === "left") c.results.turnStopLeadLeft = Math.abs(c.turned);
      if (c.phase.after === "right") c.results.turnStopLeadRight = Math.abs(c.turned);
    }
    c.phase = c.phases[++c.index];
    if (!c.phase) {
      const r = c.results;
      this.motion = {
        walkSpeed: r.walkSpeed, veer: r.veer, walkStopLead: r.walkStopLead,
        turnRate: (r.turnRateLeft + r.turnRateRight) / 2,
        turnStopLead: Math.max(r.turnStopLeadLeft, r.turnStopLeadRight),
      };
      Object.assign(this, { state: "idle", detail: "calibrated", drive: "" });
      return c.resolve({ ...this.motion, ...r });
    }
    Object.assign(c, { until: now + c.phase.ms, from: { x: robot.x, y: robot.y }, yaw: robot.yaw, turned: 0 });
    this.send(c.phase.command);
    this.drive = c.phase.command;
    this.sentAt = now;
  }
}
