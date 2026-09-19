import { cloneSampleWorldState } from "../data/sampleWorldState";
import type {
  Ack,
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

    let mode: Robot["mode"] = "idle";
    if (cmd && !posing) mode = this.step(cmd, dt);

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
    if (this.blocked(cx, cy, margin)) return;
    this.truth.x = cx;
    this.truth.y = cy;
  }

  private blocked(x: number, y: number, radius: number): boolean {
    return this.state.obstacles.some((o) => circleHitsObstacle(x, y, radius, o));
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
