import { ARM_REST_POSE, distanceTo } from "../components/mapShared";
import type { Ack, Command, Envelope, WorldState } from "../types/world";
import type { ConnectionStatus, StateSource } from "./StateSource";

/** id the bridge gives the arm's own tracked-tag obstacle (bridge/bridge.mjs). */
const ARM_TAG_OBSTACLE_ID = "so101-base";
// A camera-detected object is the arm itself when its footprint reaches the arm's base: within 3 cm of the mount,
// or within the 9 cm base radius of the tracked tag, which sits on the base. On 2026-09-19 the arm's detection
// covered the mount (0 cm) and was 2 cm from the tag. A real box beside the arm was 7.6 cm and 15 cm away.
const ARM_AT_MOUNT_M = 0.03;
const ARM_AT_TAG_M = 0.09;

/** Real backend: expects Envelope JSON frames over a WebSocket. Unused until the camera exists. */
export class WebSocketSource implements StateSource {
  private ws: WebSocket | null = null;
  private queue: Command[] = [];
  private retry: number | null = null;

  private stateSubs = new Set<(s: WorldState) => void>();
  private ackSubs = new Set<(a: Ack) => void>();
  private statusSubs = new Set<(s: ConnectionStatus) => void>();

  constructor(private url: string) {}

  subscribe(cb: (s: WorldState) => void) {
    this.stateSubs.add(cb);
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
    return () => this.statusSubs.delete(cb);
  }

  start() {
    if (this.ws) return;
    this.status("connecting");
    const ws = new WebSocket(this.url);
    this.ws = ws;

    ws.onopen = () => {
      this.status("live");
      this.queue.splice(0).forEach((c) => this.sendCommand(c));
    };
    ws.onmessage = (ev) => {
      let msg: Envelope;
      try {
        msg = JSON.parse(ev.data as string) as Envelope;
      } catch {
        return;
      }
      if (msg.type === "state") {
        if (!isWorldState(msg.data)) {
          console.warn("WebSocketSource: dropped malformed state frame", msg.data);
          return;
        }
        this.stateSubs.forEach((cb) => cb(withArm(msg.data)));
      } else if (msg.type === "ack") this.ackSubs.forEach((cb) => cb(msg.data));
    };
    ws.onerror = () => this.status("error");
    ws.onclose = () => {
      this.ws = null;
      this.status("closed");
      if (this.stateSubs.size > 0 && this.retry === null) {
        this.retry = window.setTimeout(() => {
          this.retry = null;
          this.start();
        }, 1000);
      }
    };
  }

  stop() {
    if (this.retry !== null) window.clearTimeout(this.retry);
    this.retry = null;
    this.ws?.close();
    this.ws = null;
  }

  sendCommand(c: Command, queueIfDisconnected = true) {
    if (this.ws?.readyState === WebSocket.OPEN) {
      const envelope: Envelope = { type: "command", data: c };
      this.ws.send(JSON.stringify(envelope));
    } else if (queueIfDisconnected) {
      this.queue.push(c);
    } else {
      this.ackSubs.forEach((cb) => cb({ commandId: c.id, ok: false, error: "Not connected; voice action was not queued." }));
      return false;
    }
    return true;
  }

  private status(s: ConnectionStatus) {
    this.statusSubs.forEach((cb) => cb(s));
  }

}

/**
 * The bridge only reports the arm's tracked base as a plain circle obstacle -
 * it has no joint telemetry. Turn that into a resting ArmState so the
 * articulated 3D model still shows up (just not animated) when live.
 */
export function withArm(state: WorldState): WorldState {
  if (state.arm) return state;
  const tag = state.obstacles.find((o) => o.id === ARM_TAG_OBSTACLE_ID);
  if (!tag) return state;

  // The mount is bolted to the centre of one table edge. The tracked tag position is noisy, so it only picks
  // the edge, the one it is nearest to, and the mount snaps to that edge's exact centre.
  const { width, length } = state.arena;
  const edges = [
    { distance: Math.abs(tag.x), mount: { x: 0, y: length / 2, yaw: 0, side: "west" as const } },
    { distance: Math.abs(width - tag.x), mount: { x: width, y: length / 2, yaw: Math.PI, side: "east" as const } },
    { distance: Math.abs(tag.y), mount: { x: width / 2, y: 0, yaw: Math.PI / 2, side: "south" as const } },
    { distance: Math.abs(length - tag.y), mount: { x: width / 2, y: length, yaw: -Math.PI / 2, side: "north" as const } },
  ];
  const mount = edges.reduce((nearest, edge) => (edge.distance < nearest.distance ? edge : nearest)).mount;

  // The camera also detects the arm as an object. The arm model already shows it, so that detection is not drawn
  // a second time. The bridge keeps it in its own obstacle list, where path planning still avoids it.
  const isArm = (o: WorldState["obstacles"][number]) =>
    o.id === ARM_TAG_OBSTACLE_ID || (o.source === "cv" && (distanceTo(o, mount) < ARM_AT_MOUNT_M || distanceTo(o, tag) < ARM_AT_TAG_M));

  return {
    ...state,
    obstacles: state.obstacles.filter((o) => !isArm(o)),
    hiddenObstacles: state.obstacles.filter((o) => isArm(o) && o.id !== ARM_TAG_OBSTACLE_ID).map((o) => ({ ...o, label: `the arm (${o.label ?? o.id})` })),
    // The bridge has no joint telemetry, so the arm is drawn at rest. Its mode still says when it has been asked
    // to carry the robot past an obstacle, which both maps show as a label.
    arm: state.mission?.state === "carrying"
      ? { mount, joints: ARM_REST_POSE, mode: "carrying", targetRobotId: state.robots[0]?.id }
      : { mount, joints: ARM_REST_POSE, mode: "idle" },
  };
}

/**
 * Just enough shape-checking to keep a malformed/partial frame from crashing the
 * renderer (e.g. a bridge restart sending a half-written payload). Not a full
 * schema validator - just the fields Map2D/Map3D/StatusPanel dereference directly.
 */
function isWorldState(data: unknown): data is WorldState {
  if (!data || typeof data !== "object") return false;
  const d = data as Record<string, unknown>;
  const arena = d.arena as Record<string, unknown> | undefined;
  return (
    typeof arena === "object" &&
    arena !== null &&
    typeof arena.width === "number" &&
    typeof arena.length === "number" &&
    Array.isArray(d.robots) &&
    Array.isArray(d.obstacles)
  );
}
