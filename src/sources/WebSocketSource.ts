import { ARM_REST_POSE } from "../components/mapShared";
import type { Ack, Command, Envelope, WorldState } from "../types/world";
import type { ConnectionStatus, StateSource } from "./StateSource";

/** id the bridge gives the arm's own tracked-tag obstacle (bridge/bridge.mjs). */
const ARM_TAG_OBSTACLE_ID = "so101-base";

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
        this.stateSubs.forEach((cb) => cb(this.withArm(msg.data)));
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

  sendCommand(c: Command) {
    if (this.ws?.readyState === WebSocket.OPEN) {
      const envelope: Envelope = { type: "command", data: c };
      this.ws.send(JSON.stringify(envelope));
    } else {
      this.queue.push(c);
    }
  }

  private status(s: ConnectionStatus) {
    this.statusSubs.forEach((cb) => cb(s));
  }

  /**
   * The bridge only reports the arm's tracked base as a plain circle obstacle -
   * it has no joint telemetry. Turn that into a resting ArmState so the
   * articulated 3D model still shows up (just not animated) when live.
   */
  private withArm(state: WorldState): WorldState {
    if (state.arm) return state;
    const tag = state.obstacles.find((o) => o.id === ARM_TAG_OBSTACLE_ID);
    if (!tag) return state;

    // real mount is bolted to the center of one of the arena's longer edges;
    // the tracked tag position is noisy, so only use it to pick which long
    // edge it's on, then snap to that edge's exact center
    const { width, length } = state.arena;
    const longIsNorthSouth = width >= length;
    const mount = longIsNorthSouth
      ? tag.y <= length / 2
        ? { x: width / 2, y: 0, yaw: Math.PI / 2, side: "south" as const }
        : { x: width / 2, y: length, yaw: -Math.PI / 2, side: "north" as const }
      : tag.x <= width / 2
        ? { x: 0, y: length / 2, yaw: 0, side: "west" as const }
        : { x: width, y: length / 2, yaw: Math.PI, side: "east" as const };

    return {
      ...state,
      obstacles: state.obstacles.filter((o) => o.id !== ARM_TAG_OBSTACLE_ID),
      arm: { mount, joints: ARM_REST_POSE, mode: "idle" },
    };
  }
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
