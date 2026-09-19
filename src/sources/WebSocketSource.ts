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
      if (msg.type === "state") this.stateSubs.forEach((cb) => cb(this.withArm(msg.data)));
      else if (msg.type === "ack") this.ackSubs.forEach((cb) => cb(msg.data));
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

    const { width, length } = state.arena;
    const edges = [
      { side: "south" as const, dist: tag.y },
      { side: "north" as const, dist: length - tag.y },
      { side: "west" as const, dist: tag.x },
      { side: "east" as const, dist: width - tag.x },
    ];
    const side = edges.reduce((a, b) => (b.dist < a.dist ? b : a)).side;

    return {
      ...state,
      obstacles: state.obstacles.filter((o) => o.id !== ARM_TAG_OBSTACLE_ID),
      arm: { mount: { x: tag.x, y: tag.y, yaw: tag.yaw, side }, joints: ARM_REST_POSE, mode: "idle" },
    };
  }
}
