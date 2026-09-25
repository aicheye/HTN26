import type { Ack, Command, WorldState } from "../types/world";

export type ConnectionStatus = "connecting" | "live" | "closed" | "error";

export interface StateSource {
  /** Returns an unsubscribe function. */
  subscribe(cb: (s: WorldState) => void): () => void;
  sendCommand(c: Command, queueIfDisconnected?: boolean): boolean | void;
  onAck?(cb: (a: Ack) => void): () => void;
  onStatus?(cb: (s: ConnectionStatus) => void): () => void;
  start?(): void;
  stop?(): void;
}
