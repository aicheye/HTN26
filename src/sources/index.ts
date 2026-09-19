import { MockSource } from "./MockSource";
import { WebSocketSource } from "./WebSocketSource";
import type { StateSource } from "./StateSource";

export type SourceKind = "mock" | "ws";

export const DEFAULT_SOURCE: SourceKind =
  import.meta.env.VITE_SOURCE === "ws" ? "ws" : "mock";

export const WS_URL: string =
  import.meta.env.VITE_WS_URL ?? "ws://localhost:8080/ws";

export function createSource(kind: SourceKind): StateSource {
  return kind === "ws" ? new WebSocketSource(WS_URL) : new MockSource();
}

export { MockSource, WebSocketSource };
export type { StateSource, ConnectionStatus } from "./StateSource";
