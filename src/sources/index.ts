import { MockSource } from "./MockSource";
import { WebSocketSource } from "./WebSocketSource";
import type { StateSource } from "./StateSource";

export type SourceKind = "mock" | "ws";

export const DEFAULT_SOURCE: SourceKind =
  import.meta.env.VITE_SOURCE === "mock" ? "mock" : "ws";

const WS_URL_STORAGE_KEY = "sesame:wsUrl";
const DEFAULT_WS_URL = import.meta.env.VITE_WS_URL ?? "ws://localhost:8080/ws";

/** Reads the user's saved WS URL (Settings panel), falling back to the build-time default. */
export function getWsUrl(): string {
  try {
    return localStorage.getItem(WS_URL_STORAGE_KEY) || DEFAULT_WS_URL;
  } catch {
    return DEFAULT_WS_URL;
  }
}

/** Persists a WS URL override so it survives reloads; pass "" to clear back to the default. */
export function setWsUrl(url: string) {
  try {
    if (url) localStorage.setItem(WS_URL_STORAGE_KEY, url);
    else localStorage.removeItem(WS_URL_STORAGE_KEY);
  } catch {
    // localStorage unavailable (private mode etc.) - the URL just won't persist
  }
}

export function createSource(kind: SourceKind, wsUrl?: string): StateSource {
  return kind === "ws" ? new WebSocketSource(wsUrl ?? getWsUrl()) : new MockSource();
}

export { MockSource, WebSocketSource };
export type { StateSource, ConnectionStatus } from "./StateSource";
