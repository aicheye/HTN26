import type { Command } from "../types/world";

/**
 * Maps our Command schema onto the Sesame firmware JSON API.
 * Reference: dorianborian/sesame-robot, firmware/README.md
 *   GET  /api/status  -> { currentCommand, currentFace, networkConnected, apIP, networkIP }
 *   POST /api/command <- { command?: string, face?: string }
 */

export type SesamePayload = { command?: string; face?: string };

export type SesameStatus = {
  currentCommand: string;
  currentFace: string;
  networkConnected: boolean;
  apIP?: string;
  networkIP?: string;
};

/** Returns null for commands the firmware has no equivalent for (e.g. goto). */
export function toSesamePayload(c: Command): SesamePayload | null {
  switch (c.type) {
    case "forward":
    case "backward":
    case "left":
    case "right":
      return { command: c.type, face: c.face ?? "walk" };
    case "stop":
      return { command: "stop", ...(c.face ? { face: c.face } : {}) };
    case "pose":
      return c.pose ? { command: c.pose, face: c.face ?? c.pose } : null;
    case "face":
      return c.face ? { face: c.face } : null;
    // goto is planner-level; the navigation layer turns it into forward/left/right.
    case "goto":
      return null;
  }
}

/** Optional direct link to the robot over the LAN, used alongside any StateSource. */
export class SesameHttpBridge {
  constructor(private baseUrl: string) {}

  async send(c: Command): Promise<void> {
    const payload = toSesamePayload(c);
    if (!payload) return;
    try {
      await fetch(`${this.baseUrl}/api/command`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
    } catch {
      // The robot may be off the network; the UI keeps running on mock state.
    }
  }

  async status(): Promise<SesameStatus | null> {
    try {
      const res = await fetch(`${this.baseUrl}/api/status`);
      return (await res.json()) as SesameStatus;
    } catch {
      return null;
    }
  }
}
