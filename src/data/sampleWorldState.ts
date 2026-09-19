import type { WorldState } from "../types/world";
import raw from "./sampleWorldState.json";

/** Static snapshot the MockSource starts from, and the handoff example for the camera side. */
export const sampleWorldState = raw as unknown as WorldState;

export function cloneSampleWorldState(): WorldState {
  return JSON.parse(JSON.stringify(sampleWorldState)) as WorldState;
}
