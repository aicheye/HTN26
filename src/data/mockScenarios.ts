import { cloneSampleWorldState } from "./sampleWorldState";
import type { Obstacle, Point, WorldState } from "../types/world";

export const MOCK_SCENARIOS = [
  { id: "barrier", label: "Carry over barrier" },
  { id: "mixed", label: "Mixed detections" },
  { id: "unreachable", label: "Outside arm reach" },
  { id: "empty", label: "Empty tabletop" },
] as const;
export type MockScenarioId = typeof MOCK_SCENARIOS[number]["id"];

export function makeMockScenario(id: MockScenarioId): WorldState {
  const state = cloneSampleWorldState();
  const barrier: Obstacle = { id: "detected-blue-barrier", source: "cv", shape: "rect", x: 0.215, y: 0.32,
    yaw: 0, width: 0.18, length: 0.035, height: 0.065, color: "#287cb5", confidence: 0.96 };
  let start: Point = { x: 0.215, y: 0.135 };
  let goal: Point = { x: 0.215, y: 0.48 };
  state.obstacles = id === "empty" ? [] : [barrier];
  if (id === "mixed") state.obstacles.push(
    { id: "detected-red-cylinder", source: "cv", shape: "circle", x: 0.46, y: 0.19, yaw: 0, radius: 0.035, height: 0.09, color: "#bc4a41", confidence: 0.9 },
    { id: "detected-yellow-block", source: "cv", shape: "polygon", x: 0.45, y: 0.46, yaw: 0,
      points: [{ x: 0.41, y: 0.42 }, { x: 0.485, y: 0.42 }, { x: 0.5, y: 0.475 }, { x: 0.425, y: 0.49 }],
      height: 0.045, color: "#d5a332", confidence: 0.87 },
  );
  if (id === "unreachable") {
    barrier.x = 0.5;
    start = { x: 0.5, y: 0.135 };
    goal = { x: 0.5, y: 0.48 };
  }
  if (id === "empty") start = { x: 0.3175, y: 0.3175 };
  Object.assign(state.robots[0], start, { yaw: Math.PI / 2, z: 0, mode: "idle", footprint: { width: 0.134, length: 0.125 } });
  state.simulation = { scenario: id, status: "ready", testGoal: goal,
    message: id === "empty" ? "Click the map to walk." : "Run the test: walk, detect blockage, pick up, carry, and place." };
  return state;
}
