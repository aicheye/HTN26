import { cloneSampleWorldState } from "./sampleWorldState";
import type { Obstacle, Point, WorldState } from "../types/world";

export const MOCK_SCENARIOS = [
  { id: "barrier", label: "Carry over barrier" },
  { id: "pickup", label: "Walk to the arm, then carry" },
  { id: "detour", label: "Walk around a box" },
  { id: "mixed", label: "Mixed detections" },
  { id: "unreachable", label: "Outside arm reach" },
  { id: "empty", label: "Empty tabletop" },
] as const;
export type MockScenarioId = typeof MOCK_SCENARIOS[number]["id"];

export function makeMockScenario(id: MockScenarioId): WorldState {
  const state = cloneSampleWorldState();
  // The mock walks with the bridge's planner, which goes around anything it can. The barrier therefore spans the
  // whole table: no way round, so the arm has to lift Sesame over it.
  const barrier: Obstacle = { id: "detected-blue-barrier", source: "cv", shape: "rect", x: 0.3175, y: 0.32,
    yaw: 0, width: 0.635, length: 0.035, height: 0.065, color: "#287cb5", confidence: 0.96 };
  let start: Point = { x: 0.215, y: 0.135 };
  let goal: Point = { x: 0.215, y: 0.48 };
  state.obstacles = id === "empty" ? [] : [barrier];
  if (id === "mixed") state.obstacles.push(
    { id: "detected-red-cylinder", source: "cv", shape: "circle", x: 0.46, y: 0.19, yaw: 0, radius: 0.035, height: 0.09, color: "#bc4a41", confidence: 0.9 },
    { id: "detected-yellow-block", source: "cv", shape: "polygon", x: 0.45, y: 0.46, yaw: 0,
      points: [{ x: 0.41, y: 0.42 }, { x: 0.485, y: 0.42 }, { x: 0.5, y: 0.475 }, { x: 0.425, y: 0.49 }],
      height: 0.045, color: "#d5a332", confidence: 0.87 },
  );
  if (id === "pickup") {
    // Sesame starts 0.53 m from the arm's mount, which reaches 0.31 m: it has to walk to the arm before the lift.
    start = { x: 0.5, y: 0.135 };
    goal = { x: 0.5, y: 0.48 };
  }
  if (id === "detour") {
    // A box with room beside it: the planner walks round, and the arm stays idle.
    Object.assign(barrier, { x: 0.215, width: 0.18 });
  }
  if (id === "unreachable") {
    // The barrier runs the other way and shuts Sesame in on the far side of the table, where the arm does not
    // reach. There is nothing to walk to, so the test must end blocked, with the arm idle.
    Object.assign(barrier, { x: 0.4, y: 0.3175, width: 0.035, length: 0.635 });
    start = { x: 0.55, y: 0.135 };
    goal = { x: 0.2, y: 0.48 };
  }
  if (id === "empty") start = { x: 0.3175, y: 0.3175 };
  Object.assign(state.robots[0], start, { yaw: Math.PI / 2, z: 0, mode: "idle", footprint: { width: 0.134, length: 0.125 } });
  state.simulation = { scenario: id, status: "ready", testGoal: goal,
    message: id === "empty" ? "Click the map to walk." : id === "detour" ? "Run the test: plan a path round the box and walk it."
      : id === "pickup" ? "Run the test: no way through, walk to the arm, get carried over, walk on."
      : "Run the test: no way through, pick up, carry, place, and walk on." };
  return state;
}
