// Tests for behaviours.mjs with a scripted table and robot: what Spidey decides to do by itself, and its diary.
import assert from "node:assert/strict";
import { Behaviours, isArmDetection } from "./behaviours.mjs";

function world(options) {
  const log = [];
  return { log, b: new Behaviours({ goto: (p) => log.push(["goto", p.x, p.y]), ignore: (o) => o.id === "the-arm" }, options) };
}
const robot = { x: 0.3, y: 0.3, yaw: 0, tracking: true };
const box = (id, x, y, label) => ({ id, label, source: "cv", shape: "rect", x, y, yaw: 0, width: 0.06, length: 0.06 });
const run = (b, from, to, state) => { for (let t = from; t <= to; t += 100) b.step(state, t); };
const green = box("green-box-1", 0.5, 0.5, "green box"), orange = box("orange-box-2", 0.35, 0.4, "orange box");

// The diary of the table: what was there at the start is not news. What comes, moves and goes later is.
let { b, log } = world();
run(b, 0, 3000, { robot, obstacles: [green], mission: { state: "idle" } });
assert.equal(b.diary.length, 0, "the box that was there from the start is not announced");
run(b, 3100, 6000, { robot, obstacles: [green, orange], mission: { state: "idle" } });
assert.equal(b.diary[0].text, "A orange box appeared.");
run(b, 6100, 7000, { robot, obstacles: [green, { ...orange, x: 0.47 }], mission: { state: "idle" } });
assert.equal(b.diary[0].text, "The orange box moved 12 cm.");
run(b, 7100, 8000, { robot, obstacles: [green], mission: { state: "idle" } });
assert.notEqual(b.diary[0].text, "The orange box is gone.", "one missed scan is not gone");
run(b, 8100, 12000, { robot, obstacles: [green], mission: { state: "idle" } });
assert.equal(b.diary[0].text, "The orange box is gone.");
assert.equal(log.length, 0, "with curious off Spidey stays where it is");
console.log("PASS  diary of the table: appeared, moved 12 cm, gone after 4 s, nothing about what was there from the start");

// The diary of the robot, by name.
({ b } = world());
const says = (mission) => { b.step({ robot, obstacles: [], mission }, (says.t = (says.t ?? 0) + 100)); return b.diary[0]?.text; };
says({ state: "idle" });
says({ state: "navigating" });
assert.equal(says({ state: "recovering" }), "Spidey got stuck and is backing off.");
assert.equal(says({ state: "navigating", via: { x: 0.2, y: 0.2 } }), "No way through. Spidey walks over to where Armie can reach.");
assert.equal(says({ state: "carrying" }), "No way through. Spidey asks Armie for a lift.");
assert.equal(says({ state: "navigating" }), "Armie set Spidey down. Walking on.");
assert.equal(says({ state: "done" }), "Spidey arrived.");
says({ state: "navigating" });
assert.equal(says({ state: "failed", detail: "no walkable path to the goal" }), "Spidey gave up: no walkable path to the goal.");
console.log("PASS  diary of the robot: stuck, walking to Armie, asking for a lift, set down, arrived, gave up");

// Curious: walks to a new object, looks at it, and is then free again. Armie's detection is never a thing.
({ b, log } = world({ curious: true }));
run(b, 0, 2000, { robot, obstacles: [green], mission: { state: "idle" } });
run(b, 2100, 4000, { robot, obstacles: [green, box("the-arm", 0.1, 0.3, "orange object")], mission: { state: "idle" } });
assert.equal(log.length, 0, "Armie appearing is ignored");
run(b, 4100, 6000, { robot, obstacles: [green, orange], mission: { state: "idle" } });
assert.deepEqual(log, [["goto", 0.35, 0.4]]);
assert.deepEqual(b.status().errand, { label: "orange box", looking: false });
run(b, 6100, 7000, { robot, obstacles: [green, orange], mission: { state: "navigating" } });
run(b, 7100, 7200, { robot, obstacles: [green, orange], mission: { state: "done" } });
assert.equal(b.diary[0].text, "Spidey had a good look at the orange box.");
assert.equal(b.status().errand.looking, true);
run(b, 7300, 10500, { robot, obstacles: [green, orange], mission: { state: "done" } });
assert.equal(b.status().errand, null);
assert.equal(log.length, 1, "it does not go back to the same object");
b.step({ robot, obstacles: [green, orange, box("blue-3", 0.1, 0.1, "blue box")], mission: { state: "done" } }, 10600);
run(b, 10700, 12500, { robot, obstacles: [green, orange, box("blue-3", 0.1, 0.1, "blue box")], mission: { state: "done" } });
assert.equal(log.length, 2);
b.interrupt();
assert.equal(b.status().errand, null, "a manual command ends the errand");
console.log("PASS  curious: walks to the new object once, looks for 2.5 s, ignores Armie, stops when interrupted");

assert.equal(isArmDetection({ shape: "rect", x: 0.167, y: 0.319, yaw: 1.708, width: 0.102, length: 0.444 }, { x: -0.074, y: 0.291 }), true);
assert.equal(isArmDetection({ shape: "rect", x: 0.103, y: 0.48, yaw: 1.917, width: 0.177, length: 0.167 }, { x: -0.074, y: 0.291 }), false);
assert.equal(isArmDetection(green, null), false);
console.log("PASS  Armie's own detection is told apart from a box next to it (live positions of 2026-09-19)");
