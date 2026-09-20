// Tests for behaviours.mjs with a scripted table and robot: what the robot decides to do by itself.
import assert from "node:assert/strict";
import { Behaviours, isArmDetection } from "./behaviours.mjs";

function world(options) {
  const log = [];
  const hooks = { goto: (p) => log.push(["goto", p.x, p.y]), face: (f) => log.push(["face", f]), pose: (p) => log.push(["pose", p]), ignore: (o) => o.id === "the-arm" };
  return { log, b: new Behaviours(hooks, options) };
}
const robot = { x: 0.3, y: 0.3, yaw: 0, tracking: true };
const box = (id, x, y, label) => ({ id, label, source: "cv", shape: "rect", x, y, yaw: 0, width: 0.06, length: 0.06 });
const run = (b, from, to, state) => { for (let t = from; t <= to; t += 100) b.step(typeof state === "function" ? state(t) : state, t); };

// Moods follow the mission.
let { b, log } = world();
run(b, 0, 300, { robot, obstacles: [], mission: { state: "idle" } });
b.step({ robot, obstacles: [], mission: { state: "navigating" } }, 400);
b.step({ robot, obstacles: [], mission: { state: "recovering" } }, 500);
b.step({ robot, obstacles: [], mission: { state: "navigating" } }, 600);
b.step({ robot, obstacles: [], mission: { state: "done" } }, 700);
assert.deepEqual(log, [["face", "excited"], ["face", "angry"], ["face", "happy"], ["pose", "wave"]]);
b.step({ robot, obstacles: [], mission: { state: "navigating" } }, 800);
b.step({ robot, obstacles: [], mission: { state: "failed", detail: "no walkable path to the goal" } }, 900);
assert.deepEqual(log.slice(-2), [["face", "sad"], ["pose", "shrug"]]);
assert.match(b.diary[0].text, /Gave up: no walkable path/);
run(b, 1000, 47000, { robot, obstacles: [], mission: { state: "failed" } });
assert.deepEqual(log.at(-1), ["face", "sleepy"]);
assert.equal(log.filter((l) => l[1] === "sleepy").length, 1, "it dozes off once, not on every tick");
console.log("PASS  moods: excited, angry, happy with a wave, sad with a shrug, sleepy after 45 s");

({ b, log } = world({ moods: false }));
b.step({ robot, obstacles: [], mission: { state: "navigating" } }, 0);
b.step({ robot, obstacles: [], mission: { state: "done" } }, 100);
assert.deepEqual(log, []);
console.log("PASS  moods off: no faces, no poses");

// The diary: what was there at the start is not news. What comes, moves and goes later is.
({ b, log } = world());
const green = box("green-box-1", 0.5, 0.5, "green box");
run(b, 0, 3000, { robot, obstacles: [green], mission: { state: "idle" } });
assert.equal(b.diary.length, 0, "the box that was there from the start is not announced");
const orange = box("orange-box-2", 0.35, 0.4, "orange box");
run(b, 3100, 6000, { robot, obstacles: [green, orange], mission: { state: "idle" } });
assert.equal(b.diary[0].text, "A orange box appeared.".replace("A o", "A o"));
assert.deepEqual(log.at(-1), ["face", "surprised"], "it landed 11 cm from the robot");
run(b, 6100, 7000, { robot, obstacles: [green, { ...orange, x: 0.47 }], mission: { state: "idle" } });
assert.equal(b.diary[0].text, "The orange box moved 12 cm.");
run(b, 7100, 8000, { robot, obstacles: [green], mission: { state: "idle" } });
assert.notEqual(b.diary[0].text, "The orange box is gone.", "one missed scan is not gone");
run(b, 8100, 12000, { robot, obstacles: [green], mission: { state: "idle" } });
assert.equal(b.diary[0].text, "The orange box is gone.");
console.log("PASS  diary: appeared, moved 12 cm, gone after 4 s, and nothing about what was there from the start");

// Curious: walks to a new object, looks at it, and is then free again. The arm's detection is never a thing.
({ b, log } = world({ curious: true, moods: false }));
run(b, 0, 2000, { robot, obstacles: [green], mission: { state: "idle" } });
run(b, 2100, 4000, { robot, obstacles: [green, box("the-arm", 0.1, 0.3, "orange object")], mission: { state: "idle" } });
assert.equal(log.length, 0, "the arm appearing is ignored");
run(b, 4100, 6000, { robot, obstacles: [green, orange], mission: { state: "idle" } });
assert.deepEqual(log, [["goto", 0.35, 0.4]]);
assert.equal(b.status().errand.label, "orange box");
run(b, 6100, 7000, { robot, obstacles: [green, orange], mission: { state: "navigating" } });
run(b, 7100, 7200, { robot, obstacles: [green, orange], mission: { state: "done" } });
assert.match(b.diary[0].text, /Had a look at the orange box/);
run(b, 7300, 10500, { robot, obstacles: [green, orange], mission: { state: "done" } });
assert.equal(b.status().errand, null);
assert.equal(log.length, 1, "it does not go back to the same object");
console.log("PASS  curious: walks to the new object once, looks for 2.5 s, ignores the arm");

// Tour: every object once, nearest first, skips one that cannot be reached and one that is taken away.
({ b, log } = world());
const far = box("blue-object-3", 0.55, 0.1, "blue object"), gone = box("white-box-4", 0.1, 0.55, "white box");
let objects = [far, green, orange, gone];
run(b, 0, 2000, { robot, obstacles: objects, mission: { state: "idle" } });
assert.equal(b.startTour(objects, robot, 2000), true);
let mission = { state: "idle" };
const tick = (t) => b.step({ robot, obstacles: objects, mission }, t);
tick(2100);
assert.deepEqual(log.at(-1), ["goto", 0.35, 0.4], "the orange box is nearest");
mission = { state: "navigating" }; tick(2200);
mission = { state: "done" }; tick(2300);
for (let t = 2400; t <= 5000; t += 100) tick(t);
assert.deepEqual(log.at(-1), ["goto", 0.5, 0.5], "then the green box");
mission = { state: "navigating" }; tick(5100);
mission = { state: "failed", detail: "no walkable path to the goal" }; tick(5200);
assert.match(b.diary[0].text, /Could not get to the green box/);
objects = [far, green, orange];   // the white box is taken away before its turn
mission = { state: "failed" }; tick(5300);
assert.deepEqual(log.at(-1), ["goto", 0.55, 0.1], "the white box is skipped, the blue object is next");
mission = { state: "navigating" }; tick(5400);
mission = { state: "done" }; tick(5500);
for (let t = 5600; t <= 8300; t += 100) tick(t);
assert.equal(b.status().tour, null);
assert.deepEqual(log.slice(-2), [["face", "happy"], ["pose", "bow"]]);
assert.equal(b.diary[0].text, "Tour finished.");
b.startTour(objects, robot, 9000); b.interrupt();
assert.equal(b.status().tour, null, "a manual command ends the tour");
console.log("PASS  tour: nearest first, skips the unreachable and the removed, ends with a bow, stops when interrupted");

assert.equal(isArmDetection({ shape: "rect", x: 0.167, y: 0.319, yaw: 1.708, width: 0.102, length: 0.444 }, { x: -0.074, y: 0.291 }), true);
assert.equal(isArmDetection({ shape: "rect", x: 0.103, y: 0.48, yaw: 1.917, width: 0.177, length: 0.167 }, { x: -0.074, y: 0.291 }), false);
assert.equal(isArmDetection(green, null), false);
console.log("PASS  the arm's own detection is told apart from a box next to it (live positions of 2026-09-19)");
