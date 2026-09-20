// Runs the Navigator against a simulated robot with delay, drift and noise. No network, simulated clock.
import assert from "node:assert/strict";
import { Navigator } from "./navigator.mjs";

const arena = { width: 0.76, length: 0.6 };
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

// Simulated robot: commands take effect after `delay` ms, walking drifts by `veer` rad per metre.
function simulate({ start, goal, obstacles = [], motion, robotModel = {}, blockedUntil = 0, maxSeconds = 120, calibrate = false }) {
  const model = { walkSpeed: 0.045, turnRate: 0.55, veer: 0.6, delay: 350, ...robotModel };
  const robot = { ...start, tracking: true };
  const queue = [], sent = [];
  let active = "", now = 0, switches = 0, pending;
  const navigator = new Navigator((command) => { sent.push(command); queue.push({ at: now + model.delay, command }); }, motion);
  if (calibrate) pending = navigator.calibrate(4);
  else navigator.start(goal);
  for (; now < maxSeconds * 1000; now += 100) {
    while (queue.length && queue[0].at <= now) {
      const command = queue.shift().command;
      const next = command === "stop" ? "" : command;
      if (next !== active) switches++;
      active = next;
    }
    const free = now >= blockedUntil;
    if ((active === "forward" || active === "backward") && free) {
      const step = (active === "forward" ? 1 : -1) * model.walkSpeed * 0.1;
      robot.x += step * Math.cos(robot.yaw); robot.y += step * Math.sin(robot.yaw);
      robot.yaw = wrap(robot.yaw + model.veer * Math.abs(step));
    } else if ((active === "left" || active === "right") && free) {
      robot.yaw = wrap(robot.yaw + (active === "left" ? 1 : -1) * model.turnRate * 0.1);
    }
    const seen = { ...robot, x: robot.x + (Math.random() - 0.5) * 0.004, y: robot.y + (Math.random() - 0.5) * 0.004, yaw: robot.yaw + (Math.random() - 0.5) * 0.03 };
    navigator.step(seen, arena, obstacles, now);
    if (!calibrate && (navigator.state === "done" || navigator.state === "failed")) break;
    if (calibrate && navigator.state !== "calibrating") break;
  }
  return { navigator, robot, seconds: now / 1000, switches, sent, pending };
}

let run = simulate({ start: { x: 0.1, y: 0.1, yaw: Math.PI }, goal: { x: 0.65, y: 0.5 } });
assert.equal(run.navigator.state, "done");
assert.ok(Math.hypot(run.robot.x - 0.65, run.robot.y - 0.5) < 0.07, "ends near the goal");
console.log(`PASS  open floor with delay, drift and noise: arrived in ${run.seconds.toFixed(0)} s, ${run.switches} gait changes`);

const wall = { shape: "rect", x: 0.38, y: 0.2, yaw: 0, width: 0.04, length: 0.4 };
run = simulate({ start: { x: 0.1, y: 0.1, yaw: 0 }, goal: { x: 0.66, y: 0.1 }, obstacles: [wall] });
assert.equal(run.navigator.state, "done");
console.log(`PASS  around a wall: arrived in ${run.seconds.toFixed(0)} s, ${run.switches} gait changes`);

run = simulate({ start: { x: 0.4, y: 0.3, yaw: 0 }, goal: { x: 0.3, y: 0.3 } });
assert.equal(run.navigator.state, "done");
assert.ok(run.sent.includes("backward") && !run.sent.includes("left") && !run.sent.includes("right"));
console.log("PASS  close target behind the robot: walks backward without turning");

run = simulate({ start: { x: 0.1, y: 0.1, yaw: 0 }, goal: { x: 0.6, y: 0.1 }, blockedUntil: 4000 });
assert.equal(run.navigator.state, "done");
assert.equal(run.navigator.recoveries, 1);
console.log("PASS  blocked for 4 s: detects stuck, backs off, turns, replans, arrives");

run = simulate({ start: { x: 0.1, y: 0.1, yaw: 0 }, goal: { x: 0.6, y: 0.1 }, blockedUntil: 1e9 });
assert.equal(run.navigator.state, "failed");
assert.match(run.navigator.detail, /stuck/);
console.log(`PASS  permanently blocked: gives up after 3 recoveries (${run.seconds.toFixed(0)} s)`);

const fullWall = { ...wall, y: 0.3, length: 0.6 };
run = simulate({ start: { x: 0.1, y: 0.1, yaw: 0 }, goal: { x: 0.66, y: 0.1 }, obstacles: [fullWall] });
assert.equal(run.navigator.state, "failed");
assert.match(run.navigator.detail, /no walkable path/);
console.log("PASS  wall across the whole arena: reports no walkable path");

run = simulate({ start: { x: 0.2, y: 0.3, yaw: 0 }, goal: { x: 0.755, y: 0.3 } });
assert.equal(run.navigator.state, "done");
assert.ok(run.robot.x < arena.width - 0.05, "stops short of the wall");
console.log("PASS  goal against the wall: goes to the nearest reachable spot and finishes");

{
  // A robot that keeps moving but never gets closer: the watchdog must end the mission.
  const navigator = new Navigator(() => {});
  navigator.start({ x: 0.6, y: 0.3 });
  let now = 0;
  for (; now < 90000 && !["failed", "done"].includes(navigator.state); now += 100) {
    navigator.step({ x: 0.2 + 0.03 * Math.sin(now / 1000), y: 0.3, yaw: 0, tracking: true }, arena, [], now);
  }
  assert.equal(navigator.state, "failed");
  assert.match(navigator.detail, /progress/);
  assert.ok(now <= 60000);
  console.log(`PASS  moving without getting closer: gives up after ${(now / 1000).toFixed(0)} s`);
}

run = simulate({ start: { x: 0.2, y: 0.3, yaw: 0 }, calibrate: true });
const m = await run.pending;
assert.ok(Math.abs(m.walkSpeed - 0.045) < 0.006 && Math.abs(m.turnRate - 0.55) < 0.06 && Math.abs(m.veer - 0.6) < 0.25, JSON.stringify(m));
assert.ok(m.turnStopLead > 0.1 && m.turnStopLead < 0.3, `turn stop lead ${m.turnStopLead}`);
console.log(`PASS  calibration: walk ${m.walkSpeed.toFixed(3)} m/s (true 0.045), turn ${m.turnRate.toFixed(2)} rad/s (true 0.55), veer ${m.veer.toFixed(2)} rad/m (true 0.6), keeps turning ${m.turnStopLead.toFixed(2)} rad after stop`);

// Does calibration pay off? Same course with default and with measured motion values.
const course = { start: { x: 0.1, y: 0.1, yaw: Math.PI }, goal: { x: 0.65, y: 0.5 } };
const average = (motion) => { let s = 0, t = 0; for (let i = 0; i < 20; i++) { const r = simulate({ ...course, motion }); s += r.switches; t += r.seconds; } return [s / 20, t / 20]; };
const [s0, t0] = average(undefined), [s1, t1] = average(run.navigator.motion);
console.log(`INFO  20 runs each: default values ${s0.toFixed(1)} gait changes, ${t0.toFixed(0)} s. calibrated ${s1.toFixed(1)} gait changes, ${t1.toFixed(0)} s`);
