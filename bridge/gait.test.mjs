import assert from "node:assert/strict";
import { extractGait, FIRMWARE_GAITS } from "./extract-gaits.mjs";
import { FIRMWARE, GaitEngine, STAND, TROT } from "./gait.mjs";

for (const [name, fn] of Object.entries(FIRMWARE_GAITS)) assert.deepEqual(FIRMWARE[name], extractGait(fn), `${name} differs from the firmware source`);
console.log("PASS  firmware gait tables match movement-sequences.h");

// Share of the cycle each leg spends in the air, and frames where both legs of one side are up.
const LIFT = { R3: [135, "R1"], R4: [45, "R2"], L3: [45, "L1"], L4: [135, "L2"] };
function airtime(gait) {
  const knees = { R3: 180, R4: 0, L3: 0, L4: 180 }, air = { R1: 0, R2: 0, L1: 0, L2: 0 };
  let oneSideUp = 0;
  for (let pass = 0; pass < 2; pass++) {  // first pass settles the knees into the steady cycle
    for (const frame of gait.cycle) {
      Object.assign(knees, Object.fromEntries(Object.entries(frame).filter(([s]) => s in knees)));
      if (pass === 0) continue;
      const up = Object.entries(LIFT).filter(([knee, [angle]]) => knees[knee] === angle).map(([, [, hip]]) => hip);
      for (const hip of up) air[hip]++;
      if ((up.includes("R1") && up.includes("R2")) || (up.includes("L1") && up.includes("L2"))) oneSideUp++;
    }
  }
  return { air, oneSideUp };
}
const firmware = airtime(FIRMWARE.forward), trot = airtime(TROT.forward);
assert.deepEqual(firmware, { air: { R1: 4, R2: 4, L1: 2, L2: 2 }, oneSideUp: 2 });
assert.deepEqual(trot, { air: { R1: 2, R2: 2, L1: 2, L2: 2 }, oneSideUp: 0 });
console.log(`PASS  frames in the air per 6-frame cycle. firmware walk: ${JSON.stringify(firmware.air)}, both right legs up in ${firmware.oneSideUp} frames. trot: ${JSON.stringify(trot.air)}, never`);

// Fake robot: applies a pose 30 ms after it arrives and reports it back, like the firmware's state message.
function fakeRobot(options) {
  const sent = [], servos = { ...STAND };
  const engine = new GaitEngine((pose) => {
    sent.push({ at: Date.now(), pose });
    setTimeout(() => { Object.assign(servos, pose); engine.onRobotState({ servos: { ...servos } }); }, 30);
  }, { frameDelay: 20, ...options });
  return { engine, sent };
}
const wait = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

let { engine, sent } = fakeRobot({ gait: "firmware" });
engine.set("forward");
await wait(700);
engine.set("stop");
await wait(200);
const expected = [...FIRMWARE.forward.start, ...FIRMWARE.forward.cycle];
assert.deepEqual(sent.slice(0, expected.length).map((s) => s.pose), expected);
assert.deepEqual(sent.at(-1).pose, STAND);
assert.ok(sent.every((s, i) => i === 0 || s.at - sent[i - 1].at >= 45), "each frame waits for the robot to confirm the previous one");
console.log("PASS  engine replays the firmware walk exactly, waits for each pose to be confirmed, stands on stop");

({ engine, sent } = fakeRobot({ gait: "trot", trim: 0.2 }));
engine.set("forward");
await wait(500);
engine.set("stop");
await wait(200);
const swing = (hip) => { const a = sent.map((s) => s.pose[hip]).filter((v) => v !== undefined && v !== STAND[hip]); return Math.max(...a, STAND[hip]) - Math.min(...a, STAND[hip]); };
assert.ok(swing("R1") < swing("L1") && Math.abs(swing("R1") / swing("L1") - 0.8) < 0.05, `right swing ${swing("R1")}, left swing ${swing("L1")}`);
console.log(`PASS  trim 0.2 shortens the right stride: R1 swings ${swing("R1")} degrees, L1 swings ${swing("L1")}`);
