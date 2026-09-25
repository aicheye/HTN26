// Checks the pose filter on synthetic data with known truth, then replays the real recordings when they are present.
import assert from "node:assert/strict";
import fs from "node:fs";
import { floorToPixel, pixelToFloor } from "../pi/client/floor.js";
import { PoseFilter } from "./pose-filter.mjs";

const noise = (sigma) => sigma * Math.sqrt(-2 * Math.log(1 - Math.random())) * Math.cos(2 * Math.PI * Math.random());
// Camera looking down from `height` over floor point (cx, cy), in a right-handed floor frame (see bridge/test.mjs).
const cameraOver = (cx, cy, height) => ({ f: 1693, cx: 1152, cy: 648, rvec: [Math.PI, 0, 0], tvec: [-cx, cy, height] });
const spread = (values) => { const m = values.reduce((a, b) => a + b, 0) / values.length; return Math.sqrt(values.reduce((a, b) => a + (b - m) ** 2, 0) / values.length); };

// 1. Robot standing still at (30, 25), marker 7 cm up, handheld camera wandering. The tracker's own x, y use a height
//    that reads 5 cm low, as in the recordings.
{
  const truth = { x: 30, y: 25, z: 7, heading: 0.5 }, filter = new PoseFilter({}, { markerHeight: null });
  const raw = [], filtered = [];
  for (let i = 0; i < 300; i++) {
    const t = i / 15, camera = cameraOver(31 + 22 * Math.sin(t / 3), 31 + 18 * Math.cos(t / 4), 105 + 8 * Math.sin(t / 5));
    const p = floorToPixel(camera, truth.x, truth.y, truth.z), px = [p.u + noise(1), p.v + noise(1)];
    const biased = pixelToFloor(camera, px[0], px[1], truth.z - 5 + noise(2));
    filter.predict(1 / 15);
    filter.update({ px, heading: truth.heading + noise(0.015), x: biased.x, y: biased.y, camera, floorMarkers: 3 });
    if (i >= 150) { raw.push(Math.hypot(biased.x - truth.x, biased.y - truth.y)); filtered.push(Math.hypot(filter.pose.x - truth.x, filter.pose.y - truth.y)); }
  }
  const mean = (a) => a.reduce((s, v) => s + v, 0) / a.length;
  assert.ok(Math.abs(filter.pose.z - truth.z) < 1, `height ${filter.pose.z}`);
  assert.ok(mean(filtered) < mean(raw) / 3);
  console.log(`PASS  (synthetic only, real data does not support it) still robot, moving camera: height learned ${filter.pose.z.toFixed(1)} cm (true 7, size-based reads 2). position error ${mean(raw).toFixed(2)} cm raw, ${mean(filtered).toFixed(2)} cm filtered`);
}

// 2. Robot walking with the marker hidden for 3 s (for example under the arm), plus a few wild detections.
{
  const motion = { walkSpeed: 0.045, turnRate: 0.5, veer: 0 }, filter = new PoseFilter({ ...motion, walkSpeed: 0.04 }, { markerHeight: 7 });  // filter's speed is 11% off
  const truth = { x: 10, y: 20, heading: 0.3, z: 7 }, camera = cameraOver(31, 31, 110);
  let worstHidden = 0, used = 0, ignored = 0, errors = [];
  for (let i = 0; i < 225; i++) {
    const dt = 1 / 15, hidden = i >= 90 && i < 135, wild = i % 40 === 17;
    truth.x += motion.walkSpeed * 100 * Math.cos(truth.heading) * dt; truth.y += motion.walkSpeed * 100 * Math.sin(truth.heading) * dt;
    filter.predict(dt, "forward");
    if (!hidden) {
      const p = floorToPixel(camera, truth.x, truth.y, truth.z), px = wild ? [p.u + 90, p.v - 70] : [p.u + noise(1), p.v + noise(1)];
      const accepted = filter.update({ px, heading: truth.heading + noise(0.015), x: truth.x, y: truth.y, camera, floorMarkers: 4 });
      if (wild) { accepted ? used++ : ignored++; }
    }
    const error = Math.hypot(filter.pose.x - truth.x, filter.pose.y - truth.y);
    if (hidden) worstHidden = Math.max(worstHidden, error); else if (i > 30) errors.push(error);
  }
  assert.equal(used, 0);
  assert.ok(worstHidden < 3, `error while hidden ${worstHidden}`);
  console.log(`PASS  walking robot: ${ignored} wild detections ignored, error ${Math.max(...errors).toFixed(2)} cm at worst while visible, ${worstHidden.toFixed(2)} cm at worst after 3 s hidden (it walked 13.5 cm in that time)`);
}

// 3. The real recordings, when present: robot standing still, handheld camera.
for (const name of ["rec-001", "rec-002"]) {
  const file = new URL(`../recordings/${name}/states.jsonl`, import.meta.url);
  if (!fs.existsSync(file)) { console.log(`SKIP  ${name}: not on this machine`); continue; }
  const filter = new PoseFilter();  // default marker height, 10.5 cm as measured
  let last = null; const raw = [], out = [];
  for (const line of fs.readFileSync(file, "utf8").trim().split("\n")) {
    const state = JSON.parse(line).state, robot = state.robot;
    if (last !== null) filter.predict((state.t - last) / 1000, "", state.zUp === false);
    last = state.t;
    if (!robot || robot.x === undefined || !state.camera) continue;
    filter.update({ px: robot.px, heading: (robot.heading * Math.PI) / 180, x: robot.x, y: robot.y, camera: state.camera, floorMarkers: state.floorMarkers });
    raw.push([robot.x, robot.y, robot.z]); out.push([filter.pose.x, filter.pose.y, filter.pose.z]);
  }
  const half = Math.floor(out.length / 2), col = (rows, i) => rows.slice(half).map((r) => r[i]);
  const jumps = (rows) => rows.slice(1).map((r, i) => Math.hypot(r[0] - rows[i][0], r[1] - rows[i][1])).sort((a, b) => a - b);
  console.log(`INFO  ${name}, second half of ${out.length} frames: position spread raw ${Math.hypot(spread(col(raw, 0)), spread(col(raw, 1))).toFixed(2)} cm -> filtered ${Math.hypot(spread(col(out, 0)), spread(col(out, 1))).toFixed(2)} cm. largest frame-to-frame jump raw ${jumps(raw).at(-1).toFixed(2)} -> ${jumps(out).at(-1).toFixed(2)} cm`);
}
