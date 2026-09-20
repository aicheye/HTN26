// End-to-end test of bridge.mjs against a fake tracker and a fake robot. Run: npm test
import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import fs from "node:fs";
import net from "node:net";
import os from "node:os";
import { WebSocketServer } from "ws";
import { floorToPixel } from "../pi/client/floor.js";

const camera = { f: 1693, cx: 1152, cy: 648, rvec: [Math.PI, 0, 0], tvec: [-38, 30, 130] };  // straight down from 130 cm over (38, 30)
let robotPose = { x: 38, y: 30, z: 10.5, heading: 90 };
const startedAt = Date.now();
const robotWithPixel = () => { const p = floorToPixel(camera, robotPose.x, robotPose.y, 10.5); return { ...robotPose, px: [p.u, p.v] }; };
const trackerLine = () => JSON.stringify({
  t: Date.now() - startedAt, calibrated: true, frame: [2304, 1296], floor: [76, 60], zUp: true, floorMarkers: 4, fps: 15, markers: [0, 1, 2, 3, 4, 5],
  robot: robotWithPixel(), arm: { x: 70, y: 10, z: 5, heading: 180, px: [0, 0] }, camera,
}) + "\n";

const fakeTracker = net.createServer((socket) => {
  const timer = setInterval(() => socket.write(trackerLine()), 50);
  socket.on("close", () => clearInterval(timer));
  socket.on("error", () => {});
}).listen(19003);
const robotMessages = [];
const fakeRobot = new WebSocketServer({ port: 18081 });
fakeRobot.on("connection", (socket) => {
  socket.send(JSON.stringify({ command: "", face: "happy", servos: {} }));
  socket.on("message", (data) => robotMessages.push(JSON.parse(data)));
});

const bridge = spawn("node", ["bridge.mjs"], {
  env: { ...process.env, BRIDGE_STATE_DIR: fs.mkdtempSync(os.tmpdir() + "/bridge-test-"), PORT: "18080", TRACKER_HOST: "127.0.0.1", TRACKER_PORT: "19003", ROBOT_URL: "ws://127.0.0.1:18081", ARM_REACH_M: "0.45" },
  stdio: "inherit",
});
const wait = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
const near = (a, b, tolerance = 1e-6) => assert.ok(Math.abs(a - b) < tolerance, `${a} is not near ${b}`);

try {
  await wait(1500);
  const client = new WebSocket("ws://127.0.0.1:18080/ws");
  const states = [], acks = [];
  client.onmessage = (event) => {
    const envelope = JSON.parse(event.data);
    (envelope.type === "state" ? states : acks).push(envelope.data);
  };
  await wait(500);

  const state = states.at(-1);
  assert.equal(state.schemaVersion, 1);
  near(state.arena.width, 0.76); near(state.arena.length, 0.6);
  assert.equal(state.calibration.ok, true);
  const robot = state.robots[0];
  near(robot.x, 0.38); near(robot.y, 0.3); near(robot.yaw, Math.PI / 2);
  assert.equal(robot.tracking, true); assert.equal(robot.mode, "idle"); assert.equal(robot.face, "happy");
  assert.deepEqual([state.obstacles[0].id, state.obstacles[0].tagId], ["so101-base", 5]);
  console.log("PASS  state: metres, radians, robot, arm base");

  client.send(JSON.stringify({ type: "command", data: { id: "c1", ts: 0, robotId: "sesame-1", type: "forward", face: "walk" } }));
  client.send(JSON.stringify({ type: "command", data: { id: "c2", ts: 0, robotId: "sesame-1", type: "pose", pose: "wave" } }));
  client.send(JSON.stringify({ type: "command", data: { id: "c3", ts: 0, robotId: "sesame-1", type: "stop" } }));
  await wait(300);
  assert.deepEqual(acks.map((a) => [a.commandId, a.ok]), [["c1", true], ["c2", true], ["c3", true]]);
  assert.deepEqual(robotMessages, [{ command: "forward", face: "walk" }, { command: "wave" }, { command: "stop" }]);
  console.log("PASS  commands reach the robot and are acked");

  const page = await fetch("http://127.0.0.1:18080/");
  assert.match(page.headers.get("content-type"), /text\/html/);
  assert.match(await page.text(), /Sesame controller/);
  const dropped = new WebSocket("ws://127.0.0.1:18080/ws");
  await wait(300);
  dropped.send(JSON.stringify({ type: "command", data: { id: "d1", ts: 0, robotId: "sesame-1", type: "left" } }));
  await wait(200);
  dropped.close();
  await wait(300);
  assert.deepEqual(robotMessages.slice(-2), [{ command: "left" }, { command: "stop" }]);
  console.log("PASS  controller page is served, and a client that disconnects while walking stops the robot");

  // A 10 px box centred on the pixel where floor point (48, 30) cm appears.
  const u = 1152 + (1693 * 10) / 130;
  const posted = await (await fetch("http://127.0.0.1:18080/objects", {
    method: "POST", body: JSON.stringify({ objects: [{ label: "chocolate", box: [u - 5, 643, u + 5, 653], confidence: 0.9 }] }),
  })).json();
  near(posted[0].x, 0.48, 1e-4); near(posted[0].y, 0.3, 1e-4);
  await wait(200);
  assert.equal(states.at(-1).obstacles[1].id, "chocolate-1");
  console.log("PASS  detector box becomes a cv obstacle at the right floor position");

  // Goal straight along +x while the robot faces +y: expect a right turn, then forward once it faces the goal.
  await fetch("http://127.0.0.1:18080/objects", { method: "POST", body: JSON.stringify({ objects: [] }) });  // the chocolate is in the way
  robotMessages.length = 0;
  client.send(JSON.stringify({ type: "command", data: { id: "c4", ts: 0, robotId: "sesame-1", type: "goto", target: { x: 0.6, y: 0.3 } } }));
  await wait(300);
  assert.deepEqual(robotMessages.at(-1), { command: "right" });
  assert.equal(states.at(-1).mission.state, "navigating");
  robotPose = { ...robotPose, heading: 5 };   // a jump: the filter accepts it after 12 consistent detections
  await wait(1500);
  assert.deepEqual(robotMessages.at(-1), { command: "forward" });
  assert.deepEqual(states.at(-1).goal, { x: 0.6, y: 0.3 });
  robotPose = { ...robotPose, x: 58, y: 30 };
  await wait(1500);
  assert.deepEqual(robotMessages.at(-1), { command: "stop" });
  assert.equal(states.at(-1).goal, undefined);
  assert.equal(states.at(-1).mission.state, "done");
  console.log("PASS  goto: turns, walks, stops at the goal");

  // A manual obstacle across the whole arena makes the goal unreachable.
  await fetch("http://127.0.0.1:18080/obstacles", { method: "POST", body: JSON.stringify([{ shape: "rect", x: 0.3, y: 0.3, width: 0.04, length: 0.6 }]) });
  client.send(JSON.stringify({ type: "command", data: { id: "c5", ts: 0, robotId: "sesame-1", type: "goto", target: { x: 0.1, y: 0.3 } } }));
  await wait(3500);
  assert.equal(states.at(-1).mission.state, "failed");
  assert.match(states.at(-1).mission.detail, /no walkable path/);
  console.log("PASS  goto behind a full wall: reports no walkable path");

  // An obstacle's picture is served from a URL that changes when the picture changes, so the UI knows to reload it.
  const textured = async (bytes) => {
    const body = JSON.stringify([{ id: "green-box-1", source: "cv", shape: "rect", x: 0.5, y: 0.5, width: 0.1, length: 0.1, texture: Buffer.from(bytes).toString("base64") }]);
    return (await (await fetch("http://127.0.0.1:18080/obstacles", { method: "POST", body })).json())[0];
  };
  const first = await textured("picture one"), again = await textured("picture one"), second = await textured("picture two");
  assert.equal(first.texture, undefined);
  assert.equal(first.textureUrl, again.textureUrl);
  assert.notEqual(first.textureUrl, second.textureUrl);
  assert.equal(await (await fetch(second.textureUrl)).text(), "picture two");
  console.log("PASS  obstacle picture: served by URL, and the URL changes only with the picture");
  // A wall across the arena at x = 0.45 with the goal on the arm's side of it (the fake tracker reports the arm's
  // base at 0.70, 0.10, and this bridge takes its reach as 0.45 m). No path exists. The robot stands 0.59 m from
  // the base, out of reach, so it first walks to a spot on its own side that the arm does reach. Only there is
  // the arm asked, with drop points in the tracker's frame.
  const gotoAcrossWall = async (id) => {
    await fetch("http://127.0.0.1:18080/obstacles", { method: "POST", body: JSON.stringify([{ shape: "rect", x: 0.45, y: 0.3, width: 0.04, length: 0.6 }]) });
    robotPose = { ...robotPose, x: 15, y: 30, heading: 0 };
    await wait(1200);
    client.send(JSON.stringify({ type: "command", data: { id, ts: 0, robotId: "sesame-1", type: "goto", target: { x: 0.62, y: 0.5 } } }));
    await wait(3500);
    const walking = states.at(-1).mission;
    assert.equal(walking.state, "navigating");
    assert.match(walking.detail, /walking to where the arm can pick the robot up/);
    assert.ok(walking.via.x < 0.45 - 0.02 - 0.1 + 0.011, `the pick-up spot ${JSON.stringify(walking.via)} is on the robot's side of the wall`);
    assert.ok(Math.hypot(walking.via.x - 0.7, walking.via.y - 0.1) <= 0.45 - 0.03 + 0.011, "and within the arm's reach");
    assert.deepEqual(await (await fetch("http://127.0.0.1:18080/carry")).json(), {}, "the arm is not asked before the robot is there");
    robotPose = { ...robotPose, x: walking.via.x * 100, y: walking.via.y * 100, heading: 0 };   // the robot has walked there
    // The fake robot never walks, so by now the navigator may be in a stuck recovery, which takes about 3 s.
    for (let i = 0; i < 16 && states.at(-1).mission.state !== "carrying"; i++) await wait(500);
    assert.equal(states.at(-1).mission.state, "carrying");
    const request = await (await fetch("http://127.0.0.1:18080/carry")).json();
    assert.ok(request.id > 0 && request.drops.length >= 1);
    for (const [x, y] of request.drops) assert.ok(x > 45 + 2 + 10 && Math.hypot(x - 70, y - 10) <= 45.5, `drop (${x}, ${y}) cm is past the wall and within the arm's reach`);
    return request;
  };
  const answerCarry = (body) => fetch("http://127.0.0.1:18080/carry", { method: "POST", body: JSON.stringify(body) });

  robotMessages.length = 0;
  let request = await gotoAcrossWall("c6");
  assert.equal((await answerCarry({ id: request.id + 7, ok: true })).status, 409);
  await answerCarry({ id: request.id, ok: false, reason: "the Sesame is 41 cm from the arm's base, out of reach" });
  await wait(300);
  assert.equal(states.at(-1).mission.state, "failed");
  assert.match(states.at(-1).mission.detail, /could not carry the robot: the Sesame is 41 cm/);
  assert.deepEqual(await (await fetch("http://127.0.0.1:18080/carry")).json(), {});
  console.log("PASS  fully blocked, arm refuses: walks to the arm's reach, then goto fails with the arm's reason");

  robotMessages.length = 0;
  request = await gotoAcrossWall("c7");
  robotPose = { ...robotPose, x: request.drops[0][0], y: request.drops[0][1], heading: 90 };   // the arm has set it down
  await answerCarry({ id: request.id, ok: true });
  await wait(1500);
  assert.equal(states.at(-1).mission.state, "navigating");
  assert.ok(robotMessages.some((m) => ["forward", "backward", "left", "right"].includes(m.command)), "walks on from where the arm set it down");
  robotPose = { ...robotPose, x: 62, y: 50 };
  await wait(1500);
  assert.equal(states.at(-1).mission.state, "done");
  console.log("PASS  fully blocked, arm carries: walks to the arm's reach, is carried, continues from the drop point and arrives");

  // The limit at the arena's edge is 0.07 m for the robot's centre. The camera sees the robot walk to 0.06 m from
  // the east edge, facing it, so forward is refused and the robot is told to stop. Facing the other way, forward
  // is accepted.
  await fetch("http://127.0.0.1:18080/obstacles", { method: "POST", body: "[]" });
  while (Math.hypot(robotPose.x - 70, robotPose.y - 30) > 0.5) {
    const dx = 70 - robotPose.x, dy = 30 - robotPose.y, d = Math.hypot(dx, dy), step = Math.min(1, d);
    robotPose = { ...robotPose, x: robotPose.x + (dx / d) * step, y: robotPose.y + (dy / d) * step, heading: 0 };
    await wait(60);
  }
  await wait(1500);
  const edgeAcks = [];
  const previousOnMessage = client.onmessage;
  client.onmessage = (event) => { const envelope = JSON.parse(event.data); if (envelope.type === "ack") edgeAcks.push(envelope.data); previousOnMessage?.(event); };
  robotMessages.length = 0;
  client.send(JSON.stringify({ type: "command", data: { id: "e1", ts: 0, robotId: "sesame-1", type: "forward" } }));
  await wait(400);
  assert.equal(edgeAcks.at(-1).ok, false);
  assert.match(edgeAcks.at(-1).error, /edge/);
  assert.ok(!robotMessages.some((m) => m.command === "forward"), "forward must not reach the robot");
  assert.equal(robotMessages.at(-1).command, "stop");
  robotPose = { ...robotPose, heading: 180 };
  await wait(1500);
  client.send(JSON.stringify({ type: "command", data: { id: "e2", ts: 0, robotId: "sesame-1", type: "forward" } }));
  await wait(400);
  assert.equal(edgeAcks.at(-1).ok, true);
  client.send(JSON.stringify({ type: "command", data: { id: "e3", ts: 0, robotId: "sesame-1", type: "stop" } }));
  await wait(200);
  console.log("PASS  limit at the edge: forward toward the edge is refused and stopped, forward away from it is accepted");
  client.close();

  // A second bridge whose tracker never answers, as when the camera is off or sees no marker. The robot is
  // connected, so it must still be listed (untracked), or the frontend has nothing to send its commands to.
  const blind = spawn("node", ["bridge.mjs"], {
    env: { ...process.env, BRIDGE_STATE_DIR: fs.mkdtempSync(os.tmpdir() + "/bridge-test-"), PORT: "18090", TRACKER_HOST: "127.0.0.1", TRACKER_PORT: "19099", ROBOT_URL: "ws://127.0.0.1:18081" },
    stdio: "ignore",
  });
  try {
    await wait(1500);
    const unseen = (await (await fetch("http://127.0.0.1:18090/state")).json()).robots;
    assert.equal(unseen.length, 1);
    assert.equal(unseen[0].id, "sesame-1");
    assert.equal(unseen[0].tracking, false);
    assert.equal(unseen[0].mode, "lost");
    const pad = new WebSocket("ws://127.0.0.1:18090/ws");
    const acks = [];
    pad.onmessage = (event) => { const envelope = JSON.parse(event.data); if (envelope.type === "ack") acks.push(envelope.data); };
    await new Promise((resolve) => { pad.onopen = resolve; });
    robotMessages.length = 0;
    pad.send(JSON.stringify({ type: "command", data: { id: "b1", ts: 0, robotId: "sesame-1", type: "forward" } }));
    await wait(400);
    assert.deepEqual(acks, [{ commandId: "b1", ok: true }]);
    assert.equal(robotMessages.at(-1).command, "forward");
    pad.send(JSON.stringify({ type: "command", data: { id: "b2", ts: 0, robotId: "sesame-1", type: "stop" } }));
    await wait(300);
    pad.close();
    console.log("PASS  robot connected but never seen by the camera: listed as untracked, and manual commands reach it");
  } finally {
    blind.kill();
  }
} finally {
  bridge.kill();
  fakeTracker.close();
  fakeRobot.close();
}
process.exit(0);
