// End-to-end test of bridge.mjs against a fake tracker and a fake robot. Run: npm test
import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import net from "node:net";
import { WebSocketServer } from "ws";

const camera = { f: 1693, cx: 1152, cy: 648, rvec: [Math.PI, 0, 0], tvec: [-38, 30, 130] };  // straight down from 130 cm over (38, 30)
let robotPose = { x: 38, y: 30, z: 8, heading: 90, px: [1152, 648] };
const trackerLine = () => JSON.stringify({
  t: 1, calibrated: true, frame: [2304, 1296], floor: [76, 60], zUp: true, floorMarkers: 4, fps: 15, markers: [0, 1, 2, 3, 4, 5],
  robot: robotPose, arm: { x: 70, y: 10, z: 5, heading: 180, px: [0, 0] }, camera,
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
  env: { ...process.env, PORT: "18080", TRACKER_HOST: "127.0.0.1", TRACKER_PORT: "19003", ROBOT_URL: "ws://127.0.0.1:18081" },
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
  robotPose = { ...robotPose, heading: 5 };
  await wait(300);
  assert.deepEqual(robotMessages.at(-1), { command: "forward" });
  assert.deepEqual(states.at(-1).goal, { x: 0.6, y: 0.3 });
  robotPose = { ...robotPose, x: 58, y: 30 };
  await wait(300);
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
  client.close();
} finally {
  bridge.kill();
  fakeTracker.close();
  fakeRobot.close();
}
process.exit(0);
