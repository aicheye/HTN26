// Backend for the frontend's WebSocket contract (frontend branch: src/types/world.ts, docs/HANDOFF.md).
// Runs on the laptop while it is on the Sesame-Controller WiFi:
//   cd bridge && npm install && npm start
// It reads poses from the Pi tracker (TCP, centimetres and degrees), talks to the robot firmware
// (WebSocket), and serves ws://localhost:8080/ws with WorldState in metres and radians.
// The object detector posts its boxes to POST http://localhost:8080/objects.
import http from "node:http";
import net from "node:net";
import { WebSocketServer } from "ws";
import { pixelToFloor } from "../pi/client/floor.js";

const PORT = Number(process.env.PORT ?? 8080);
const TRACKER_HOST = process.env.TRACKER_HOST ?? "qnxpi78.local";
const TRACKER_PORT = Number(process.env.TRACKER_PORT ?? 9003);
const ROBOT_URL = process.env.ROBOT_URL ?? "ws://192.168.4.1:81";

const ROBOT_TAG = 0, ARM_TAG = 5, CORNER_TAGS = [1, 2, 3, 4];
const ROBOT_FOOTPRINT = { width: 0.105, length: 0.125 };  // metres, from the frontend's sample state
const ARM_BASE_RADIUS = 0.09;                             // metres, estimate of the SO-101 base
const TRACKING_TIMEOUT_MS = 500;
const GOAL_REACHED_M = 0.06;
const TURN_THRESHOLD_RAD = 0.45;  // turn in place while the heading error is larger than this
const MOVES = ["forward", "backward", "left", "right"];

let tracker = null;        // latest tracker message
let trackerAt = 0;         // when it arrived, ms since epoch
let robotState = null;     // latest firmware state: {command, face, servos}
let robotSocket = null;
let lastRobot = null;      // last known robot pose in the frontend's frame, with lastSeen
let lastArm = null;
let cvObstacles = [];      // from the detector, already in metres
let goal = null;
let drive = "";            // last movement command sent by the goto controller
let seq = 0;

// Tracker frame: centimetres, origin at floor marker 1, x toward marker 2, y toward marker 4.
// Frontend frame: metres, +y up the screen, yaw counter-clockwise seen from above. When the markers
// run clockwise seen from above (zUp false), y and the rotation direction are mirrored.
function toWorld(x, y, headingDeg = 0) {
  const mirrored = tracker && tracker.zUp === false;
  const lengthCm = tracker ? tracker.floor[1] : 0;
  const yaw = ((mirrored ? -headingDeg : headingDeg) * Math.PI) / 180;
  return { x: x / 100, y: (mirrored ? lengthCm - y : y) / 100, yaw };
}

function connectTracker() {
  const socket = net.connect(TRACKER_PORT, TRACKER_HOST);
  let buffer = "";
  socket.on("data", (chunk) => {
    buffer += chunk;
    const lines = buffer.split("\n");
    buffer = lines.pop();
    for (const line of lines) {
      try {
        tracker = JSON.parse(line);
        trackerAt = Date.now();
      } catch {}
    }
  });
  socket.on("error", () => {});
  socket.on("close", () => setTimeout(connectTracker, 1000));
}

function connectRobot() {
  const socket = new WebSocket(ROBOT_URL);
  socket.onopen = () => { robotSocket = socket; };
  socket.onmessage = (event) => { try { robotState = JSON.parse(event.data); } catch {} };
  socket.onerror = () => {};
  socket.onclose = () => {
    robotSocket = null;
    setTimeout(connectRobot, 1000);
  };
}

function sendToRobot(message) {
  if (!robotSocket || robotSocket.readyState !== WebSocket.OPEN) return false;
  robotSocket.send(JSON.stringify(message));
  return true;
}

function buildState() {
  const now = Date.now();
  const fresh = tracker && now - trackerAt < TRACKING_TIMEOUT_MS && tracker.calibrated;
  if (fresh && tracker.robot) lastRobot = { ...toWorld(tracker.robot.x, tracker.robot.y, tracker.robot.heading), lastSeen: trackerAt };
  if (fresh && tracker.arm) lastArm = toWorld(tracker.arm.x, tracker.arm.y, tracker.arm.heading);
  const tracking = Boolean(fresh && tracker.robot);

  const command = robotState?.command ?? "";
  const mode = !tracking ? "lost" : command === "left" || command === "right" ? "turning" : MOVES.includes(command) ? "moving" : "idle";
  const robot = lastRobot && {
    id: "sesame-1", tagId: ROBOT_TAG, x: lastRobot.x, y: lastRobot.y, yaw: lastRobot.yaw, footprint: ROBOT_FOOTPRINT,
    tracking, lastSeen: lastRobot.lastSeen, mode,
    ...(robotState?.face ? { face: robotState.face } : {}),
    ...(command && !MOVES.includes(command) ? { pose: command } : {}),
  };
  const arm = lastArm && {
    id: "so101-base", source: "tag", shape: "circle", x: lastArm.x, y: lastArm.y, yaw: lastArm.yaw,
    radius: ARM_BASE_RADIUS, tagId: ARM_TAG,
  };
  return {
    schemaVersion: 1, seq: seq++, timestamp: now,
    arena: { width: (tracker?.floor[0] ?? 0) / 100, length: (tracker?.floor[1] ?? 0) / 100, cornerTagIds: CORNER_TAGS },
    calibration: { ok: Boolean(fresh) },
    robots: robot ? [robot] : [],
    obstacles: [...(arm ? [arm] : []), ...cvObstacles],
    ...(goal ? { goal, path: robot ? [{ x: robot.x, y: robot.y }, goal] : [goal] } : {}),
  };
}

// Drives toward the goal with the firmware's continuous gaits: turn in place until roughly facing
// the goal, then walk forward. Stops when the goal is reached or the robot has not been seen for a while.
function stepGoto(state) {
  if (!goal) return;
  const robot = state.robots[0];
  let want = "stop";
  if (robot?.tracking) {
    const dx = goal.x - robot.x, dy = goal.y - robot.y;
    if (Math.hypot(dx, dy) < GOAL_REACHED_M) {
      goal = null;
    } else {
      const error = Math.atan2(Math.sin(Math.atan2(dy, dx) - robot.yaw), Math.cos(Math.atan2(dy, dx) - robot.yaw));
      want = Math.abs(error) > TURN_THRESHOLD_RAD ? (error > 0 ? "left" : "right") : "forward";
    }
  }
  if (want !== drive) {
    sendToRobot({ command: want });
    drive = want;
  }
  if (!goal) drive = "";
}

function handleCommand(command) {
  const ack = (ok, error) => ({ commandId: command.id, ok, ...(error ? { error } : {}) });
  const face = command.face ? { face: command.face } : {};
  let sent;
  if (command.type === "goto") {
    if (!command.target) return ack(false, "goto needs a target");
    goal = command.target;
    drive = "";
    return ack(true);
  }
  goal = null;  // any manual command cancels a goto
  drive = "";
  if (MOVES.includes(command.type)) sent = sendToRobot({ command: command.type, ...face });
  else if (command.type === "stop") sent = sendToRobot({ command: "stop" });
  else if (command.type === "pose") sent = command.pose ? sendToRobot({ command: command.pose, ...face }) : null;
  else if (command.type === "face") sent = command.face ? sendToRobot(face) : null;
  else return ack(false, `unknown command type ${command.type}`);
  if (sent === null) return ack(false, `${command.type} needs a ${command.type} field`);
  return sent ? ack(true) : ack(false, "robot not connected");
}

// Detector input: {"objects": [{"label", "box": [left, top, right, bottom], "confidence"?}], "camera"?}
// Boxes are full-frame pixels. "camera" is the tracker's camera object from the moment the frame was taken.
// Without it the latest one is used, which is only right if the camera has not moved since.
function setObjects(body) {
  const camera = body.camera ?? tracker?.camera;
  if (!camera) throw new Error("no camera pose yet: the tracker has not seen a floor marker");
  cvObstacles = body.objects.map((object, index) => {
    const [left, top, right, bottom] = object.box;
    const corners = [[left, top], [right, top], [right, bottom], [left, bottom]].map(([u, v]) => {
      const p = pixelToFloor(camera, u, v);
      return toWorld(p.x, p.y);
    });
    const xs = corners.map((p) => p.x), ys = corners.map((p) => p.y);
    const width = Math.max(...xs) - Math.min(...xs), length = Math.max(...ys) - Math.min(...ys);
    return {
      id: `${object.label}-${index + 1}`, source: "cv", shape: "rect",
      x: Math.min(...xs) + width / 2, y: Math.min(...ys) + length / 2, yaw: 0, width, length,
      ...(object.confidence !== undefined ? { confidence: object.confidence } : {}),
    };
  });
  return cvObstacles;
}

const server = http.createServer((request, response) => {
  const reply = (status, body) => {
    response.writeHead(status, { "Content-Type": "application/json", "Access-Control-Allow-Origin": "*", "Access-Control-Allow-Headers": "*" });
    response.end(JSON.stringify(body));
  };
  if (request.method === "OPTIONS") return reply(204, {});
  if (request.url === "/state") return reply(200, buildState());
  if (request.url === "/objects" && request.method === "GET") return reply(200, cvObstacles);
  if (request.url === "/objects" && request.method === "POST") {
    let text = "";
    request.on("data", (chunk) => { text += chunk; });
    request.on("end", () => {
      try { reply(200, setObjects(JSON.parse(text))); } catch (error) { reply(400, { error: error.message }); }
    });
    return;
  }
  reply(404, { error: "not found" });
});

const sockets = new WebSocketServer({ server, path: "/ws" });
sockets.on("connection", (socket) => {
  socket.on("message", (data) => {
    let envelope;
    try { envelope = JSON.parse(data); } catch { return; }
    if (envelope.type === "command") socket.send(JSON.stringify({ type: "ack", data: handleCommand(envelope.data) }));
  });
});

setInterval(() => {
  const state = buildState();
  stepGoto(state);
  const message = JSON.stringify({ type: "state", data: state });
  for (const socket of sockets.clients) if (socket.readyState === WebSocket.OPEN) socket.send(message);
}, 100);

connectTracker();
connectRobot();
server.listen(PORT, () => console.log(`bridge: ws://localhost:${PORT}/ws  tracker ${TRACKER_HOST}:${TRACKER_PORT}  robot ${ROBOT_URL}`));
