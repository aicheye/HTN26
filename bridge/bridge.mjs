// Backend for the frontend's WebSocket contract (frontend branch: src/types/world.ts, docs/HANDOFF.md).
// Runs on the laptop while it is on the Sesame-Controller WiFi:
//   cd bridge && npm install && npm start
// It reads poses from the Pi tracker (TCP, centimetres and degrees), talks to the robot firmware
// (WebSocket), and serves ws://localhost:8080/ws with WorldState in metres and radians.
// The object detector posts its boxes to POST http://localhost:8080/objects.
// goto is handled by navigator.mjs: path planning around obstacles, walking control, stuck recovery.
// POST /drive {"mode": "software", "gait": "trot", "trim": 0.1} walks the robot with gait.mjs (poses streamed from
// here, no reflash needed) in place of the firmware's own gaits. The setting is saved to drive.json.
// GET / serves controller.html, a manual controller that sends the same /ws commands, so it walks with the drive
// settings above where the firmware's captive portal always uses the firmware gaits.
// POST /calibrate measures the robot's real walking and turning with the camera and saves motion.json.
import crypto from "node:crypto";
import fs from "node:fs";
import http from "node:http";
import net from "node:net";
import { pathToFileURL } from "node:url";
import { WebSocketServer } from "ws";
import { pixelToFloor } from "../pi/client/floor.js";
import { GaitEngine } from "./gait.mjs";
import { Navigator } from "./navigator.mjs";
import { ROBOT_RADIUS_M, edgeMargin, leavesArena } from "./planner.mjs";
import { PoseFilter } from "./pose-filter.mjs";
import { createVoiceHandler } from "./voice.mjs";

const PORT = Number(process.env.PORT ?? 8080);
// The Pi's name does not resolve on every network. pi/host can hold its address (see pi/common.sh).
const HOST_FILE = new URL("../pi/host", import.meta.url);
const TRACKER_HOST = process.env.TRACKER_HOST ?? (fs.existsSync(HOST_FILE) ? fs.readFileSync(HOST_FILE, "utf8").trim() : "qnxpi78.local");
const TRACKER_PORT = Number(process.env.TRACKER_PORT ?? 9003);
// pi/robot-host holds the robot's address on a shared network (written by pi/robot-join-wifi.sh). Without it the
// robot is reached on its own access point.
const ROBOT_HOST_FILE = new URL("../pi/robot-host", import.meta.url);
const ROBOT_URL = process.env.ROBOT_URL ?? `ws://${fs.existsSync(ROBOT_HOST_FILE) ? fs.readFileSync(ROBOT_HOST_FILE, "utf8").trim() : "192.168.4.1"}:81`;

const ROBOT_TAG = 0, ARM_TAG = 5, CORNER_TAGS = [1, 2, 3, 4];
const ROBOT_FOOTPRINT = { width: 0.105, length: 0.125 };  // metres, from the frontend's sample state
const ARM_BASE_RADIUS = 0.09;                             // metres, estimate of the SO-101 base
const TRACKING_TIMEOUT_MS = 500;
// Saved settings live next to this file. The tests point BRIDGE_STATE_DIR at an empty folder.
const STATE_DIR = process.env.BRIDGE_STATE_DIR ? pathToFileURL(process.env.BRIDGE_STATE_DIR + "/") : new URL("./", import.meta.url);
const MOTION_FILE = new URL("motion.json", STATE_DIR);
const DRIVE_FILE = new URL("drive.json", STATE_DIR);
const CONTROLLER_FILE = new URL("./controller.html", import.meta.url);
const MOVES = ["forward", "backward", "left", "right"];

let tracker = null;        // latest tracker message
let trackerAt = 0;         // when it arrived, ms since epoch
let robotState = null;     // latest firmware state: {command, face, servos}
let robotSocket = null;
let lastRobot = null;      // last known robot pose in the frontend's frame, with lastSeen
let lastArm = null;
let cvObstacles = [];      // from the detector, already in metres
let manualObstacles = [];  // posted directly in metres: from the frontend, the simulator, or vision/scan.py
const textures = new Map();  // obstacle id -> PNG bytes. Sent once by vision/scan.py, served at /textures/<id>.png
let seq = 0;

const savedMotion = fs.existsSync(MOTION_FILE) ? JSON.parse(fs.readFileSync(MOTION_FILE, "utf8")) : {};

// Movement goes through the firmware's own gaits (mode "firmware") or through gait.mjs (mode "software").
const drive = { mode: "firmware", gait: "trot", trim: 0, frameDelay: 100, ...(fs.existsSync(DRIVE_FILE) ? JSON.parse(fs.readFileSync(DRIVE_FILE, "utf8")) : {}) };
const gaitEngine = new GaitEngine((servos) => sendToRobot({ servos }), drive);
// Every walking command passes through here: manual driving, voice, goto and its stuck recovery. A walk that would
// take a tracked robot into the closed strip along the table's edge (planner.mjs) is replaced by a stop.
let edgeStops = 0;
let latestState = null;  // the state of the last 100 ms tick, for the edge check
// The robot's reach from its marker in metres, legs included. vision/scan.py measures it in the camera picture and
// posts it to /robot. Obstacle clearance and the closed strip along the edge both grow with it.
let robotRadius = ROBOT_RADIUS_M;
function move(command, face = {}) {
  const robot = latestState?.robots[0];
  if (robot?.tracking && leavesArena(robot, latestState.arena, command, robotRadius)) {
    edgeStops++;
    move("stop");
    return false;
  }
  if (drive.mode !== "software") return sendToRobot({ command, ...face });
  if (Object.keys(face).length) sendToRobot(face);
  gaitEngine.set(command);
  return robotSocket?.readyState === WebSocket.OPEN;
}
const navigator = new Navigator((command) => move(command), savedMotion);

// The robot pose shown and used for navigation comes from a Kalman filter over the tracker's detections
// (pose-filter.mjs). markerHeight is the height of the robot's marker above the floor in cm: 10.5, measured on the
// standing robot. Change it with POST /drive {"markerHeight": ...}.
const poseFilter = new PoseFilter(navigator.motion, { markerHeight: drive.markerHeight ?? 10.5 });
let filteredAt = null, lastAcceptedAt = 0;
function onTrackerFrame(frame) {
  const command = drive.mode === "software" ? gaitEngine.command : MOVES.includes(robotState?.command) ? robotState.command : "";
  if (filteredAt !== null) poseFilter.predict(Math.min(1, Math.max(0, (frame.t - filteredAt) / 1000)), command, frame.zUp === false);
  filteredAt = frame.t;
  const robot = frame.robot;
  if (!frame.calibrated || !frame.camera || robot?.x === undefined) return;
  const used = poseFilter.update({ px: robot.px, heading: (robot.heading * Math.PI) / 180, x: robot.x, y: robot.y, camera: frame.camera, floorMarkers: frame.floorMarkers });
  if (used) lastAcceptedAt = Date.now();
}

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
        onTrackerFrame(tracker);
      } catch {}
    }
  });
  socket.on("error", () => {});
  socket.on("close", () => setTimeout(connectTracker, 1000));
}

function connectRobot() {
  const socket = new WebSocket(ROBOT_URL);
  socket.onopen = () => { robotSocket = socket; };
  socket.onmessage = (event) => {
    try { robotState = JSON.parse(event.data); } catch { return; }
    gaitEngine.onRobotState(robotState);
  };
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
  // The filtered pose keeps moving with the commanded gait while the marker is hidden. tracking says whether a
  // detection was accepted recently, so the frontend can grey the robot out while it is only predicted.
  const pose = poseFilter.pose;
  if (pose) lastRobot = { ...toWorld(pose.x, pose.y, (pose.heading * 180) / Math.PI), lastSeen: lastAcceptedAt || (lastRobot?.lastSeen ?? 0), sigma: pose.sigma / 100 };
  if (fresh && tracker.arm) lastArm = toWorld(tracker.arm.x, tracker.arm.y, tracker.arm.heading);
  const tracking = Boolean(fresh && now - lastAcceptedAt < TRACKING_TIMEOUT_MS);

  const command = (drive.mode === "software" && gaitEngine.command) || (robotState?.command ?? "");
  const mode = !tracking ? "lost" : command === "left" || command === "right" ? "turning" : MOVES.includes(command) ? "moving" : "idle";
  // The frontend sends commands to a robot from this list, so without an entry its controls do nothing. A robot
  // that is connected but has never been seen by the camera is therefore listed too, in the middle of the arena
  // with tracking false, which the frontend draws greyed out. Manual driving then works without the camera. goto
  // does not: the navigator refuses to steer a robot that is not tracked.
  const connected = robotSocket?.readyState === WebSocket.OPEN;
  const known = lastRobot ?? (connected ? { x: (tracker?.floor[0] ?? 0) / 200, y: (tracker?.floor[1] ?? 0) / 200, yaw: 0, lastSeen: 0, sigma: 1 } : null);
  const robot = known && {
    id: "sesame-1", tagId: ROBOT_TAG, x: known.x, y: known.y, yaw: known.yaw, footprint: ROBOT_FOOTPRINT,
    tracking, lastSeen: known.lastSeen, mode, confidence: Math.max(0, Math.min(1, 1 - known.sigma / 0.05)),
    ...(robotState?.face ? { face: robotState.face } : {}),
    ...(command && !MOVES.includes(command) ? { pose: command } : {}),
  };
  const arm = lastArm && {
    id: "so101-base", source: "tag", shape: "circle", x: lastArm.x, y: lastArm.y, yaw: lastArm.yaw,
    radius: ARM_BASE_RADIUS, tagId: ARM_TAG,
  };
  return {
    schemaVersion: 1, seq: seq++, timestamp: now,
    // edgeMargin is not part of the frontend schema: the robot's centre stays this far inside the arena (planner.mjs)
    arena: { width: (tracker?.floor[0] ?? 0) / 100, length: (tracker?.floor[1] ?? 0) / 100, cornerTagIds: CORNER_TAGS, edgeMargin: edgeMargin(robotRadius) },
    calibration: { ok: Boolean(fresh) },
    robots: robot ? [robot] : [],
    obstacles: [...(arm ? [arm] : []), ...cvObstacles, ...manualObstacles],
    ...(navigator.goal ? { goal: navigator.goal, path: robot ? [{ x: robot.x, y: robot.y }, ...navigator.path] : navigator.path } : {}),
    mission: { ...navigator.status(), edgeStops, robotRadius },  // not part of the frontend schema: navigation state for display and debugging
  };
}

function handleCommand(command) {
  const ack = (ok, error) => ({ commandId: command.id, ok, ...(error ? { error } : {}) });
  const face = command.face ? { face: command.face } : {};
  let sent;
  if (command.type === "goto") {
    if (!command.target) return ack(false, "goto needs a target");
    navigator.start(command.target);
    return ack(true);
  }
  navigator.cancel();  // any manual command cancels a goto
  if (latestState?.robots[0]?.tracking && leavesArena(latestState.robots[0], latestState.arena, command.type, robotRadius)) {
    move("stop");
    return ack(false, "the strip along the table's edge is closed to the robot");
  }
  if (MOVES.includes(command.type) || command.type === "stop") sent = move(command.type, command.type === "stop" ? {} : face);
  else if (command.type === "pose") sent = command.pose ? (gaitEngine.set("stop"), sendToRobot({ command: command.pose, ...face })) : null;
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

const voiceHandler = createVoiceHandler({ getState: buildState });
const server = http.createServer((request, response) => {
  if (request.url === "/voice") { void voiceHandler(request, response); return; }
  const reply = (status, body) => {
    response.writeHead(status, { "Content-Type": "application/json", "Access-Control-Allow-Origin": "*", "Access-Control-Allow-Headers": "*" });
    response.end(JSON.stringify(body));
  };
  if (request.method === "OPTIONS") return reply(204, {});
  if (request.url === "/" && request.method === "GET") {
    response.writeHead(200, { "Content-Type": "text/html; charset=utf-8" });
    return response.end(fs.readFileSync(CONTROLLER_FILE));  // read per request, so edits show on reload
  }
  if (request.url === "/state") return reply(200, buildState());
  if (request.url === "/obstacles" && request.method === "GET") return reply(200, manualObstacles);
  if (request.url.startsWith("/textures/")) {
    const png = textures.get(decodeURIComponent(request.url.slice("/textures/".length).replace(/\.png(\?.*)?$/, "")));
    if (!png) return reply(404, { error: "no such texture" });
    // The URL carries a hash of the picture, so a cached copy is never out of date.
    response.writeHead(200, { "Content-Type": "image/png", "Access-Control-Allow-Origin": "*", "Cache-Control": "max-age=3600" });
    return response.end(png);
  }
  if (request.url === "/objects" && request.method === "GET") return reply(200, cvObstacles);
  if (request.url === "/robot" && request.method === "GET") return reply(200, { radius: robotRadius });
  if (request.url === "/robot" && request.method === "POST") {
    let text = "";
    request.on("data", (chunk) => { text += chunk; });
    request.on("end", () => {
      try {
        const radius = Number(JSON.parse(text).radius);
        if (!Number.isFinite(radius)) throw new Error("radius must be a number, in metres");
        // Never below the body's own half-diagonal. 0.12 m is the most the legs reach, and a larger value means the
        // measurement merged the robot with something next to it.
        robotRadius = navigator.robotRadius = Math.min(0.12, Math.max(ROBOT_RADIUS_M, radius));
        reply(200, { radius: robotRadius });
      } catch (error) { reply(400, { error: error.message }); }
    });
    return;
  }
  if (request.url === "/drive" && request.method === "GET") return reply(200, drive);
  if (request.url === "/drive" && request.method === "POST") {
    let text = "";
    request.on("data", (chunk) => { text += chunk; });
    request.on("end", () => {
      try {
        gaitEngine.set("stop");
        Object.assign(drive, JSON.parse(text));
        gaitEngine.configure(drive);
        if (drive.markerHeight !== undefined) { poseFilter.options.markerHeight = drive.markerHeight; poseFilter.state = null; }
        fs.writeFileSync(DRIVE_FILE, JSON.stringify(drive, null, 2) + "\n");
        reply(200, drive);
      } catch (error) { reply(400, { error: error.message }); }
    });
    return;
  }
  if (request.url === "/calibrate" && request.method === "POST") {
    navigator.calibrate().then((motion) => {
      fs.writeFileSync(MOTION_FILE, JSON.stringify(navigator.motion, null, 2) + "\n");
      poseFilter.motion = { ...poseFilter.motion, ...navigator.motion };
      reply(200, motion);
    }, (error) => reply(409, { error: error.message }));
    return;
  }
  if ((request.url === "/objects" || request.url === "/obstacles") && request.method === "POST") {
    let text = "";
    request.on("data", (chunk) => { text += chunk; });
    request.on("end", () => {
      try {
        // /obstacles takes a list of obstacles in the frontend's own format (metres) and replaces the manual ones.
        if (request.url === "/obstacles") {
          // A texture arrives as base64 PNG. It is kept here and replaced by a URL, so the 10 state messages per
          // second stay small.
          textures.clear();
          manualObstacles = JSON.parse(text).map((o, i) => {
            const { texture, ...rest } = { id: `manual-${i + 1}`, source: "manual", yaw: 0, ...o };
            if (!texture) return rest;
            const png = Buffer.from(texture, "base64");
            textures.set(rest.id, png);
            // The hash makes the URL change exactly when the picture changes, which is what tells the UI to reload it.
            const version = crypto.createHash("sha1").update(png).digest("hex").slice(0, 10);
            return { ...rest, textureUrl: `http://localhost:${PORT}/textures/${encodeURIComponent(rest.id)}.png?v=${version}` };
          });
          reply(200, manualObstacles);
        }
        else reply(200, setObjects(JSON.parse(text)));
      } catch (error) { reply(400, { error: error.message }); }
    });
    return;
  }
  reply(404, { error: "not found" });
});

const sockets = new WebSocketServer({ server, path: "/ws" });
sockets.on("connection", (socket) => {
  let walking = false;  // this client's last command was a move, so the robot is walking on its behalf
  socket.on("message", (data) => {
    let envelope;
    try { envelope = JSON.parse(data); } catch { return; }
    if (envelope.type !== "command") return;
    walking = MOVES.includes(envelope.data.type);
    socket.send(JSON.stringify({ type: "ack", data: handleCommand(envelope.data) }));
  });
  // A controller that drops off the WiFi while a direction is held can never send its stop.
  socket.on("close", () => { if (walking) move("stop"); });
});

setInterval(() => {
  const state = latestState = buildState();
  // A walk keeps going until the next command, so it is checked on every tick, not only when it starts.
  const walk = (drive.mode === "software" && gaitEngine.command) || (robotState?.command ?? "");
  if (state.robots[0]?.tracking && leavesArena(state.robots[0], state.arena, walk, robotRadius)) { edgeStops++; move("stop"); }
  navigator.step(state.robots[0], state.arena, state.obstacles);
  const message = JSON.stringify({ type: "state", data: state });
  for (const socket of sockets.clients) if (socket.readyState === WebSocket.OPEN) socket.send(message);
}, 100);

connectTracker();
connectRobot();
server.listen(PORT, () => console.log(`bridge: ws://localhost:${PORT}/ws  tracker ${TRACKER_HOST}:${TRACKER_PORT}  robot ${ROBOT_URL}`));
