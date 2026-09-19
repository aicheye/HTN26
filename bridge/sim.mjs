// Simulated tracker and robot, for working on the frontend and the bridge without hardware.
//   cd bridge && npm run sim
// Starts a fake Pi tracker (TCP 9003) and a fake robot firmware (WebSocket 8081) that moves a simulated
// robot in response to commands, then starts bridge.mjs pointed at both. The frontend connects to
// ws://localhost:8080/ws exactly as it would with the real hardware.
import { spawn } from "node:child_process";
import net from "node:net";
import { WebSocketServer } from "ws";

const FLOOR = [76, 60];            // cm, same rectangle as the real arena
const WALK_CM_PER_S = 5;           // guesses, the real robot has not been measured
const TURN_DEG_PER_S = 30;
const POSE_SECONDS = 2;
const MOVES = ["forward", "backward", "left", "right"];
// Camera looking straight down from 130 cm over the middle of the floor.
const camera = { f: 1693, cx: 1152, cy: 648, rvec: [Math.PI, 0, 0], tvec: [-FLOOR[0] / 2, FLOOR[1] / 2, 130] };

const robot = { x: 20, y: 15, heading: 0, command: "", face: "idle", poseUntil: 0 };
const started = Date.now();

const robotClients = new WebSocketServer({ port: 8081 });
const robotState = () => JSON.stringify({ command: robot.command, face: robot.face, servos: {} });
robotClients.on("connection", (socket) => {
  socket.send(robotState());
  socket.on("message", (data) => {
    const message = JSON.parse(data);
    if (message.face) robot.face = message.face;
    if (message.command === "stop") robot.command = "";
    else if (message.command) {
      robot.command = message.command;
      robot.poseUntil = MOVES.includes(message.command) ? 0 : Date.now() + POSE_SECONDS * 1000;
    }
    for (const client of robotClients.clients) client.send(robotState());
  });
});

const trackerClients = new Set();
net.createServer((socket) => {
  trackerClients.add(socket);
  socket.on("close", () => trackerClients.delete(socket));
  socket.on("error", () => trackerClients.delete(socket));
}).listen(9003);

const STEP_MS = 1000 / 15;
setInterval(() => {
  const dt = STEP_MS / 1000, radians = (robot.heading * Math.PI) / 180;
  if (robot.command === "forward" || robot.command === "backward") {
    const direction = robot.command === "forward" ? 1 : -1;
    robot.x = Math.min(FLOOR[0], Math.max(0, robot.x + direction * WALK_CM_PER_S * dt * Math.cos(radians)));
    robot.y = Math.min(FLOOR[1], Math.max(0, robot.y + direction * WALK_CM_PER_S * dt * Math.sin(radians)));
  } else if (robot.command === "left" || robot.command === "right") {
    robot.heading += (robot.command === "left" ? 1 : -1) * TURN_DEG_PER_S * dt;
    robot.heading = ((robot.heading + 540) % 360) - 180;
  } else if (robot.command && Date.now() > robot.poseUntil) {
    robot.command = "";
    for (const client of robotClients.clients) client.send(robotState());
  }
  const pixel = (x, y) => [camera.cx + (camera.f * (x - FLOOR[0] / 2)) / 130, camera.cy - (camera.f * (y - FLOOR[1] / 2)) / 130];
  const line = JSON.stringify({
    t: Date.now() - started, calibrated: true, frame: [2304, 1296], floor: FLOOR, zUp: true, floorMarkers: 4, fps: 15,
    markers: [0, 1, 2, 3, 4, 5],
    robot: { x: robot.x, y: robot.y, z: 8, heading: robot.heading, px: pixel(robot.x, robot.y) },
    arm: { x: 70, y: 52, z: 5, heading: -135, px: pixel(70, 52) },
    camera,
  }) + "\n";
  for (const socket of trackerClients) socket.write(line);
}, STEP_MS);

const bridge = spawn("node", ["bridge.mjs"], {
  env: { ...process.env, TRACKER_HOST: "127.0.0.1", TRACKER_PORT: "9003", ROBOT_URL: "ws://127.0.0.1:8081" },
  stdio: "inherit",
});
const stop = () => { bridge.kill(); process.exit(0); };
process.on("SIGINT", stop);
process.on("SIGTERM", stop);
console.log("sim: fake tracker on tcp 9003, fake robot on ws 8081");
