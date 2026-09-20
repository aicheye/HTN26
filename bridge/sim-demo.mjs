// Pathfinding demo for the simulator: puts objects on the arena and sends the robot back and forth between two
// goals on either side of one, so the walk around it can be watched in the web UI.
//   cd bridge && npm run sim          in one terminal
//   cd bridge && npm run sim:demo     in another, then open the web UI (VITE_SOURCE=ws npx vite, port 5173)
// The objects carry their camera photos when a scan of a recording has been saved next to it:
//   BRIDGE_URL=http://127.0.0.1:9 vision/.venv/bin/python vision/scan.py recordings/rec-006
// writes recordings/rec-006/obstacles.json. Without that file plain boxes are used.
// Only two objects: each one blocks its own size plus 0.10 m on every side, so a 0.63 m arena fills up quickly.
import fs from "node:fs";

const BRIDGE = process.env.BRIDGE_URL ?? "http://localhost:8080";
const GOALS = [{ x: 0.17, y: 0.48 }, { x: 0.17, y: 0.16 }];
const PLACES = { "orange-box-6": { x: 0.17, y: 0.32, yaw: 0 }, "green-object-5": { x: 0.56, y: 0.08, yaw: 0.3 } };
const SAVED = new URL("../recordings/rec-006/obstacles.json", import.meta.url);

function moved(o, to) {
  const turn = to.yaw - o.yaw, c = Math.cos(turn), s = Math.sin(turn);
  const points = (o.points ?? []).map((p) => ({ x: to.x + (p.x - o.x) * c - (p.y - o.y) * s, y: to.y + (p.x - o.x) * s + (p.y - o.y) * c }));
  return { ...o, ...to, points };
}
const saved = fs.existsSync(SAVED) ? JSON.parse(fs.readFileSync(SAVED, "utf8")) : [];
const scene = Object.entries(PLACES).map(([id, to]) => {
  const found = saved.find((o) => o.id === id);
  return found ? moved(found, to) : { id, source: "cv", shape: "rect", width: 0.1, length: 0.07, color: "#f97316", ...to };
});
await fetch(`${BRIDGE}/obstacles`, { method: "POST", body: JSON.stringify(scene) });
console.log(`${scene.length} objects placed${saved.length ? ", with their photos" : " as plain boxes (no saved scan found)"}`);

const socket = new WebSocket(`${BRIDGE.replace("http", "ws")}/ws`);
let leg = 0, sentAt = 0, last = "";
socket.onmessage = (event) => {
  const envelope = JSON.parse(event.data);
  if (envelope.type !== "state") return;
  const { mission, robots } = envelope.data, now = Date.now();
  if (mission.state !== last) { console.log(`  ${mission.state}${mission.detail ? `: ${mission.detail}` : ""}`); last = mission.state; }
  const idle = ["idle", "done", "failed"].includes(mission.state);
  if (!robots[0] || !idle || now - sentAt < 3000) return;
  const target = GOALS[leg++ % GOALS.length];
  sentAt = now;
  console.log(`goto (${target.x}, ${target.y})`);
  socket.send(JSON.stringify({ type: "command", data: { id: `demo-${leg}`, ts: now, robotId: robots[0].id, type: "goto", target } }));
};
