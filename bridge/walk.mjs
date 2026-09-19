// Tries a gait on the real robot, without the tracker or the bridge. Join the Sesame-Controller WiFi, then:
//   node walk.mjs <forward|backward|left|right> [seconds] [--gait trot|firmware|builtin] [--trim 0.1] [--delay 100]
// --gait builtin sends the firmware's own command, for comparison. trot and firmware are streamed from here.
// --trim above 0 shortens the right stride (fixes veering left). Below 0 shortens the left stride.
// Defaults for gait, trim and delay come from drive.json when it exists. Frame timings are written to walk-log.json.
import fs from "node:fs";
import { GaitEngine } from "./gait.mjs";

const saved = fs.existsSync(new URL("./drive.json", import.meta.url)) ? JSON.parse(fs.readFileSync(new URL("./drive.json", import.meta.url), "utf8")) : {};

const args = process.argv.slice(2), flag = (name, fallback) => (args.includes(name) ? args[args.indexOf(name) + 1] : fallback);
const command = args[0] ?? "forward", seconds = Number(args[1] && !args[1].startsWith("--") ? args[1] : 6);
const gait = flag("--gait", saved.gait ?? "trot"), trim = Number(flag("--trim", saved.trim ?? 0)), frameDelay = Number(flag("--delay", saved.frameDelay ?? 100));
const socket = new WebSocket(process.env.ROBOT_URL ?? "ws://192.168.4.1:81");
const engine = new GaitEngine((servos) => socket.send(JSON.stringify({ servos })), { gait, trim, frameDelay });
let frames = 0;
socket.onmessage = (event) => { const state = JSON.parse(event.data); if (state.servos) { frames++; engine.onRobotState(state); } };
socket.onerror = () => { console.log("cannot reach the robot. Is this machine on the Sesame-Controller WiFi?"); process.exit(1); };
socket.onopen = () => {
  console.log(`${command} for ${seconds} s, gait ${gait}, trim ${trim}, frame delay ${frameDelay} ms`);
  if (gait === "builtin") socket.send(JSON.stringify({ command }));
  else engine.set(command);
  setTimeout(() => {
    if (gait === "builtin") socket.send(JSON.stringify({ command: "stop" }));
    else engine.set("stop");
    setTimeout(() => {
      const t = engine.timings, sorted = t.map((f) => f.confirmMs).sort((a, b) => a - b);
      const gaps = t.slice(1).map((f, i) => f.at - t[i].at);
      if (t.length) {
        console.log(`${t.length} frames. confirm time: median ${sorted[sorted.length >> 1]} ms, 95th percentile ${sorted[Math.floor(sorted.length * 0.95)]} ms, max ${sorted.at(-1)} ms`);
        console.log(`${t.filter((f) => f.timedOut).length} frames were never confirmed. longest gap between frames ${Math.max(...gaps)} ms`);
      }
      fs.writeFileSync(new URL("./walk-log.json", import.meta.url), JSON.stringify({ command, gait, trim, frameDelay, stateMessages: frames, timings: t }, null, 1));
      process.exit(0);
    }, 1500);
  }, seconds * 1000);
};
