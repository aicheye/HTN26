// Tries a gait on the real robot, without the tracker or the bridge. Join the Sesame-Controller WiFi, then:
//   node walk.mjs <forward|backward|left|right> [seconds] [--gait trot|firmware|builtin] [--trim 0.1] [--delay 100]
// --gait builtin sends the firmware's own command, for comparison. trot and firmware are streamed from here.
// --trim above 0 shortens the right stride (fixes veering left). Below 0 shortens the left stride.
import { GaitEngine } from "./gait.mjs";

const args = process.argv.slice(2), flag = (name, fallback) => (args.includes(name) ? args[args.indexOf(name) + 1] : fallback);
const command = args[0] ?? "forward", seconds = Number(args[1] && !args[1].startsWith("--") ? args[1] : 6);
const gait = flag("--gait", "trot"), trim = Number(flag("--trim", 0)), frameDelay = Number(flag("--delay", 100));
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
    setTimeout(() => { console.log(`done, ${frames} state messages received`); process.exit(0); }, 1500);
  }, seconds * 1000);
};
