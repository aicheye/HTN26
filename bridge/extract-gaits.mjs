// Prints the firmware's walk and turn gaits as frame tables, read from firmware/src/movement-sequences.h.
// A frame is the set of servo angles written between two pressingCheck() calls.
import fs from "node:fs";
const header = fs.readFileSync(new URL("../firmware/src/movement-sequences.h", import.meta.url), "utf8");
export function extractGait(functionName) {
  const body = header.slice(header.indexOf(`inline void ${functionName}()`));
  const code = body.slice(0, body.indexOf("runStandPose(1)"));  // every gait ends by standing
  const loopAt = code.indexOf("for (int i = 0; i < walkCycles");
  const frames = (text) => text.split("pressingCheck").map((part) => Object.fromEntries([...part.matchAll(/setServoAngle\((\w+), (\d+)\)/g)].map((m) => [m[1], Number(m[2])]))).filter((f) => Object.keys(f).length);
  return { start: frames(code.slice(0, loopAt)), cycle: frames(code.slice(loopAt)) };
}
export const FIRMWARE_GAITS = { forward: "runWalkPose", backward: "runWalkBackward", left: "runTurnLeft", right: "runTurnRight" };
if (process.argv[1] === new URL(import.meta.url).pathname) {
  for (const [name, fn] of Object.entries(FIRMWARE_GAITS)) console.log(name, JSON.stringify(extractGait(fn)));
}
