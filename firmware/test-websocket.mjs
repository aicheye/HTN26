// Tests the robot's WebSocket API. Join the Sesame-Controller WiFi, then run:
//   node test-websocket.mjs [host]
// Needs Node 22 or newer (built-in WebSocket). Moves servo R1 by 10 degrees and back.

const host = process.argv[2] ?? "192.168.4.1";
const url = `ws://${host}:81`;
const TIMEOUT_MS = 3000;

const ws = new WebSocket(url);
const waiters = [];
let failures = 0;

ws.onmessage = (event) => {
  const msg = JSON.parse(event.data);
  for (let i = waiters.length - 1; i >= 0; i--) {
    if (waiters[i].match(msg)) waiters.splice(i, 1)[0].resolve(msg);
  }
};

// Resolves with the first message that satisfies `match`, or rejects after TIMEOUT_MS.
function waitFor(match, label) {
  return new Promise((resolve, reject) => {
    const waiter = { match, resolve };
    waiters.push(waiter);
    setTimeout(() => {
      const i = waiters.indexOf(waiter);
      if (i !== -1) {
        waiters.splice(i, 1);
        reject(new Error(`no ${label} within ${TIMEOUT_MS} ms`));
      }
    }, TIMEOUT_MS);
  });
}

async function check(name, fn) {
  try {
    const detail = await fn();
    console.log(`PASS  ${name}${detail ? `  (${detail})` : ""}`);
  } catch (err) {
    failures++;
    console.log(`FAIL  ${name}: ${err.message}`);
  }
}

async function moveServo(name, angle) {
  const start = performance.now();
  const reply = waitFor((m) => m.servos?.[name] === angle, `state with ${name}=${angle}`);
  ws.send(JSON.stringify({ servos: { [name]: angle } }));
  await reply;
  return performance.now() - start;
}

const firstState = waitFor((m) => m.servos !== undefined, "state message on connect");

function connectFailed() {
  console.log(`FAIL  connect to ${url}. Is this machine on the Sesame-Controller WiFi?`);
  process.exit(1);
}
ws.onerror = connectFailed;
const connectTimer = setTimeout(connectFailed, TIMEOUT_MS);
firstState.catch(() => {}); // reported by the first check, or by connectFailed

ws.onopen = async () => {
  clearTimeout(connectTimer);
  let state;
  await check("state message on connect", async () => {
    state = await firstState;
    return JSON.stringify(state);
  });

  const home = state?.servos?.R1 ?? 90;
  const target = home <= 170 ? home + 10 : home - 10;

  await check(`servo write and readback, R1 ${home} -> ${target} -> ${home}`, async () => {
    const out = await moveServo("R1", target);
    const back = await moveServo("R1", home);
    return `round trips ${out.toFixed(0)} ms and ${back.toFixed(0)} ms`;
  });

  await check("invalid angle is rejected", async () => {
    const reply = waitFor((m) => m.error !== undefined, "error reply");
    ws.send(JSON.stringify({ servos: { R1: 999 } }));
    return (await reply).error;
  });

  await check("invalid JSON is rejected", async () => {
    const reply = waitFor((m) => m.error !== undefined, "error reply");
    ws.send("not json");
    return (await reply).error;
  });

  await check("face change is broadcast", async () => {
    const face = state?.face === "happy" ? "sad" : "happy";
    const reply = waitFor((m) => m.face === face, `state with face=${face}`);
    ws.send(JSON.stringify({ face }));
    await reply;
    return `face=${face}`;
  });

  ws.close();
  console.log(failures === 0 ? "\nAll checks passed." : `\n${failures} check(s) failed.`);
  process.exit(failures === 0 ? 0 : 1);
};
