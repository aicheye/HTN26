import assert from "node:assert/strict";
import http from "node:http";
import test from "node:test";
import { createVoiceHandler, interpretVoice } from "./voice.mjs";

const scene = { robot: { id: "sesame-1", x: 0.2, y: 0.3, yaw: 0 }, obstacles: [{ id: "chocolate-1", x: 0.5, y: 0.3, color: "#654321" }] };
const audio = new Blob(["test audio"], { type: "audio/webm" });
const transcript = (text) => ({ text, duration: 2, segments: [{ no_speech_prob: 0.01, avg_logprob: -0.1 }] });
const step = (action, extra = {}) => ({ action, pose: "", obstacleId: "", relation: "", amount: 0, unit: "", ...extra });
const plan = (...steps) => ({ choices: [{ message: { content: JSON.stringify({ steps, reason: "" }) } }] });
const refusal = (reason) => ({ choices: [{ message: { content: JSON.stringify({ steps: [], reason }) } }] });
const completion = (action, extra = {}) => plan(step(action, action === "goto" && !extra.relation ? { relation: "at", ...extra } : extra));
const json = (body) => new Response(JSON.stringify(body), { headers: { "Content-Type": "application/json" } });

function groq(responses) {
  const calls = [];
  return { calls, fetchImpl: async (url, options) => { calls.push({ url, ...options }); return json(responses.shift()); } };
}

test("Groq transcribes audio then interprets a detected object without choosing coordinates", async () => {
  const mock = groq([transcript("Could you walk over to the chocolate?"), completion("goto", { obstacleId: "chocolate-1" })]);
  const result = await interpretVoice({ audio, scene, apiKey: "test-only", signal: new AbortController().signal, fetchImpl: mock.fetchImpl });
  assert.deepEqual(result.intent, { type: "goto", name: "chocolate-1", relation: "at" });
  assert.equal(mock.calls.length, 2);
  assert.equal(mock.calls[0].body.get("model"), "whisper-large-v3-turbo");
  assert.equal(mock.calls[0].body.get("file").type, "audio/webm");
  const request = JSON.parse(mock.calls[1].body);
  assert.equal(request.response_format.json_schema.strict, true);
  assert.ok(request.messages[1].content.includes("chocolate-1"));
  assert.equal(request.model, "openai/gpt-oss-20b");
});

test("Groq actions stay bounded and unknown destinations or output fields are rejected", async () => {
  for (const [action, expected] of [["forward", { type: "forward", durationMs: 500 }], ["pose", { type: "pose", pose: "wave" }]]) {
    const mock = groq([transcript("Do something"), completion(action, action === "pose" ? { pose: "wave" } : {})]);
    assert.deepEqual((await interpretVoice({ audio, scene, apiKey: "test-only", fetchImpl: mock.fetchImpl })).intent, expected);
  }
  for (const bad of [completion("goto", { obstacleId: "invented" }), completion("forward", { durationMs: 999999 }), completion("pose", { pose: "launch" })]) {
    const mock = groq([transcript("Please move"), bad]);
    await assert.rejects(interpretVoice({ audio, scene, apiKey: "test-only", fetchImpl: mock.fetchImpl }), /invalid|unsupported/i);
  }
});

test("Silence, uncertain audio, negation and compound commands never produce movement", async () => {
  for (const data of [transcript(""), { ...transcript("forward"), segments: [{ no_speech_prob: 0.9, avg_logprob: -2 }] }, transcript("don't move forward"), transcript("forward unless it is blocked")]) {
    const mock = groq([data]);
    const result = await interpretVoice({ audio, scene, apiKey: "test-only", fetchImpl: mock.fetchImpl });
    assert.equal(result.intent, null);
    assert.equal(mock.calls.length, 1);
  }
  const mock = groq([transcript("stop")]);
  assert.deepEqual((await interpretVoice({ audio, scene, apiKey: "test-only", fetchImpl: mock.fetchImpl })).intent, { type: "stop" });
  assert.equal(mock.calls.length, 1);
});

test("Groq refusal and provider errors do not disclose provider payloads", async () => {
  const mock = groq([transcript("go to the thing"), refusal("Which object did you mean?")]);
  const result = await interpretVoice({ audio, scene, apiKey: "test-only", fetchImpl: mock.fetchImpl });
  assert.equal(result.intent, null);
  assert.equal(result.message, "Which object did you mean?");
  await assert.rejects(interpretVoice({ audio, scene, apiKey: "test-only", fetchImpl: async () => new Response("private provider details", { status: 401 }) }), /Groq API key/);
});

async function serverFor(t, options = {}) {
  const handler = createVoiceHandler({ getState: () => ({ robots: [scene.robot], obstacles: scene.obstacles }), ...options });
  const server = http.createServer(handler);
  await new Promise((resolve) => server.listen(0, "127.0.0.1", resolve));
  t.after(() => new Promise((resolve) => { server.close(resolve); server.closeAllConnections(); }));
  return `http://127.0.0.1:${server.address().port}/voice`;
}
const headers = { Origin: "http://localhost:5173" };
function upload() {
  const body = new FormData();
  body.set("audio", audio, "command.webm");
  body.set("source", "mock");
  body.set("scene", JSON.stringify(scene));
  return body;
}

test("Voice HTTP endpoint restricts origins, reports missing setup and never calls Groq without a key", async (t) => {
  const url = await serverFor(t, { apiKey: () => "", fetchImpl: () => assert.fail("must not call Groq") });
  const denied = await fetch(url, { method: "POST", headers: { Origin: "https://untrusted.example" }, body: upload() });
  assert.equal(denied.status, 403);
  assert.equal(denied.headers.get("access-control-allow-origin"), null);
  const preflight = await fetch(url, { method: "OPTIONS", headers });
  assert.equal(preflight.status, 204);
  const missing = await fetch(url, { method: "POST", headers, body: upload() });
  assert.equal(missing.status, 503);
  assert.match((await missing.json()).error, /GROQ_API_KEY/);
});

test("Voice HTTP accepts bounded audio and rejects oversized or wrong-type uploads", async (t) => {
  const mock = groq([transcript("give me a wave"), completion("pose", { pose: "wave" })]);
  const url = await serverFor(t, { apiKey: () => "test-only", fetchImpl: mock.fetchImpl });
  const response = await fetch(url, { method: "POST", headers, body: upload() });
  assert.equal(response.status, 200);
  assert.deepEqual((await response.json()).intent, { type: "pose", pose: "wave" });
  const wrong = await fetch(url, { method: "POST", headers: { ...headers, "Content-Type": "application/json" }, body: "{}" });
  assert.equal(wrong.status, 415);
  const large = upload();
  large.set("audio", new Blob([new Uint8Array(2 * 1024 * 1024 + 1)], { type: "audio/webm" }), "large.webm");
  const oversized = await fetch(url, { method: "POST", headers, body: large });
  assert.equal(oversized.status, 413);
  assert.equal(mock.calls.length, 2);
});

test("Live voice uses the bridge scene, not client-supplied objects", async (t) => {
  const mock = groq([transcript("go to chocolate"), completion("goto", { obstacleId: "chocolate-1" })]);
  const url = await serverFor(t, { apiKey: () => "test-only", fetchImpl: mock.fetchImpl });
  const body = upload();
  body.set("source", "ws");
  body.set("robotId", "sesame-1");
  body.set("scene", JSON.stringify({ obstacles: [{ id: "invented", x: 1, y: 2 }] }));
  const result = await (await fetch(url, { method: "POST", headers, body })).json();
  assert.deepEqual(result.intent, { type: "goto", name: "chocolate-1", relation: "at" });
  assert.ok(!JSON.parse(mock.calls[1].body).messages[1].content.includes("invented"));
});

test("Cancelling an HTTP voice request aborts the upstream Groq request", async (t) => {
  let started, aborted;
  const ready = new Promise((resolve) => { started = resolve; });
  const cancelled = new Promise((resolve) => { aborted = resolve; });
  const url = await serverFor(t, { apiKey: () => "test-only", fetchImpl: async (_url, { signal }) => {
    started();
    return new Promise((_resolve, reject) => signal.addEventListener("abort", () => { aborted(); reject(new DOMException("Aborted", "AbortError")); }, { once: true }));
  } });
  const controller = new AbortController();
  const request = fetch(url, { method: "POST", headers, body: upload(), signal: controller.signal });
  await ready;
  controller.abort();
  await assert.rejects(request, /abort/i);
  await cancelled;
});

test("Voice endpoint rate-limits requests before making more paid API calls", async (t) => {
  const mock = groq(Array.from({ length: 20 }, () => transcript("")));
  const url = await serverFor(t, { apiKey: () => "test-only", fetchImpl: mock.fetchImpl });
  for (let i = 0; i < 20; i++) {
    const response = await fetch(url, { method: "POST", headers, body: upload() });
    assert.equal(response.status, 200);
    await response.json();
  }
  const limited = await fetch(url, { method: "POST", headers, body: upload() });
  assert.equal(limited.status, 429);
  assert.equal(mock.calls.length, 20);
});

test("Spatial and multi-step instructions become a validated ordered plan", async () => {
  const two = { ...scene, obstacles: [...scene.obstacles, { id: "obstacle-2", x: 0.5, y: 0.5, color: "#0000ff" }] };
  const mock = groq([transcript("Go past the blue barrier and then wave"), plan(step("goto", { obstacleId: "obstacle-2", relation: "past" }), step("pose", { pose: "wave" }))]);
  const result = await interpretVoice({ audio, scene: two, apiKey: "test-only", fetchImpl: mock.fetchImpl });
  assert.deepEqual(result.plan, [{ type: "goto", name: "obstacle-2", relation: "past" }, { type: "pose", pose: "wave" }]);
  assert.equal(result.intent, null);
  const request = JSON.parse(mock.calls[1].body);
  assert.equal(request.response_format.json_schema.schema.properties.steps.maxItems, 4);
  assert.ok(request.messages[1].content.includes("#0000ff"));
});

test("Plans reject partial, oversized, stop-mixed or malformed steps", async () => {
  const bad = [
    plan(step("forward"), step("goto", { obstacleId: "invented", relation: "at" })),
    plan(...Array.from({ length: 5 }, () => step("left"))),
    plan(step("forward"), step("stop")),
    plan(step("goto", { obstacleId: "chocolate-1" })),
    plan(step("forward", { relation: "past" })),
  ];
  for (const data of bad) {
    const mock = groq([transcript("Do a sequence"), data]);
    await assert.rejects(interpretVoice({ audio, scene, apiKey: "test-only", fetchImpl: mock.fetchImpl }), /invalid|unsupported/i);
  }
});

test("A spoken answer is interpreted together with the question that prompted it", async () => {
  const mock = groq([transcript("the chocolate one"), completion("goto", { obstacleId: "chocolate-1", relation: "past" })]);
  const clarification = { transcript: "go past the barrier", question: "Please specify which obstacle or location to go to." };
  const result = await interpretVoice({ audio, scene, clarification, apiKey: "test-only", fetchImpl: mock.fetchImpl });
  assert.deepEqual(result.plan, [{ type: "goto", name: "chocolate-1", relation: "past" }]);
  const user = JSON.parse(JSON.parse(mock.calls[1].body).messages[1].content);
  assert.equal(user.earlierRequest, "go past the barrier");
  assert.equal(user.questionYouAsked, clarification.question);
  const asked = groq([transcript("go somewhere"), refusal("Which object?")]);
  assert.equal((await interpretVoice({ audio, scene, apiKey: "test-only", fetchImpl: asked.fetchImpl })).clarify, true);
  const plain = groq([transcript("forward"), completion("forward")]);
  await interpretVoice({ audio, scene, clarification: { transcript: 5 }, apiKey: "test-only", fetchImpl: plain.fetchImpl });
  assert.ok(!("earlierRequest" in JSON.parse(JSON.parse(plain.calls[1].body).messages[1].content)));
});

test("Spoken amounts become measured turns and walks, converted and range-checked", async () => {
  const mock = groq([transcript("Turn to the right and walk 5 steps"), plan(step("right", { amount: 90, unit: "degrees" }), step("forward", { amount: 5, unit: "steps" }))]);
  const result = await interpretVoice({ audio, scene, apiKey: "test-only", fetchImpl: mock.fetchImpl });
  assert.deepEqual(result.plan, [{ type: "right", durationMs: 500, angleDeg: 90 }, { type: "forward", durationMs: 500, distanceCm: 25 }]);
  for (const [extra, expected] of [[{ amount: 0.2, unit: "meters" }, 20], [{ amount: 4, unit: "inches" }, 10.2], [{ amount: 12, unit: "cm" }, 12]]) {
    const one = groq([transcript("Walk"), plan(step("backward", extra))]);
    assert.equal((await interpretVoice({ audio, scene, apiKey: "test-only", fetchImpl: one.fetchImpl })).plan[0].distanceCm, expected);
  }
});

test("Out-of-range or mismatched amounts are refused with a useful message", async () => {
  for (const bad of [step("forward", { amount: 100, unit: "steps" }), step("left", { amount: 2, unit: "degrees" }), step("left", { amount: 400, unit: "degrees" })]) {
    const mock = groq([transcript("Move a lot"), plan(bad)]);
    await assert.rejects(interpretVoice({ audio, scene, apiKey: "test-only", fetchImpl: mock.fetchImpl }), /out of range/);
  }
  for (const bad of [step("left", { amount: 30, unit: "steps" }), step("forward", { amount: 30, unit: "degrees" }), step("forward", { amount: 30 }), step("forward", { unit: "cm" }), step("pose", { pose: "wave", amount: 5, unit: "steps" })]) {
    const mock = groq([transcript("Move"), plan(bad)]);
    await assert.rejects(interpretVoice({ audio, scene, apiKey: "test-only", fetchImpl: mock.fetchImpl }), /unsupported/);
  }
});

test("Corners are offered to Groq with map-view labels and robot-relative offsets", async () => {
  const cornerScene = { robot: { id: "sesame-1", x: 0.2, y: 0.3, yaw: Math.PI / 2 }, arena: { width: 0.6, length: 0.6 }, obstacles: [] };
  const mock = groq([transcript("Go to the top left corner"), plan(step("goto", { obstacleId: "corner-top-left", relation: "at" }))]);
  const result = await interpretVoice({ audio, scene: cornerScene, apiKey: "test-only", fetchImpl: mock.fetchImpl });
  assert.deepEqual(result.plan, [{ type: "goto", name: "corner-top-left", relation: "at" }]);
  const request = JSON.parse(mock.calls[1].body);
  const { corners } = JSON.parse(request.messages[1].content).scene;
  assert.equal(corners.length, 4);
  assert.match(corners.find((c) => c.id === "corner-top-left").onMap, /top left/);
  // robot at (0.2, 0.3) facing +y: the top-left corner (0, 0.6) is 30 cm ahead and 20 cm to the left
  const topLeft = corners.find((c) => c.id === "corner-top-left");
  assert.equal(topLeft.forwardCm, 30);
  assert.equal(topLeft.leftCm, 20);
  assert.ok(request.response_format.json_schema.schema.properties.steps.items.properties.obstacleId.enum.includes("corner-bottom-right"));
});
