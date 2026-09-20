import { POSES, type PoseName } from "../types/world";
import { MAX_PLAN_STEPS, VOICE_PULSE_MS, type VoiceIntent } from "./voiceCommands";
import type { VoiceSnapshot } from "./voiceSession";

export function voiceEndpoint(wsUrl: string): string {
  const url = new URL(wsUrl);
  if (!["ws:", "wss:"].includes(url.protocol) || url.username || url.password) throw new Error("Check the bridge URL in Settings.");
  url.protocol = url.protocol === "wss:" ? "https:" : "http:";
  url.pathname = "/voice";
  url.search = url.hash = "";
  return url.href;
}

export function validateVoiceIntent(value: unknown): VoiceIntent | null {
  if (!value || typeof value !== "object" || Array.isArray(value)) return null;
  const intent = value as Record<string, unknown>;
  const keys = Object.keys(intent).sort().join(",");
  if (intent.type === "stop" && keys === "type") return { type: "stop" };
  if (["forward", "backward", "left", "right"].includes(intent.type as string) && keys === "durationMs,type" && intent.durationMs === VOICE_PULSE_MS) {
    return { type: intent.type as "forward" | "backward" | "left" | "right", durationMs: VOICE_PULSE_MS };
  }
  if (intent.type === "pose" && keys === "pose,type" && POSES.includes(intent.pose as PoseName)) return { type: "pose", pose: intent.pose as PoseName };
  if (intent.type === "goto" && typeof intent.name === "string" && intent.name.trim() && intent.name.length <= 80) {
    if (keys === "name,type") return { type: "goto", name: intent.name };
    if (keys === "name,relation,type" && (intent.relation === "at" || intent.relation === "past")) return { type: "goto", name: intent.name, relation: intent.relation };
  }
  return null;
}

/** A plan is 1–4 validated steps; stop may only stand alone. */
export function validateVoicePlan(value: unknown): VoiceIntent[] | null {
  if (!Array.isArray(value) || !value.length || value.length > MAX_PLAN_STEPS) return null;
  const plan = value.map(validateVoiceIntent);
  if (plan.some((step) => !step) || (plan.length > 1 && plan.some((step) => step!.type === "stop"))) return null;
  return plan as VoiceIntent[];
}

export type Clarification = { transcript: string; question: string };

export async function requestVoice(audio: Blob, snapshot: VoiceSnapshot, wsUrl: string, source: "mock" | "ws", signal: AbortSignal, clarification?: Clarification) {
  const body = new FormData();
  const extension = audio.type.includes("mp4") ? "mp4" : audio.type.includes("ogg") ? "ogg" : "webm";
  body.set("audio", audio, `command.${extension}`);
  body.set("source", source);
  body.set("robotId", snapshot.robotId ?? "");
  if (clarification) body.set("context", JSON.stringify(clarification));
  if (source === "mock") {
    const robot = snapshot.state?.robots.find((r) => r.id === snapshot.robotId);
    body.set("scene", JSON.stringify({ robot: robot ? { id: robot.id, x: robot.x, y: robot.y, yaw: robot.yaw } : null,
      obstacles: (snapshot.state?.obstacles ?? []).slice(0, 50).map(({ id, x, y, color, shape, width, length, radius, height }) => ({ id, x, y, color, shape, width, length, radius, height })) }));
  }
  const endpoint = voiceEndpoint(wsUrl);
  let response: Response;
  try {
    response = await fetch(endpoint, { method: "POST", body, signal, credentials: "omit" });
  } catch (error) {
    if (signal.aborted) throw error;
    throw new Error("Voice bridge is unavailable. Start the bridge with GROQ_API_KEY set.");
  }
  let data;
  try { data = await response.json(); } catch { throw new Error("Voice endpoint is unavailable. Restart the updated bridge and try again."); }
  if (!data || typeof data !== "object" || Array.isArray(data)) throw new Error("Invalid voice response. Nothing was sent.");
  if (!response.ok) throw new Error(typeof data.error === "string" ? data.error.slice(0, 200) : "Voice service could not process the command.");
  if (typeof data.transcript !== "string" || data.transcript.length > 300) throw new Error("Invalid voice response. Try again.");
  const plan = data.plan === undefined || (Array.isArray(data.plan) && !data.plan.length) ? [] : validateVoicePlan(data.plan);
  if (!plan) throw new Error("Unsupported voice action. Nothing was sent.");
  return { transcript: data.transcript, plan, clarify: data.clarify === true, message: typeof data.message === "string" ? data.message.slice(0, 160) : "Please try one clear command." };
}
