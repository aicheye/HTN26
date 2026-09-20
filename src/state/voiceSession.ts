import type { Command, CommandType, Point, WorldState } from "../types/world";
import type { ConnectionStatus } from "../sources/StateSource";
import { parseVoiceCommand, resolveVoiceTarget, voiceReadiness, type Landmark, type VoiceIntent } from "./voiceCommands";

export type SpeechUpdate = { listening: boolean; error?: string };
type SpeechResult = { results: ArrayLike<{ isFinal: boolean; 0: { transcript: string } }>; resultIndex?: number };
export interface Recognition {
  continuous: boolean;
  interimResults: boolean;
  lang: string;
  maxAlternatives: number;
  onresult: ((event: SpeechResult) => void) | null;
  onerror: ((event: { error: string }) => void) | null;
  onend: (() => void) | null;
  start(): void;
  abort(): void;
}
export type RecognitionConstructor = new () => Recognition;

export function speechRecognition(): RecognitionConstructor | undefined {
  const browser = window as unknown as { SpeechRecognition?: RecognitionConstructor; webkitSpeechRecognition?: RecognitionConstructor };
  return window.isSecureContext ? browser.SpeechRecognition ?? browser.webkitSpeechRecognition : undefined;
}

export class SpeechSession {
  private recognition: Recognition | null = null;
  private timer: ReturnType<typeof setTimeout> | undefined;

  constructor(private Recognition: RecognitionConstructor, private update: (state: SpeechUpdate) => void, private receive: (text: string) => void) {}

  start() {
    this.cancel();
    try {
      const recognition = new this.Recognition();
      this.recognition = recognition;
      recognition.continuous = false;
      recognition.interimResults = false;
      recognition.lang = "en-US";
      recognition.maxAlternatives = 1;
      recognition.onresult = (event) => {
        if (this.recognition !== recognition) return;
        const results = Array.from(event.results);
        if (!results.length || results.some((result) => !result.isFinal)) return;
        const text = results.map((result) => result[0]?.transcript?.trim() ?? "").join(" ").trim();
        this.cancel();
        if (text) this.receive(text);
        else this.update({ listening: false, error: "No speech recognized. Try again." });
      };
      recognition.onerror = ({ error }) => {
        if (this.recognition !== recognition) return;
        this.cancel();
        this.update({ listening: false, error: error === "not-allowed" || error === "service-not-allowed"
          ? "Microphone permission denied. Allow microphone access in your browser."
          : error === "audio-capture" ? "No microphone is available. Check your audio input."
          : error === "no-speech" ? "No speech recognized. Try again."
          : `Speech recognition failed (${error}). Try again or use manual controls.` });
      };
      recognition.onend = () => {
        if (this.recognition !== recognition) return;
        this.cancel();
        this.update({ listening: false, error: "No command recognized. Try again." });
      };
      this.timer = setTimeout(() => {
        if (this.recognition !== recognition) return;
        this.cancel();
        this.update({ listening: false, error: "Listening timed out. Try again." });
      }, 10000);
      this.update({ listening: true });
      recognition.start();
    } catch {
      this.cancel();
      this.update({ listening: false, error: "Could not start speech recognition. Check microphone access." });
    }
  }

  cancel() {
    const recognition = this.recognition;
    this.recognition = null;
    clearTimeout(this.timer);
    this.timer = undefined;
    if (recognition) {
      recognition.onresult = recognition.onerror = recognition.onend = null;
      try { recognition.abort(); } catch {}
    }
    this.update({ listening: false });
  }
}

export class VoiceAction {
  private stop: (() => void) | null = null;
  private timer: ReturnType<typeof setTimeout> | undefined;

  get active() { return this.stop !== null; }

  start(stop: () => void, durationMs?: number) {
    this.cancel();
    this.stop = stop;
    if (durationMs !== undefined) this.timer = setTimeout(() => this.cancel(), durationMs);
  }

  supersede() {
    clearTimeout(this.timer);
    this.timer = undefined;
    this.stop = null;
  }

  cancel() {
    const stop = this.stop;
    this.supersede();
    stop?.();
  }
}

export type VoiceSnapshot = {
  state: WorldState | null;
  robotId: string | null;
  status: ConnectionStatus;
  receivedAt: number;
  landmarks?: readonly Landmark[];
  send: (type: CommandType, extra?: Partial<Command>) => boolean | void;
};

const GOTO_LIMIT_MS = 30000;
const GOTO_ARRIVE_M = 0.06;
const POSE_SETTLE_MS = 1000;
const POSE_MAX_MS = 8000;

type ActiveStep = { intent: VoiceIntent; startedAt: number; target?: Point };

/** Runs a short validated plan one step at a time, advancing only when the current step has finished. */
export class VoiceExecutor {
  private action = new VoiceAction();
  private queue: VoiceIntent[] = [];
  private step: ActiveStep | null = null;
  private total = 0;

  constructor(private snapshot: () => VoiceSnapshot, private notify: (message: string, error: boolean) => void = () => {}) {}

  cancel() {
    this.queue = [];
    this.step = null;
    this.action.cancel();
  }

  /** Polled while a plan runs: aborts on lost safety conditions, advances on step completion. */
  check(): string | null {
    const step = this.step;
    if (!step) return null;
    const current = this.snapshot();
    const isGoto = step.intent.type === "goto";
    // The arm lifting this robot over an obstacle is part of navigating, so only goto tolerates it.
    const error = voiceReadiness(current.state, current.robotId, current.status, current.receivedAt, undefined, isGoto);
    if (error) { this.cancel(); return error; }
    const state = current.state!;
    const robot = state.robots.find((r) => r.id === current.robotId)!;
    const elapsed = Date.now() - step.startedAt;
    let done: boolean;
    if (step.intent.type === "goto") {
      if (!this.action.active) { this.cancel(); return `the robot did not reach ${step.intent.name} within ${GOTO_LIMIT_MS / 1000} seconds.`; }
      done = elapsed > 300 && (!state.arm || state.arm.mode === "idle") && !!step.target
        && Math.hypot(robot.x - step.target.x, robot.y - step.target.y) < GOTO_ARRIVE_M;
    } else if (step.intent.type === "pose") {
      done = elapsed >= POSE_MAX_MS || (elapsed >= POSE_SETTLE_MS && !robot.pose);
    } else {
      done = !this.action.active;
    }
    return done ? this.finishStep() : null;
  }

  execute(text: string): string {
    const intent = parseVoiceCommand(text);
    if (!intent) return "Not sent: please try one clear command.";
    return this.executeIntent(intent);
  }

  executeIntent(intent: VoiceIntent): string { return this.executePlan([intent]); }

  executePlan(plan: readonly VoiceIntent[]): string {
    const current = this.snapshot();
    if (!current.robotId) return "Not sent: select a robot first.";
    if (plan.length === 1 && plan[0].type === "stop") {
      this.queue = [];
      this.step = null;
      this.action.supersede();
      current.send("stop");
      return "Stop requested.";
    }
    const error = voiceReadiness(current.state, current.robotId, current.status, current.receivedAt);
    if (error) return `Not sent: ${error}`;
    // Reject the whole plan up front if any destination is unknown; positions are re-resolved when each step runs.
    for (const [i, intent] of plan.entries()) {
      if (intent.type === "goto") {
        const result = resolveVoiceTarget(intent.name, current.landmarks ?? [], current.state!, current.robotId, intent.relation);
        if (result.error) return `Not sent: ${plan.length > 1 ? `step ${i + 1}: ` : ""}${result.error}`;
      }
    }
    this.queue = plan.slice(1);
    this.step = null;
    this.total = plan.length;
    const message = this.startStep(plan[0]);
    if (!this.step) this.queue = [];
    return this.total > 1 && this.step ? `${message} (step 1 of ${this.total})` : message;
  }

  private startStep(intent: VoiceIntent): string {
    const current = this.snapshot();
    let extra: Partial<Command>;
    let durationMs: number | undefined;
    let target: Point | undefined;
    if (intent.type === "goto") {
      const result = resolveVoiceTarget(intent.name, current.landmarks ?? [], current.state!, current.robotId!, intent.relation);
      if (result.error) return `Not sent: ${result.error}`;
      target = result.target;
      extra = { target };
      durationMs = GOTO_LIMIT_MS;
    } else if (intent.type === "pose") {
      extra = { pose: intent.pose };
    } else if (intent.type === "stop") {
      return "Not sent: stop cannot be part of a sequence.";
    } else {
      extra = { durationMs: intent.durationMs };
      durationMs = intent.durationMs;
    }
    this.action.start(() => current.send("stop"), durationMs);
    if (current.send(intent.type, extra) === false) {
      this.action.supersede();
      return "Not sent: the connection became unavailable. Reconnect and try again.";
    }
    this.step = { intent, startedAt: Date.now(), target };
    return intent.type === "goto" ? `Sent: go ${intent.relation === "past" ? "past" : "to"} ${intent.name} (${GOTO_LIMIT_MS / 1000} second limit).`
      : intent.type === "pose" ? `Sent: ${intent.pose}.` : `Sent: ${intent.type} for ${durationMs} ms.`;
  }

  private finishStep(): string | null {
    const finished = this.step!;
    this.step = null;
    if (finished.intent.type === "goto") this.action.cancel(); else this.action.supersede();
    const next = this.queue.shift();
    if (!next) {
      if (this.total > 1) this.notify(`Finished all ${this.total} steps.`, false);
      return null;
    }
    const message = this.startStep(next);
    if (!this.step) { this.queue = []; return message.replace(/^Not sent: /, ""); }
    this.notify(`${message} (step ${this.total - this.queue.length} of ${this.total})`, false);
    return null;
  }
}
