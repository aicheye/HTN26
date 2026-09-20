import { useCallback, useEffect, useRef, useState } from "react";
import { VoiceExecutor, type VoiceSnapshot } from "./voiceSession";
import { VoiceRecording, recordingSupported } from "./voiceRecording";
import { requestVoice, type Clarification } from "./voiceApi";

export function useVoiceControl(snapshot: () => VoiceSnapshot, wsUrl: string, sourceKind: "mock" | "ws") {
  const current = useRef({ snapshot, wsUrl, sourceKind });
  current.current = { snapshot, wsUrl, sourceKind };
  const [voiceMessage, setVoiceMessage] = useState("");
  const [voiceError, setVoiceError] = useState(false);
  const [executor] = useState(() => new VoiceExecutor(() => current.current.snapshot(), (message, error) => {
    setVoiceMessage(message);
    setVoiceError(error);
  }));
  const recording = useRef<VoiceRecording | null>(null);
  const pending = useRef<AbortController | null>(null);
  // The last unanswered question, so a spoken reply is interpreted together with the request that prompted it.
  const clarification = useRef<(Clarification & { at: number }) | null>(null);
  const [voicePhase, setVoicePhaseState] = useState<"idle" | "requesting" | "recording" | "processing">("idle");
  const phaseRef = useRef<typeof voicePhase>("idle");
  const setVoicePhase = useCallback((phase: typeof voicePhase) => { phaseRef.current = phase; setVoicePhaseState(phase); }, []);
  const voiceSupported = recordingSupported();

  const cancelVoice = useCallback(() => {
    recording.current?.cancel();
    recording.current = null;
    pending.current?.abort();
    pending.current = null;
    executor.cancel();
    setVoicePhase("idle");
  }, [executor, setVoicePhase]);

  const submit = useCallback(async (audio: Blob) => {
    const controller = new AbortController();
    pending.current = controller;
    setVoicePhase("processing");
    const timeout = window.setTimeout(() => controller.abort(), 22000);
    try {
      const context = current.current;
      const asked = clarification.current && Date.now() - clarification.current.at < 60000 ? clarification.current : null;
      const result = await requestVoice(audio, context.snapshot(), context.wsUrl, context.sourceKind, controller.signal,
        asked ? { transcript: asked.transcript, question: asked.question } : undefined);
      if (pending.current !== controller || controller.signal.aborted) return;
      clarification.current = result.clarify && result.transcript ? { transcript: result.transcript, question: result.message, at: Date.now() } : null;
      const message = result.plan.length ? executor.executePlan(result.plan) : result.message;
      setVoiceMessage(message);
      setVoiceError(message.startsWith("Not sent"));
    } catch (error) {
      if (pending.current !== controller) return;
      setVoiceMessage(controller.signal.aborted ? "Voice request timed out. Try again."
        : error instanceof Error ? error.message : "Voice command failed. Try again.");
      setVoiceError(true);
    } finally {
      window.clearTimeout(timeout);
      if (pending.current === controller) { pending.current = null; setVoicePhase("idle"); }
    }
  }, [executor, setVoicePhase]);

  const startListening = useCallback(() => {
    cancelVoice();
    setVoiceMessage("");
    setVoiceError(false);
    if (!recordingSupported()) {
      setVoiceMessage("Use Chrome or Edge on HTTPS or localhost to record a command.");
      setVoiceError(true);
      return;
    }
    recording.current = new VoiceRecording((phase, error) => {
      setVoicePhase(phase);
      if (error) { setVoiceMessage(error); setVoiceError(true); }
    }, (audio) => { void submit(audio); });
    void recording.current.start();
  }, [cancelVoice, submit, setVoicePhase]);

  const finishListening = useCallback(() => recording.current?.finish(), []);
  const getVoiceAnalyser = useCallback(() => recording.current?.analyser ?? null, []);

  useEffect(() => {
    const onHidden = () => { if (document.hidden) cancelVoice(); };
    const timer = window.setInterval(() => {
      const error = executor.check();
      if (error) {
        cancelVoice();
        setVoiceMessage(`Voice action stopped: ${error}`);
        setVoiceError(true);
      }
    }, 100);
    // Focus moves to the browser's own UI while the microphone permission prompt is open, so blur
    // must not cancel a recording that is starting; it only stops robot actions and pending requests.
    const onBlur = () => { if (phaseRef.current !== "recording" && phaseRef.current !== "requesting") cancelVoice(); };
    window.addEventListener("blur", onBlur);
    window.addEventListener("pagehide", cancelVoice);
    document.addEventListener("visibilitychange", onHidden);
    return () => {
      window.clearInterval(timer);
      window.removeEventListener("blur", onBlur);
      window.removeEventListener("pagehide", cancelVoice);
      document.removeEventListener("visibilitychange", onHidden);
      cancelVoice();
    };
  }, [cancelVoice, executor]);

  return { voiceSupported, voicePhase, voiceMessage, voiceError, startListening, finishListening, cancelVoice, getVoiceAnalyser };
}
