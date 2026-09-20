import { useEffect } from "react";
import { useWorld } from "../state/StateProvider";

export function VoiceControls() {
  const { voiceSupported, voicePhase, voiceMessage, voiceError, startListening, finishListening, cancelVoice } = useWorld();
  useEffect(() => () => cancelVoice(), [cancelVoice]);
  const recording = voicePhase === "recording";
  const busy = voicePhase === "processing" || voicePhase === "requesting";
  const label = recording ? "Finish voice recording" : busy ? "Cancel voice command" : "Record voice command";

  return (
    <div className="flex items-center gap-3">
      <button type="button" aria-label={label} aria-pressed={recording} disabled={!voiceSupported}
        title={voiceSupported ? `${label}. Audio is sent to Groq to interpret your command.` : "Microphone requires HTTPS or localhost and a supported browser."}
        onClick={recording ? finishListening : busy ? cancelVoice : startListening}
        className={`flex h-9 w-9 shrink-0 items-center justify-center rounded-full border transition-colors disabled:cursor-not-allowed disabled:opacity-40 ${recording
          ? "border-red-500 bg-red-600 text-white" : "border-zinc-700/80 text-zinc-300 hover:bg-zinc-800"}`}>
        {busy ? <svg viewBox="0 0 24 24" className="h-4 w-4 animate-spin" aria-hidden>
          <circle cx="12" cy="12" r="9" fill="none" stroke="currentColor" strokeWidth="2" opacity="0.2" />
          <path d="M12 3a9 9 0 0 1 9 9" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
        </svg> : <svg viewBox="0 0 24 24" className="h-4 w-4" aria-hidden>
          <g fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
            <rect x="9" y="2" width="6" height="13" rx="3" />
            <path d="M5 10v2a7 7 0 0 0 14 0v-2M12 19v3M8 22h8" />
          </g>
        </svg>}
      </button>
      <p role="status" aria-live="polite" className={`min-w-0 text-xs leading-snug ${voiceError ? "text-amber-400" : "text-zinc-400"}`}>
        {recording ? "Listening…" : voicePhase === "processing" ? "Thinking…" : voicePhase === "requesting" ? "Allow microphone access…"
          : voiceMessage || (voiceSupported ? "Tap to speak" : "Microphone unavailable")}
      </p>
    </div>
  );
}
