import { useEffect, useRef } from "react";
import { useWorld } from "../state/StateProvider";

const BAR_W = 2;
const BAR_GAP = 3;
const SAMPLE_MS = 45;

export function VoiceControls() {
  const { voiceSupported, voicePhase, voiceMessage, voiceError, startListening, finishListening, cancelVoice, getVoiceAnalyser } = useWorld();
  useEffect(() => () => cancelVoice(), [cancelVoice]);
  const recording = voicePhase === "recording";
  const busy = voicePhase === "processing" || voicePhase === "requesting";
  const label = recording ? "Finish voice recording" : busy ? "Cancel voice command" : "Record voice command";

  return (
    <div className="space-y-1.5">
      <button type="button" aria-label={label} aria-pressed={recording} disabled={!voiceSupported}
        title={voiceSupported ? `${label}. Audio is sent to Groq to interpret your command.` : "Microphone requires HTTPS or localhost and a supported browser."}
        onClick={recording ? finishListening : busy ? cancelVoice : startListening}
        className={`flex h-11 w-full items-center gap-3 rounded-md border px-3 text-sm transition-colors disabled:cursor-not-allowed disabled:opacity-40 ${recording
          ? "border-red-500/60 bg-red-500/10 text-red-300"
          : "border-zinc-700/80 bg-zinc-800/60 text-zinc-300 hover:border-zinc-600 hover:bg-zinc-800"}`}>
        {busy ? <Spinner /> : recording ? <StopIcon /> : <MicIcon />}
        <span className="min-w-0 flex-1 text-left">
          {recording ? <Waveform getAnalyser={getVoiceAnalyser} />
            : voicePhase === "processing" ? "Working out what you said…"
            : voicePhase === "requesting" ? "Allow microphone access…"
            : voiceSupported ? "Speak a command" : "Microphone unavailable"}
        </span>
        {recording && <span className="text-xs text-red-300/80">Send</span>}
      </button>
      <p role="status" aria-live="polite" className={`min-h-4 text-xs leading-snug ${voiceError ? "text-amber-400" : "text-zinc-500"}`}>
        {recording ? <span className="sr-only">Recording. Speak now.</span> : voiceMessage}
      </p>
    </div>
  );
}

/** Scrolling bars of the live microphone level, newest on the right. */
function Waveform({ getAnalyser }: { getAnalyser: () => AnalyserNode | null }) {
  const ref = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = ref.current;
    const ctx = canvas?.getContext("2d");
    if (!canvas || !ctx) return;
    const levels: number[] = [];
    let buffer: Float32Array<ArrayBuffer> | null = null;
    let last = 0;
    let frame = 0;

    const draw = (time: number) => {
      frame = requestAnimationFrame(draw);
      const dpr = window.devicePixelRatio || 1;
      const w = canvas.clientWidth, h = canvas.clientHeight;
      if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
        canvas.width = w * dpr;
        canvas.height = h * dpr;
      }
      if (time - last >= SAMPLE_MS) {
        last = time;
        const analyser = getAnalyser();
        let level = 0;
        if (analyser) {
          buffer ??= new Float32Array(analyser.fftSize);
          analyser.getFloatTimeDomainData(buffer);
          let sum = 0;
          for (const v of buffer) sum += v * v;
          level = Math.min(1, Math.sqrt(sum / buffer.length) * 7);
        }
        levels.push(level);
        levels.splice(0, Math.max(0, levels.length - Math.ceil(w / (BAR_W + BAR_GAP)) - 1));
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = "#fca5a5";
      levels.forEach((level, i) => {
        const x = w - (levels.length - i) * (BAR_W + BAR_GAP);
        const barHeight = Math.max(2, level * h);
        ctx.fillRect(x, (h - barHeight) / 2, BAR_W, barHeight);
      });
    };
    frame = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(frame);
  }, [getAnalyser]);

  return <canvas ref={ref} aria-hidden className="block h-6 w-full" />;
}

function MicIcon() {
  return (
    <svg viewBox="0 0 24 24" className="h-4 w-4 shrink-0" aria-hidden>
      <g fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
        <rect x="9" y="2" width="6" height="13" rx="3" />
        <path d="M5 10v2a7 7 0 0 0 14 0v-2M12 19v3M8 22h8" />
      </g>
    </svg>
  );
}

function StopIcon() {
  return (
    <svg viewBox="0 0 24 24" className="h-4 w-4 shrink-0" aria-hidden>
      <rect x="6" y="6" width="12" height="12" rx="2" fill="currentColor" />
    </svg>
  );
}

function Spinner() {
  return (
    <svg viewBox="0 0 24 24" className="h-4 w-4 shrink-0 animate-spin" aria-hidden>
      <circle cx="12" cy="12" r="9" fill="none" stroke="currentColor" strokeWidth="2" opacity="0.2" />
      <path d="M12 3a9 9 0 0 1 9 9" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
    </svg>
  );
}
