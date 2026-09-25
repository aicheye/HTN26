export type RecordingPhase = "idle" | "requesting" | "recording";

export function recordingSupported(): boolean {
  return window.isSecureContext && Boolean(navigator.mediaDevices?.getUserMedia) && typeof MediaRecorder !== "undefined";
}

export class VoiceRecording {
  private generation = 0;
  private recorder: MediaRecorder | null = null;
  private stream: MediaStream | null = null;
  private context: AudioContext | null = null;
  private timer: ReturnType<typeof setTimeout> | undefined;
  private meter: ReturnType<typeof setInterval> | undefined;
  /** Live microphone signal for drawing a waveform; null unless the browser supports AudioContext. */
  analyser: AnalyserNode | null = null;

  constructor(private update: (phase: RecordingPhase, error?: string) => void, private receive: (audio: Blob) => void) {}

  async start() {
    this.cancel();
    const generation = this.generation;
    this.update("requesting");
    this.timer = setTimeout(() => {
      this.cancel();
      this.update("idle", "Microphone permission timed out. Try again.");
    }, 10000);
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: { echoCancellation: true, noiseSuppression: true } });
      if (generation !== this.generation) { stream.getTracks().forEach((track) => track.stop()); return; }
      this.stream = stream;
      clearTimeout(this.timer);
      const mimeType = ["audio/webm;codecs=opus", "audio/webm", "audio/mp4", "audio/ogg;codecs=opus"].find((type) => MediaRecorder.isTypeSupported(type));
      if (!mimeType) throw new Error("unsupported");
      const recorder = new MediaRecorder(stream, { mimeType, audioBitsPerSecond: 64000 });
      this.recorder = recorder;
      const chunks: Blob[] = [];
      let size = 0;
      recorder.ondataavailable = ({ data }) => {
        if (generation !== this.generation || !data.size) return;
        size += data.size;
        if (size > 1024 * 1024) {
          this.cancel();
          this.update("idle", "Recording is too large. Try a shorter command.");
        } else chunks.push(data);
      };
      recorder.onerror = () => {
        if (generation !== this.generation) return;
        this.cancel();
        this.update("idle", "Microphone recording failed. Try again.");
      };
      recorder.onstop = () => {
        if (generation !== this.generation) return;
        const audio = new Blob(chunks, { type: recorder.mimeType });
        this.cancel();
        if (audio.size) this.receive(audio);
        else this.update("idle", "No audio recorded. Please try again.");
      };
      recorder.start(250);
      this.update("recording");
      this.timer = setTimeout(() => this.finish(), 8000);
      this.detectSilence(stream);
    } catch (error) {
      if (generation !== this.generation) return;
      this.cancel();
      this.update("idle", error instanceof DOMException && error.name === "NotAllowedError"
        ? "Allow microphone access to use voice commands." : "Could not open the microphone. Try Chrome or Edge on localhost.");
    }
  }

  finish() {
    if (this.recorder?.state === "recording") {
      clearTimeout(this.timer);
      clearInterval(this.meter);
      this.recorder.stop();
    }
  }

  cancel() {
    this.generation++;
    clearTimeout(this.timer);
    clearInterval(this.meter);
    const recorder = this.recorder;
    this.recorder = null;
    if (recorder) {
      recorder.ondataavailable = recorder.onstop = recorder.onerror = null;
      if (recorder.state !== "inactive") recorder.stop();
    }
    this.stream?.getTracks().forEach((track) => track.stop());
    this.stream = null;
    void this.context?.close().catch(() => {});
    this.context = null;
    this.analyser = null;
    this.update("idle");
  }

  private detectSilence(stream: MediaStream) {
    if (typeof AudioContext === "undefined") return;
    try {
      const context = new AudioContext();
      this.context = context;
      void context.resume().catch(() => {});
      const analyser = context.createAnalyser();
      analyser.fftSize = 512;
      this.analyser = analyser;
      context.createMediaStreamSource(stream).connect(analyser);
      const samples = new Float32Array(analyser.fftSize);
      let speechSamples = 0;
      let lastSpeech = Date.now();
      this.meter = setInterval(() => {
        analyser.getFloatTimeDomainData(samples);
        const rms = Math.sqrt(samples.reduce((sum, value) => sum + value * value, 0) / samples.length);
        if (rms > 0.02) { speechSamples++; lastSpeech = Date.now(); }
        if (speechSamples >= 2 && Date.now() - lastSpeech >= 1000) this.finish();
      }, 100);
    } catch {}
  }
}
