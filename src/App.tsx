import { useState } from "react";
import type { WorldState } from "./types/world";
import { ControlPad } from "./components/ControlPad";
import { MockSceneControls } from "./components/MockSceneControls";
import { MapView, type Renderer } from "./components/MapView";
import { CommandLog, Telemetry } from "./components/StatusPanel";
import { useWorld } from "./state/StateProvider";
import { VoiceControls } from "./components/VoiceControls";
import { PlayControls } from "./components/PlayControls";
import { PanelSection } from "./components/PanelSection";
import { TitleScreen } from "./components/TitleScreen";
import { BlurredBackdrop } from "./components/BlurredBackdrop";
import { CameraFeed } from "./components/CameraFeed";
import { DEFAULT_CAMERA_URL } from "./state/cameraFeed";
import { makeMockScenario } from "./data/mockScenarios";

type Section = "controls" | "camera" | "telemetry" | "log" | "raw" | "settings";

export default function App() {
  const {
    state,
    selectedRobotId,
    sourceKind,
    setSourceKind,
    wsUrl,
    setWsUrl,
    status,
    speed,
    setSpeed,
    send,
    cancelVoice,
  } = useWorld();
  const [started, setStarted] = useState(false);
  const [mainView, setMainView] = useState<Renderer>("3d");
  const [enteredFromTop, setEnteredFromTop] = useState(false);  // 3D was reached by tilting out of the 2D map
  const [cameraUrl, setCameraUrl] = useState<string | null>(null);
  const feedUrl = cameraUrl ?? state?.cameraFeedUrl ?? DEFAULT_CAMERA_URL;
  const [showDebug, setShowDebug] = useState(false);
  const [section, setSection] = useState<Section | null>("controls");
  const changeSection = (next: Section | null) => {
    cancelVoice();
    setSection(next);
  };
  const visible = SECTIONS.filter((s) => s.id !== "raw" || showDebug);

  return (
    <div className="relative flex h-full">
      <TitleScreen onStart={() => setStarted(true)} />
      <nav className="relative z-20 flex w-12 shrink-0 flex-col items-center gap-1 border-r border-zinc-800 bg-zinc-950 py-2">
        {visible.map((s) => (
          <button
            key={s.id}
            type="button"
            title={s.label}
            aria-label={s.label}
            aria-pressed={section === s.id}
            onClick={() => changeSection(section === s.id ? null : s.id)}
            className={`flex h-9 w-9 items-center justify-center rounded-lg transition ${
              s.id === "settings" ? "mt-auto" : ""
            } ${
              section === s.id
                ? "bg-zinc-800 text-zinc-100"
                : "text-zinc-500 hover:bg-zinc-800/80 hover:text-zinc-200"
            }`}
          >
            {s.icon}
          </button>
        ))}
      </nav>

      {/* Overlays the map rather than resizing it, so the canvas never has to change size. */}
      <aside
        aria-hidden={!section}
        className={`absolute bottom-0 left-12 top-0 z-10 w-64 border-r border-zinc-800 bg-zinc-900 shadow-[8px_0_24px_-6px_rgba(0,0,0,0.55)] transition-[translate,visibility] duration-300 ease-out ${
          section ? "visible translate-x-0" : "invisible -translate-x-full"
        }`}
      >
        <div className="flex h-full w-64 flex-col">
          <div className="flex items-center justify-between border-b border-zinc-800 px-4 py-2.5">
            <h2 className="text-sm font-medium text-zinc-200">
              {SECTIONS.find((s) => s.id === section)?.label}
            </h2>
            <button
              type="button"
              onClick={() => changeSection(null)}
              aria-label="Collapse panel"
              className="rounded p-1 text-zinc-500 transition hover:bg-zinc-800 hover:text-zinc-200"
            >
              <svg viewBox="0 0 16 16" className="h-3.5 w-3.5" aria-hidden>
                <path
                  d="m4 4 8 8M12 4l-8 8"
                  stroke="currentColor"
                  strokeWidth="1.6"
                  strokeLinecap="round"
                />
              </svg>
            </button>
          </div>

          <div className="min-h-0 flex-1 overflow-auto p-4">
            {section === "controls" && (
              <div className="space-y-6">
                <PanelSection title="Drive"><ControlPad /></PanelSection>
                <PanelSection title="Voice"><VoiceControls /></PanelSection>
                <PlayControls />
                <MockSceneControls />
              </div>
            )}
            {section === "camera" && (
              <div className="space-y-4 text-xs text-zinc-400">
                <p>The camera loads only while this tab is open. Leaving it or hiding the browser tab stops requests immediately.</p>
                <form className="space-y-2" onSubmit={(event) => {
                  event.preventDefault();
                  setCameraUrl(String(new FormData(event.currentTarget).get("cameraUrl") ?? "").trim());
                }}>
                  <label className="block font-medium">
                    Snapshot URL
                    <input key={feedUrl} name="cameraUrl" type="text" inputMode="url" defaultValue={feedUrl}
                      className="mt-1 w-full rounded border border-zinc-700 bg-zinc-800 px-2 py-1.5 font-mono text-xs text-zinc-200" />
                  </label>
                  <div className="flex flex-wrap gap-2">
                    <button type="submit" className="rounded border border-zinc-700 bg-zinc-800 px-2 py-1.5 text-zinc-200 hover:bg-zinc-700">Apply URL</button>
                    <button type="button" onClick={() => setCameraUrl(null)} className="rounded px-2 py-1.5 hover:bg-zinc-800 hover:text-zinc-200">Use default</button>
                  </div>
                </form>
                <p>Uses the scene’s camera URL when available, otherwise the Pi’s documented JPEG endpoint. Connect to the Sesame-Controller WiFi to reach the Pi.</p>
                <p>Snapshots update at up to 2 fps, with only one request at a time to limit camera load. This preview is independent of the map and robot controls.</p>
              </div>
            )}
            {section === "telemetry" && <Telemetry />}
            {section === "log" && <CommandLog />}
            {section === "raw" && (
              <pre className="font-mono text-[10px] leading-tight text-zinc-400">
                {JSON.stringify(state, null, 2)}
              </pre>
            )}
            {section === "settings" && (
              <div className="space-y-3 text-sm">
                <Field label="Data source">
                  <Segmented
                    value={sourceKind}
                    options={[
                      { value: "mock", label: "Mock" },
                      { value: "ws", label: "WebSocket" },
                    ]}
                    onChange={(v) => setSourceKind(v as "mock" | "ws")}
                  />
                </Field>
                {sourceKind === "ws" && (
                  <label className="block text-xs font-medium text-zinc-400">
                    WebSocket URL
                    <input
                      type="text"
                      defaultValue={wsUrl}
                      onBlur={(e) => setWsUrl(e.target.value.trim())}
                      placeholder="ws://localhost:8080/ws"
                      className="mt-1 w-full rounded border border-zinc-700 bg-zinc-800 px-2 py-1 font-mono text-xs text-zinc-200"
                    />
                  </label>
                )}
                <Field label="Main view">
                  <Segmented
                    value={mainView}
                    options={[
                      { value: "2d", label: "2D" },
                      { value: "3d", label: "3D" },
                    ]}
                    onChange={(v) => setMainView(v as Renderer)}
                  />
                </Field>
                <label className="block text-xs font-medium text-zinc-400">
                  <span className="flex justify-between">
                    Drive speed
                    <span className="tabular-nums text-zinc-200">{speed.toFixed(2)}</span>
                  </span>
                  <input
                    type="range"
                    min={0.1}
                    max={1}
                    step={0.05}
                    value={speed}
                    onChange={(e) => setSpeed(Number(e.target.value))}
                    className="mt-1 w-full accent-zinc-100"
                  />
                </label>
                <div className="space-y-2 border-t border-zinc-800 pt-3">
                  <Check
                    label="Raw JSON state"
                    checked={showDebug}
                    onChange={setShowDebug}
                  />
                </div>
                <p className="border-t border-zinc-800 pt-3 text-xs text-zinc-400">
                  Connection: {status}
                </p>
              </div>
            )}
          </div>
        </div>
      </aside>

      <section className="relative min-w-0 flex-1 overflow-hidden bg-zinc-950">
        {!started ? null : section === "camera" ? <div className="h-full pl-64"><CameraFeed url={feedUrl} /></div> : state && hasArena(state) ? (
          <>
            <MapView
              renderer={mainView}
              state={state}
              selectedRobotId={selectedRobotId}
              onPickGoal={(point) => send("goto", { target: point })}
              insetLeft={section ? 256 : 0}  // the open side panel is w-64 and lies over the map
              // One view, as in Google Maps: tilting 3D to straight down shows the 2D map, dragging 2D tilts into 3D.
              onSwapView={() => { setEnteredFromTop(mainView === "2d"); setMainView(mainView === "2d" ? "3d" : "2d"); }}
              enterFromTop={enteredFromTop}
            />
            <p className="pointer-events-none absolute bottom-3 right-3 rounded bg-zinc-900/80 px-2 py-1 text-[11px] text-zinc-300">
              {mainView === "2d" ? "Drag to tilt into 3D · click to set a goal" : "Tilt to straight down for the 2D map"}
            </p>

          </>
        ) : (
          <EmptyStage
            title={status !== "live" ? "Connecting to the bridge" : state ? "Place the camera above the field" : "Waiting for the first frame"}
            detail={status !== "live" ? "Check that the bridge is running and the WebSocket URL in Settings is correct."
              : state ? "The map appears once the camera can see all four corner markers." : undefined}
          />
        )}
      </section>
    </div>
  );
}

const SECTIONS: { id: Section; label: string; icon: React.ReactNode }[] = [
  { id: "controls", label: "Controls", icon: <ControlsIcon /> },
  { id: "camera", label: "Camera", icon: <CameraIcon /> },
  { id: "telemetry", label: "Telemetry", icon: <TelemetryIcon /> },
  { id: "log", label: "Command log", icon: <LogIcon /> },
  { id: "raw", label: "Raw state", icon: <CodeIcon /> },
  { id: "settings", label: "Settings", icon: <SettingsIcon /> },
];

function ControlsIcon() {
  return (
    <svg viewBox="0 0 20 20" className="h-4.5 w-4.5" aria-hidden>
      <g
        fill="none"
        stroke="currentColor"
        strokeWidth="1.4"
        strokeLinecap="round"
        strokeLinejoin="round"
      >
        <rect x="2.5" y="5.5" width="15" height="9" rx="2" />
        <path d="M5.5 8.5h.01M8.5 8.5h.01M11.5 8.5h.01M14.5 8.5h.01M6.5 11.5h7" />
      </g>
    </svg>
  );
}

function CameraIcon() {
  return (
    <svg viewBox="0 0 20 20" className="h-4.5 w-4.5" aria-hidden>
      <g fill="none" stroke="currentColor" strokeWidth="1.4" strokeLinecap="round" strokeLinejoin="round">
        <path d="M3.5 5.5h3l1-2h5l1 2h3a1 1 0 0 1 1 1v9a1 1 0 0 1-1 1h-13a1 1 0 0 1-1-1v-9a1 1 0 0 1 1-1Z" />
        <circle cx="10" cy="10.5" r="3" />
      </g>
    </svg>
  );
}

function TelemetryIcon() {
  return (
    <svg viewBox="0 0 20 20" className="h-4.5 w-4.5" aria-hidden>
      <path
        d="M5 15V9.5M10 15V5M15 15v-3.5"
        fill="none"
        stroke="currentColor"
        strokeWidth="1.8"
        strokeLinecap="round"
      />
    </svg>
  );
}

function LogIcon() {
  return (
    <svg viewBox="0 0 20 20" className="h-4.5 w-4.5" aria-hidden>
      <g
        fill="none"
        stroke="currentColor"
        strokeWidth="1.4"
        strokeLinecap="round"
        strokeLinejoin="round"
      >
        <rect x="2.5" y="4" width="15" height="12" rx="2" />
        <path d="m6 8.5 2.2 1.8L6 12.1M11 12.2h3.2" />
      </g>
    </svg>
  );
}

function SettingsIcon() {
  return (
    <svg viewBox="0 0 24 24" className="h-4.5 w-4.5" aria-hidden>
      <g fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
        <circle cx="12" cy="12" r="3" />
        <path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 1 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 1 1-2.83-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 1 1 2.83-2.83l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 1 1 2.83 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1Z" />
      </g>
    </svg>
  );
}

function CodeIcon() {
  return (
    <svg viewBox="0 0 20 20" className="h-4.5 w-4.5" aria-hidden>
      <path
        d="M8.2 3.5c-1.8 0-1.8 2-1.8 3.2 0 1.1-.4 2.1-1.9 3.3 1.5 1.2 1.9 2.2 1.9 3.3 0 1.2 0 3.2 1.8 3.2M11.8 3.5c1.8 0 1.8 2 1.8 3.2 0 1.1.4 2.1 1.9 3.3-1.5 1.2-1.9 2.2-1.9 3.3 0 1.2 0 3.2-1.8 3.2"
        fill="none"
        stroke="currentColor"
        strokeWidth="1.4"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}

/** The tracker reports a 0 x 0 arena until it has calibrated; drawing that would blow the corner markers up to fill the view. */
function hasArena(state: { arena: { width: number; length: number } }) {
  return state.arena.width > 0 && state.arena.length > 0;
}

/** An empty table with no robot or obstacles: a blurred 3D backdrop that hints at what will appear here. */
const BACKDROP: WorldState = (() => {
  const { robots: _r, obstacles: _o, arm: _a, goal: _g, path: _p, simulation: _s, ...scene } = makeMockScenario("empty");
  return { ...scene, robots: [], obstacles: [] };
})();

function EmptyStage({ title, detail }: { title: string; detail?: string }) {
  return (
    <div className="relative h-full w-full overflow-hidden">
      <BlurredBackdrop state={BACKDROP} />
      <div className="absolute inset-0 flex flex-col items-center justify-center gap-1.5 px-8 text-center">
        <p className="text-base font-medium text-zinc-100">{title}</p>
        {detail && <p className="max-w-xs text-sm leading-relaxed text-zinc-400">{detail}</p>}
      </div>
    </div>
  );
}

function Field({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <div className="space-y-1.5">
      <span className="text-xs font-medium text-zinc-400">{label}</span>
      {children}
    </div>
  );
}

function Segmented({
  value,
  options,
  onChange,
}: {
  value: string;
  options: { value: string; label: string }[];
  onChange: (v: string) => void;
}) {
  return (
    <div className="flex rounded-md border border-zinc-700 bg-zinc-800 p-0.5">
      {options.map((o) => (
        <button
          key={o.value}
          type="button"
          onClick={() => onChange(o.value)}
          className={`flex-1 rounded px-2 py-1 text-xs font-medium transition ${
            value === o.value
              ? "bg-zinc-100 text-zinc-900 shadow-sm"
              : "text-zinc-400 hover:text-zinc-200"
          }`}
        >
          {o.label}
        </button>
      ))}
    </div>
  );
}

function Check({
  label,
  checked,
  onChange,
}: {
  label: string;
  checked: boolean;
  onChange: (v: boolean) => void;
}) {
  return (
    <label className="flex items-center gap-2 text-sm text-zinc-300">
      <input
        type="checkbox"
        checked={checked}
        onChange={(e) => onChange(e.target.checked)}
        className="h-3.5 w-3.5 rounded border-zinc-600 accent-zinc-100"
      />
      {label}
    </label>
  );
}
