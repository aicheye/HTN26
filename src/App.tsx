import { useState } from "react";
import { ControlPad } from "./components/ControlPad";
import { MapView, type Renderer } from "./components/MapView";
import { CommandLog, Telemetry } from "./components/StatusPanel";
import { useWorld } from "./state/StateProvider";

type Section = "controls" | "telemetry" | "log" | "raw" | "settings";

export default function App() {
  const {
    state,
    selectedRobotId,
    sourceKind,
    setSourceKind,
    status,
    speed,
    setSpeed,
    send,
  } = useWorld();
  const [mainView, setMainView] = useState<Renderer>("3d");
  const [showCamera, setShowCamera] = useState(false);
  const [showDebug, setShowDebug] = useState(false);
  const [section, setSection] = useState<Section | null>("controls");
  const pipView: Renderer = mainView === "3d" ? "2d" : "3d";
  const visible = SECTIONS.filter((s) => s.id !== "raw" || showDebug);

  return (
    <div className="flex h-full">
      <nav className="relative z-10 flex w-12 shrink-0 flex-col items-center gap-1 border-r border-zinc-800/60 bg-zinc-950 py-2 shadow-[4px_0_16px_rgba(0,0,0,0.35)]">
        {visible.map((s) => (
          <button
            key={s.id}
            type="button"
            title={s.label}
            aria-label={s.label}
            aria-pressed={section === s.id}
            onClick={() => setSection((cur) => (cur === s.id ? null : s.id))}
            className={`flex h-9 w-9 items-center justify-center rounded-lg transition ${
              s.id === "settings" ? "mt-auto" : ""
            } ${
              section === s.id
                ? "bg-zinc-100 text-zinc-900 shadow-md"
                : "text-zinc-500 hover:bg-zinc-800/80 hover:text-zinc-200"
            }`}
          >
            {s.icon}
          </button>
        ))}
      </nav>

      <aside
        className={`relative z-10 min-w-0 shrink-0 overflow-hidden border-r bg-zinc-900 shadow-[6px_0_24px_rgba(0,0,0,0.4)] transition-[width] duration-200 ${
          section ? "w-64 border-zinc-800/60" : "w-0 border-transparent shadow-none"
        }`}
      >
        <div className="flex h-full w-64 flex-col">
          <div className="flex items-center justify-between border-b border-zinc-800/60 bg-zinc-900/95 px-4 py-2.5 backdrop-blur-sm">
            <h2 className="text-xs font-semibold uppercase tracking-wide text-zinc-400">
              {SECTIONS.find((s) => s.id === section)?.label}
            </h2>
            <button
              type="button"
              onClick={() => setSection(null)}
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
            {section === "controls" && <ControlPad />}
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
                    label="Camera feed layer"
                    checked={showCamera}
                    onChange={setShowCamera}
                  />
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
        {state ? (
          <>
            <MapView
              renderer={mainView}
              state={state}
              selectedRobotId={selectedRobotId}
              showCameraLayer={showCamera}
              onPickGoal={(p) => send("goto", { target: p })}
            />

            <div className="absolute right-3 top-3 h-40 w-56 overflow-hidden rounded-lg bg-zinc-900 shadow-[0_8px_24px_rgba(0,0,0,0.45)]">
              <div className="pointer-events-none h-full w-full">
                <MapView
                  renderer={pipView}
                  state={state}
                  selectedRobotId={selectedRobotId}
                  showCameraLayer={showCamera}
                  compact
                />
              </div>
              <button
                type="button"
                onClick={() => setMainView(pipView)}
                title={`Show ${pipView.toUpperCase()} full size`}
                className="group absolute inset-0 flex items-end justify-between p-1.5"
              >
                <span className="rounded bg-zinc-900/85 px-1.5 py-0.5 text-[10px] font-semibold text-zinc-300">
                  {pipView.toUpperCase()}
                </span>
                <span className="rounded bg-zinc-900/0 px-1.5 py-0.5 text-[10px] font-medium text-transparent transition group-hover:bg-zinc-100/90 group-hover:text-zinc-900">
                  Swap
                </span>
              </button>
            </div>
          </>
        ) : (
          <div className="flex h-full items-center justify-center text-sm text-zinc-500">
            Waiting for first frame
          </div>
        )}
      </section>
    </div>
  );
}

const SECTIONS: { id: Section; label: string; icon: React.ReactNode }[] = [
  { id: "controls", label: "Controls", icon: <ControlsIcon /> },
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
