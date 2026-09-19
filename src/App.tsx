import { useState } from "react";
import { ControlPad } from "./components/ControlPad";
import { Disclosure, Menu } from "./components/Disclosure";
import { MapView, type Renderer } from "./components/MapView";
import { CommandLog, Telemetry } from "./components/StatusPanel";
import { useWorld } from "./state/StateProvider";

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
  const [renderer, setRenderer] = useState<Renderer>("2d");
  const [showCamera, setShowCamera] = useState(false);
  const [showDebug, setShowDebug] = useState(false);
  const [panelOpen, setPanelOpen] = useState(true);

  return (
    <div className="flex h-full flex-col">
      <header className="relative z-20 flex items-center gap-3 border-b border-zinc-200 bg-white px-5 py-3">
        <h1 className="text-[15px] font-semibold tracking-tight text-zinc-900">
          Sesame Controller
        </h1>

        <div className="ml-auto flex items-center gap-2">
          <Menu label="Options">
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
              <Field label="Renderer">
                <Segmented
                  value={renderer}
                  options={[
                    { value: "2d", label: "2D" },
                    { value: "3d", label: "3D" },
                  ]}
                  onChange={(v) => setRenderer(v as Renderer)}
                />
              </Field>
              <label className="block text-xs font-medium text-zinc-500">
                <span className="flex justify-between">
                  Drive speed
                  <span className="tabular-nums text-zinc-700">{speed.toFixed(2)}</span>
                </span>
                <input
                  type="range"
                  min={0.1}
                  max={1}
                  step={0.05}
                  value={speed}
                  onChange={(e) => setSpeed(Number(e.target.value))}
                  className="mt-1 w-full accent-zinc-900"
                />
              </label>
              <div className="space-y-2 border-t border-zinc-100 pt-3">
                <Check
                  label="Camera feed layer"
                  checked={showCamera}
                  onChange={setShowCamera}
                />
                <Check label="Raw JSON state" checked={showDebug} onChange={setShowDebug} />
              </div>
              <p className="border-t border-zinc-100 pt-3 text-xs text-zinc-500">
                Connection: {status}
              </p>
            </div>
          </Menu>

          <button
            type="button"
            onClick={() => setPanelOpen((v) => !v)}
            aria-label={panelOpen ? "Hide controls" : "Show controls"}
            aria-pressed={panelOpen}
            className="rounded-md border border-zinc-300 bg-white p-1.5 text-zinc-600 hover:bg-zinc-50"
          >
            <PanelIcon open={panelOpen} />
          </button>
        </div>
      </header>

      <main className="flex min-h-0 flex-1 gap-4 p-4">
        <section className="min-w-0 flex-1 overflow-hidden rounded-xl border border-zinc-200 bg-white">
          {state ? (
            <MapView
              renderer={renderer}
              state={state}
              selectedRobotId={selectedRobotId}
              showCameraLayer={showCamera}
              onPickGoal={(p) => send("goto", { target: p })}
            />
          ) : (
            <div className="flex h-full items-center justify-center text-sm text-zinc-400">
              Waiting for first frame
            </div>
          )}
        </section>

        <aside
          className={`min-w-0 shrink-0 overflow-hidden rounded-xl border bg-white transition-[width] duration-200 ${
            panelOpen ? "w-75 border-zinc-200" : "w-0 border-transparent"
          }`}
        >
          <div className="flex h-full w-75 flex-col px-4 py-4">
            <div className="flex min-h-0 flex-1 items-center justify-center">
              <ControlPad />
            </div>

            <div className="max-h-3/5 shrink-0 overflow-auto border-t border-zinc-200">
              <Disclosure title="Telemetry">
                <Telemetry />
              </Disclosure>
              <Disclosure title="Command log">
                <CommandLog />
              </Disclosure>
              {showDebug && (
                <Disclosure title="Raw state">
                  <pre className="max-h-72 overflow-auto rounded-md bg-zinc-50 p-2 font-mono text-[10px] leading-tight text-zinc-600">
                    {JSON.stringify(state, null, 2)}
                  </pre>
                </Disclosure>
              )}
            </div>
          </div>
        </aside>
      </main>
    </div>
  );
}

function Field({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <div className="space-y-1.5">
      <span className="text-xs font-medium text-zinc-500">{label}</span>
      {children}
    </div>
  );
}

function PanelIcon({ open }: { open: boolean }) {
  return (
    <svg viewBox="0 0 20 20" className="h-4 w-4" aria-hidden>
      <rect
        x="2.5"
        y="3.5"
        width="15"
        height="13"
        rx="2.5"
        fill="none"
        stroke="currentColor"
        strokeWidth="1.6"
      />
      <line x1="12.5" y1="3.5" x2="12.5" y2="16.5" stroke="currentColor" strokeWidth="1.6" />
      {open && <rect x="12.5" y="3.5" width="5" height="13" fill="currentColor" />}
    </svg>
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
    <div className="flex rounded-md border border-zinc-200 bg-zinc-100 p-0.5">
      {options.map((o) => (
        <button
          key={o.value}
          type="button"
          onClick={() => onChange(o.value)}
          className={`flex-1 rounded px-2 py-1 text-xs font-medium transition ${
            value === o.value
              ? "bg-white text-zinc-900 shadow-sm"
              : "text-zinc-500 hover:text-zinc-700"
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
    <label className="flex items-center gap-2 text-sm text-zinc-600">
      <input
        type="checkbox"
        checked={checked}
        onChange={(e) => onChange(e.target.checked)}
        className="h-3.5 w-3.5 rounded border-zinc-300 accent-zinc-900"
      />
      {label}
    </label>
  );
}
