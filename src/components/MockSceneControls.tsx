import { MOCK_SCENARIOS, type MockScenarioId } from "../data/mockScenarios";
import { useWorld } from "../state/StateProvider";
import { PanelSection } from "./PanelSection";

const DOT = { blocked: "bg-amber-400", complete: "bg-emerald-400", running: "bg-zinc-300", ready: "bg-zinc-600" } as const;

export function MockSceneControls() {
  const { state, sourceKind, resetMockScenario, runMockTest } = useWorld();
  if (sourceKind !== "mock" || !state?.simulation) return null;
  const sim = state.simulation;
  return <PanelSection title="Simulation">
    <select aria-label="Mock obstacle scenario" value={sim.scenario}
      onChange={(e) => resetMockScenario(e.target.value as MockScenarioId)}
      className="h-9 w-full rounded-md border border-zinc-700/80 bg-transparent px-2 text-sm text-zinc-200">
      {MOCK_SCENARIOS.map((s) => <option key={s.id} value={s.id}>{s.label}</option>)}
    </select>
    <div className="grid grid-cols-[1fr_auto] gap-2">
      <button type="button" onClick={runMockTest} className="h-9 rounded-md bg-zinc-100 text-sm font-medium text-zinc-900 transition-colors hover:bg-white">
        {sim.status === "running" ? "Restart test" : "Run test"}
      </button>
      <button type="button" onClick={() => resetMockScenario(sim.scenario as MockScenarioId)} className="h-9 rounded-md border border-zinc-700/80 px-3 text-sm text-zinc-300 transition-colors hover:bg-zinc-800">Reset</button>
    </div>
    <div role="status" className="flex items-start gap-2 text-xs leading-snug text-zinc-400">
      <span aria-hidden className={`mt-1 h-1.5 w-1.5 shrink-0 rounded-full ${DOT[sim.status]}`} />
      <p><span className="capitalize text-zinc-200">{sim.status}</span> · {sim.message}</p>
    </div>
  </PanelSection>;
}
