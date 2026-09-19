import { MOCK_SCENARIOS, type MockScenarioId } from "../data/mockScenarios";
import { useWorld } from "../state/StateProvider";

export function MockSceneControls() {
  const { state, sourceKind, resetMockScenario, runMockTest } = useWorld();
  if (sourceKind !== "mock" || !state?.simulation) return null;
  const sim = state.simulation;
  return <section className="mt-6 space-y-3 border-t border-zinc-700 pt-4 text-xs">
    <h3 className="font-semibold uppercase tracking-wide text-zinc-300">Simulation tests</h3>
    <label className="block text-zinc-400">
      Mock detected obstacles
      <select aria-label="Mock obstacle scenario" value={sim.scenario}
        onChange={(e) => resetMockScenario(e.target.value as MockScenarioId)}
        className="mt-1 w-full rounded border border-zinc-700 bg-zinc-800 px-2 py-2 text-zinc-100">
        {MOCK_SCENARIOS.map((s) => <option key={s.id} value={s.id}>{s.label}</option>)}
      </select>
    </label>
    <div className="flex gap-2">
      <button type="button" onClick={runMockTest} className="flex-1 rounded bg-blue-800 px-2 py-2 font-semibold text-white hover:bg-blue-700">
        {sim.status === "running" ? "Restart test" : "Run test"}
      </button>
      <button type="button" onClick={() => resetMockScenario(sim.scenario as MockScenarioId)} className="rounded border border-zinc-600 px-3 py-2 text-zinc-200 hover:bg-zinc-800">Reset</button>
    </div>
    <div role="status" className={`rounded border p-2 leading-relaxed ${sim.status === "blocked" ? "border-amber-700 text-amber-300" : sim.status === "complete" ? "border-emerald-700 text-emerald-300" : "border-zinc-700 text-zinc-300"}`}>
      <span className="font-semibold uppercase">{sim.status}</span>
      <p>{sim.message}</p>
    </div>
    <p className="leading-relaxed text-zinc-500">Synthetic detections, frontend only. The test button never sends hardware commands. Kinematic carry test, not a force or grip simulation.</p>
  </section>;
}
