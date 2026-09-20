import { useState } from "react";
import { useWorld } from "../state/StateProvider";
import { clearTrail, setTrailEnabled, trailEnabled } from "../state/trail";
import { PanelSection } from "./PanelSection";

/** What the robot does by itself (bridge/behaviours.mjs): switches for each, and its diary of the table. */
export function PlayControls() {
  const { state, send, sourceKind, addMockObject } = useWorld();
  const [showTrail, setShowTrail] = useState(trailEnabled());
  const play = state?.play;
  if (!play) return null;
  const objects = state.obstacles.filter((o) => o.source === "cv").length;
  const toggle = "h-8 rounded-md border px-2 text-xs font-medium transition-colors";
  const on = "border-emerald-500/60 bg-emerald-500/15 text-emerald-200", off = "border-zinc-700/80 text-zinc-400 hover:bg-zinc-800";
  return (
    <PanelSection title="Play">
      <div className="grid grid-cols-2 gap-2">
        <button type="button" title="Faces and poses follow what happens: happy on arrival, a shrug when there is no way, sleepy when idle"
          className={`${toggle} ${play.moods ? on : off}`} onClick={() => send("play", { play: { moods: !play.moods } })}>Moods</button>
        <button type="button" title="Walks over to anything new that appears on the table"
          className={`${toggle} ${play.curious ? on : off}`} onClick={() => send("play", { play: { curious: !play.curious } })}>Curious</button>
        <button type="button" title="Draws where the robot has walked on both maps"
          className={`${toggle} ${showTrail ? on : off}`} onClick={() => { setTrailEnabled(!showTrail); setShowTrail(!showTrail); }}>Trail</button>
        <button type="button" title="Forget where the robot has walked" className={`${toggle} ${off}`} onClick={clearTrail}>Clear trail</button>
      </div>
      <button type="button" disabled={!play.tour && objects === 0} title="Visits every object once, nearest first, then takes a bow"
        className="h-9 w-full rounded-md bg-zinc-100 text-sm font-medium text-zinc-900 transition-colors hover:bg-white disabled:opacity-40"
        onClick={() => send("play", { play: { tour: !play.tour } })}>
        {play.tour ? `Stop the tour (${play.tour.visited} of ${play.tour.total})` : `Tour the table (${objects})`}
      </button>
      {sourceKind === "mock" && (
        <button type="button" className={`${toggle} ${off} w-full`} onClick={addMockObject}>Drop a box on the table</button>
      )}
      {play.errand && <p className="text-xs text-zinc-300">{play.errand.kind === "curious" ? "Going to look at" : "Visiting"} the {play.errand.label}</p>}
      {play.diary.length > 0 && (
        <ul className="max-h-40 space-y-1 overflow-y-auto pr-1 text-xs leading-snug">
          {play.diary.slice(0, 12).map((entry, i) => (
            <li key={`${entry.at}-${i}`} className="flex gap-2">
              <span className="shrink-0 tabular-nums text-zinc-600">{new Date(entry.at).toLocaleTimeString([], { minute: "2-digit", second: "2-digit" })}</span>
              <span className={entry.kind === "robot" ? "text-zinc-200" : "text-zinc-400"}>{entry.text}</span>
            </li>
          ))}
        </ul>
      )}
    </PanelSection>
  );
}
