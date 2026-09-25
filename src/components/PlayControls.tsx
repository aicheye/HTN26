import { useState } from "react";
import { useWorld } from "../state/StateProvider";
import { clearTrail, setTrailEnabled, trailEnabled } from "../state/trail";
import { SPIDEY } from "../robot/names";
import { PanelSection } from "./PanelSection";

function Switch({ label, hint, on, onChange }: { label: string; hint: string; on: boolean; onChange: (next: boolean) => void }) {
  return (
    <button type="button" role="switch" aria-checked={on} onClick={() => onChange(!on)} className="group flex w-full items-start gap-3 text-left">
      <span className={`mt-0.5 flex h-5 w-9 shrink-0 items-center rounded-full p-0.5 transition-colors ${on ? "bg-emerald-500" : "bg-zinc-700 group-hover:bg-zinc-600"}`}>
        <span className={`h-4 w-4 rounded-full bg-white shadow transition-transform ${on ? "translate-x-4" : ""}`} />
      </span>
      <span>
        <span className="block text-sm font-medium text-zinc-100">{label}</span>
        <span className="block text-xs leading-snug text-zinc-500">{hint}</span>
      </span>
    </button>
  );
}

/** "now", "12 s", "3 min": short enough to leave the line to the entry itself. The panel redraws with every state. */
function ago(at: number) {
  const seconds = Math.max(0, Math.round((Date.now() - at) / 1000));
  return seconds < 3 ? "now" : seconds < 60 ? `${seconds} s` : `${Math.floor(seconds / 60)} min`;
}

/** What Spidey does by itself (bridge/behaviours.mjs), the trail on the maps, and the diary of the table. */
export function PlayControls() {
  const { state, send, sourceKind, addMockObject } = useWorld();
  const [showTrail, setShowTrail] = useState(trailEnabled());
  const play = state?.play;
  if (!play) return null;
  return (
    <>
      <PanelSection title="Play">
        <Switch label="Curious" hint={`${SPIDEY} walks over to anything new you put on the table.`} on={play.curious}
          onChange={(curious) => send("play", { play: { curious } })} />
        <Switch label="Trail" hint={`Draw where ${SPIDEY} has walked.`} on={showTrail}
          onChange={(next) => { setTrailEnabled(next); setShowTrail(next); if (!next) clearTrail(); }} />
        {sourceKind === "mock" && (
          <button type="button" onClick={addMockObject}
            className="h-9 w-full rounded-md border border-zinc-700/80 text-sm text-zinc-200 transition-colors hover:bg-zinc-800">
            Drop a box on the table
          </button>
        )}
      </PanelSection>
      <PanelSection title="Table diary">
        {play.diary.length === 0 ? (
          <p className="text-xs leading-snug text-zinc-500">Nothing has happened yet. Put something on the table, or send {SPIDEY} somewhere.</p>
        ) : (
          <ul className="max-h-56 space-y-1.5 overflow-y-auto pr-1 text-xs leading-snug">
            {play.diary.slice(0, 14).map((entry, i) => (
              <li key={`${entry.at}-${i}`} className="flex items-start gap-2">
                <span aria-hidden className={`mt-1.5 h-1.5 w-1.5 shrink-0 rounded-full ${entry.kind === "robot" ? "bg-sky-400" : "bg-zinc-500"}`} />
                <span className={entry.kind === "robot" ? "text-zinc-100" : "text-zinc-400"}>{entry.text}</span>
                <span className="ml-auto shrink-0 tabular-nums text-zinc-600">{ago(entry.at)}</span>
              </li>
            ))}
          </ul>
        )}
      </PanelSection>
    </>
  );
}
