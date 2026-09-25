import { useWorld } from "../state/StateProvider";
import { ARMIE, SPIDEY } from "../robot/names";
import type { WorldState } from "../types/world";

type Tone = "idle" | "busy" | "good" | "bad";
const DOT: Record<Tone, string> = { idle: "bg-zinc-500", busy: "bg-sky-400 animate-pulse", good: "bg-emerald-400", bad: "bg-amber-400" };

/** One plain sentence for what is going on right now, from the same state the rest of the UI draws. */
export function story(state: WorldState): { text: string; tone: Tone } {
  const robot = state.robots[0], mission = state.mission, errand = state.play?.errand, arm = state.arm;
  if (!robot) return { text: `Waiting for ${SPIDEY} to show up`, tone: "idle" };
  if (arm && arm.mode !== "idle" && arm.mode !== "returning") {
    const doing: Record<string, string> = { reaching: `reaching for ${SPIDEY}`, grasping: `taking hold of ${SPIDEY}`, lifting: `lifting ${SPIDEY}`,
      carrying: `carrying ${SPIDEY} over`, placing: `setting ${SPIDEY} down`, releasing: `letting go of ${SPIDEY}` };
    return { text: `${ARMIE} is ${doing[arm.mode] ?? "helping"}`, tone: "busy" };
  }
  if (!robot.tracking) return { text: `The camera cannot see ${SPIDEY} right now`, tone: "bad" };
  if (mission?.state === "carrying") return { text: `No way through. ${SPIDEY} is waiting for ${ARMIE} to lift it over`, tone: "busy" };
  if (mission?.via) return { text: `No way through. ${SPIDEY} is walking over to where ${ARMIE} can reach`, tone: "busy" };
  if (mission?.state === "recovering") return { text: `${SPIDEY} got stuck and is wriggling free`, tone: "bad" };
  if (errand?.looking) return { text: `${SPIDEY} is having a good look at the ${errand.label}`, tone: "good" };
  if (mission?.state === "navigating") {
    const path = state.path ?? [];
    let left = 0;
    for (let i = 1; i < path.length; i++) left += Math.hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    const where = errand ? `over to the ${errand.label}` : "to the goal";
    return { text: `${SPIDEY} is walking ${where}${left > 0.02 ? ` · ${Math.round(left * 100)} cm to go` : ""}`, tone: "busy" };
  }
  if (mission?.state === "failed") return { text: `${SPIDEY} gave up: ${mission.detail || "no way there"}`, tone: "bad" };
  if (mission?.state === "done") return { text: `${SPIDEY} arrived. Click the table to send it somewhere else`, tone: "good" };
  if (robot.mode === "moving" || robot.mode === "turning") return { text: `You are driving ${SPIDEY}`, tone: "busy" };
  return { text: `Click anywhere on the table to send ${SPIDEY} there`, tone: "idle" };
}

/** The sentence, as a pill over the top of the map. */
export function StoryBanner({ insetLeft = 0 }: { insetLeft?: number }) {
  const { state } = useWorld();
  if (!state) return null;
  const { text, tone } = story(state);
  return (
    <div className="pointer-events-none absolute right-0 top-3 flex justify-center px-4 transition-[left] duration-300 ease-out" style={{ left: insetLeft }}>
      <p role="status" className="flex max-w-xl items-center gap-2.5 rounded-full bg-zinc-900/90 px-4 py-2 text-sm font-medium text-zinc-100 shadow-lg backdrop-blur">
        <span aria-hidden className={`h-2 w-2 shrink-0 rounded-full ${DOT[tone]}`} />
        {text}
      </p>
    </div>
  );
}
