import type { WorldState } from "../types/world";
import { MapView } from "./MapView";

/** A blurred, non-interactive 3D scene used behind full-stage messages. */
export function BlurredBackdrop({ state, orbit = false, blur = "blur-md", tint = "from-zinc-950/40 via-zinc-950/55 to-zinc-950/80" }: {
  state: WorldState;
  orbit?: boolean;
  blur?: string;
  tint?: string;
}) {
  return (
    <div className="absolute inset-0 bg-zinc-950">
      <div aria-hidden className={`pointer-events-none absolute -inset-8 opacity-80 ${blur}`}>
        <MapView renderer="3d" state={state} selectedRobotId={null} compact orbit={orbit} />
      </div>
      <div className={`absolute inset-0 bg-gradient-to-b ${tint}`} />
    </div>
  );
}
