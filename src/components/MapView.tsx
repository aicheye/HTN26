import { Map2D } from "./Map2D";
import type { MapProps } from "./MapProps";

export type Renderer = "2d" | "3d";

/**
 * Renderer boundary. Drop in a Map3D that accepts MapProps and switch here;
 * nothing else in the app has to change.
 */
export function MapView({ renderer = "2d", ...props }: MapProps & { renderer?: Renderer }) {
  if (renderer === "3d") {
    return (
      <div className="flex h-full w-full items-center justify-center text-sm text-zinc-400">
        3D view not implemented yet — same props as the 2D map.
      </div>
    );
  }
  return <Map2D {...props} />;
}
