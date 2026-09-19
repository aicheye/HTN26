import { Suspense, lazy } from "react";
import { Map2D } from "./Map2D";
import type { MapProps } from "./MapProps";

const Map3D = lazy(() =>
  import("./Map3D").then((m) => ({ default: m.Map3D })),
);

export type Renderer = "2d" | "3d";

/**
 * Renderer boundary. Both maps take exactly MapProps, so switching is one line
 * and nothing else in the app has to change.
 */
export function MapView({ renderer = "2d", ...props }: MapProps & { renderer?: Renderer }) {
  if (renderer === "3d") {
    return (
      <Suspense
        fallback={
          <div className="flex h-full w-full items-center justify-center text-sm text-zinc-400">
            Loading 3D view
          </div>
        }
      >
        <Map3D {...props} />
      </Suspense>
    );
  }
  return <Map2D {...props} />;
}
