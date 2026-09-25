import { Suspense, lazy } from "react";
import { ErrorBoundary } from "./ErrorBoundary";
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
  return <div className="relative h-full w-full">
    <MapRenderer renderer={renderer} {...props} />
  </div>;
}

function MapRenderer({ renderer = "2d", ...props }: MapProps & { renderer?: Renderer }) {
  if (renderer === "3d") {
    return (
      <ErrorBoundary
        fallback={(retry) => (
          <div className="flex h-full w-full flex-col items-center justify-center gap-2 text-sm text-zinc-400">
            <p>3D view crashed (WebGL unavailable or lost context).</p>
            <button
              type="button"
              onClick={retry}
              className="rounded bg-zinc-800 px-3 py-1 text-zinc-200 hover:bg-zinc-700"
            >
              Retry
            </button>
            <p className="text-xs text-zinc-500">Or switch to 2D in Settings.</p>
          </div>
        )}
      >
        <Suspense
          fallback={
            <div className="flex h-full w-full items-center justify-center text-sm text-zinc-400">
              Loading 3D view
            </div>
          }
        >
          <Map3D {...props} />
        </Suspense>
      </ErrorBoundary>
    );
  }
  return <Map2D {...props} />;
}
