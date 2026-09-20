import { useEffect, useState } from "react";
import { cameraSnapshotUrl, startCameraFeed, type CameraFrame } from "../state/cameraFeed";

export function CameraFeed({ url }: { url: string }) {
  const [frame, setFrame] = useState<CameraFrame>({});
  const [paused, setPaused] = useState(document.hidden);
  const [retry, setRetry] = useState(0);

  useEffect(() => {
    let stop = () => {};
    const sync = () => {
      stop();
      setFrame({});
      setPaused(document.hidden);
      if (document.hidden) return;
      try {
        stop = startCameraFeed(cameraSnapshotUrl(url, window.location.href), setFrame);
      } catch (error) {
        setFrame({ error: error instanceof Error ? error.message : "Invalid camera URL." });
      }
    };
    document.addEventListener("visibilitychange", sync);
    sync();
    return () => {
      document.removeEventListener("visibilitychange", sync);
      stop();
    };
  }, [url, retry]);

  return (
    <div className="flex h-full min-h-0 flex-col p-4" aria-label="Camera feed">
      <div className="mb-3 flex items-center justify-between gap-3">
        <h2 className="text-sm font-semibold text-zinc-200">Camera preview</h2>
        <span role="status" className="text-xs text-zinc-400">{paused ? "Paused" : frame.error ? "Unavailable" : frame.url ? "Updating · up to 2 fps" : "Connecting…"}</span>
      </div>
      <div className="relative flex min-h-0 flex-1 items-center justify-center overflow-hidden rounded-lg border border-zinc-800 bg-black">
        {frame.url && !paused && <img src={frame.url} alt="Live view from the arena camera" className="h-full w-full object-contain" />}
        {!frame.url && <div className="max-w-md space-y-3 p-6 text-center text-sm text-zinc-400">
          {frame.error ? <>
            <p role="alert" className="text-amber-300">{frame.error}</p>
            <p>Check the snapshot URL and connect to the camera’s network. No further requests are made until you retry or reopen this tab.</p>
            <button type="button" onClick={() => setRetry((value) => value + 1)}
              className="rounded border border-zinc-700 bg-zinc-800 px-3 py-1.5 text-xs text-zinc-200 hover:bg-zinc-700">Retry camera</button>
          </> : <p>{paused ? "Camera requests are paused while this browser tab is hidden." : "Loading the first camera image…"}</p>}
        </div>}
      </div>
    </div>
  );
}
