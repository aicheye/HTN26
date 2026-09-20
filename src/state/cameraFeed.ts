export const DEFAULT_CAMERA_URL = "http://qnxpi78.local:8003/frame.jpg?w=960";
export type CameraFrame = { url?: string; error?: string };

export function cameraSnapshotUrl(value: string, pageUrl: string): string {
  if (!value.trim()) throw new Error("Enter a camera snapshot URL.");
  const url = new URL(value.trim(), pageUrl);
  if (url.protocol !== "http:" && url.protocol !== "https:") throw new Error("Use an HTTP or HTTPS camera snapshot URL.");
  if (new URL(pageUrl).protocol === "https:" && url.protocol === "http:") {
    throw new Error("This HTTP camera cannot load from an HTTPS page. Open the controller over HTTP on localhost or use an HTTPS camera endpoint.");
  }
  return url.href;
}

export function startCameraFeed(url: string, update: (frame: CameraFrame) => void): () => void {
  const controller = new AbortController();
  let active = true;
  let frameUrl: string | undefined;
  let poll: ReturnType<typeof setTimeout> | undefined;
  let timeout: ReturnType<typeof setTimeout> | undefined;

  const releaseFrame = () => {
    if (frameUrl) URL.revokeObjectURL(frameUrl);
    frameUrl = undefined;
  };
  const load = async () => {
    timeout = setTimeout(() => controller.abort(), 5000);
    try {
      const response = await fetch(url, { signal: controller.signal, cache: "no-store", credentials: "omit", referrerPolicy: "no-referrer" });
      if (!response.ok) throw new Error(`Camera returned HTTP ${response.status}.`);
      if (!response.headers.get("content-type")?.toLowerCase().startsWith("image/")) {
        throw new Error("Use a JPEG or PNG snapshot endpoint, not a video stream or webpage.");
      }
      const image = await response.blob();
      if (!active) return;
      if (controller.signal.aborted) throw new Error("Camera request timed out.");
      if (!image.size) throw new Error("The camera returned an empty image.");
      const nextUrl = URL.createObjectURL(image);
      const previousUrl = frameUrl;
      frameUrl = nextUrl;
      update({ url: nextUrl });
      if (previousUrl) URL.revokeObjectURL(previousUrl);
      poll = setTimeout(load, 500);
    } catch (error) {
      if (!active) return;
      const timedOut = controller.signal.aborted;
      controller.abort();
      releaseFrame();
      update({ error: timedOut ? "Camera request timed out. Check the camera connection and retry."
        : error instanceof Error ? error.message : "Unable to load the camera image." });
    } finally {
      clearTimeout(timeout);
    }
  };
  void load();
  return () => {
    active = false;
    clearTimeout(poll);
    clearTimeout(timeout);
    controller.abort();
    releaseFrame();
  };
}
