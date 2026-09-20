"""Second obstacle layer: labelled objects from Sean's detector (vision/detect.py, MobileSAM when its model
files are present), run on our frames with our frozen camera pose.

The chroma segmentation (segment.py) is the fast per-tick layer. This one is slow (seconds per scan, in a
background thread) but it draws proper outlines with SAM, names objects by colour and shape, and its
colour cue is judged against the board's colour in the same way the frontend's obstacles are. Its outlines
are rasterised into the rectified view and OR-ed into the costmap next to the chroma mask.

Frame conventions: Sean's detector works in the tracker's floor frame (origin at floor marker 1, x toward
marker 2, y toward marker 4) with the camera pose of each frame. Our arena frame is defined the same way
from the tags, so the detector is handed our frozen pose (R, t in the arena frame) as if it were the
tracker's, and its floor rectangle is our tag rectangle. Its output is then already in arena cm.
"""
import collections
import multiprocessing as mp
import os
import queue
import sys
import time

import numpy as np
import cv2

VISION = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "vision")
SCAN_WIDTH = 1152          # frames are scanned at this width (half size), as Sean's scan.py does; the pose is scaled to match


def load_detector(use_sam=True):
    """(detect module, Sam instance or None) or (None, None) when vision/ is not in this checkout."""
    if not os.path.isfile(os.path.join(VISION, "detect.py")):
        return None, None
    if VISION not in sys.path:
        sys.path.insert(0, VISION)
    import detect                                     # Sean's module
    sam = None
    if use_sam:
        try:
            from sam import Sam
            sam = Sam()
        except Exception as e:                         # onnxruntime or the model files missing
            print(f"objects: MobileSAM not available ({str(e)[:60]}), using the colour and parallax cues alone")
    return detect, sam


def name_of(obj):
    """A rough name from the object's mean colour and box fill (after vision/scan.py's name_of)."""
    r, g, b = obj["colour"]
    hue, sat, val = (int(v) for v in cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)[0, 0])
    if sat < 50:
        colour = "white" if val > 170 else "black" if val < 70 else "grey"
    else:
        colour = next(name for limit, name in [(8, "red"), (20, "orange"), (34, "yellow"), (85, "green"), (130, "blue"), (165, "purple"), (181, "red")] if hue < limit)
    return f"{colour} {'box' if obj['fill'] >= 0.8 else 'object'}"


def _worker(jobs, results, use_sam):
    """Runs in its own process: loads the detector once, scans every job. Keeps Sean's numpy-heavy code and
    SAM's threads out of the 20 Hz loop's interpreter."""
    try:
        os.nice(10)                                       # the 20 Hz loop keeps priority over the scans
    except OSError:
        pass
    detect, sam = load_detector(use_sam)
    if sam is not None:                                   # fewer decoder threads: SAM shares the cores with the loop
        from concurrent.futures import ThreadPoolExecutor
        sam.workers = 3
        sam.pool = ThreadPoolExecutor(max_workers=3)
    while True:
        job = jobs.get()
        if job is None:
            return
        frames, floor = job
        t0 = time.perf_counter()
        try:
            objects, _, _, _ = detect.detect(frames, floor, sam)
            results.put(("ok", objects, sam is not None, 1000 * (time.perf_counter() - t0)))
        except Exception as e:
            results.put(("error", str(e)[:80], sam is not None, 1000 * (time.perf_counter() - t0)))


class ObjectLayer:
    def __init__(self, geometry, every_s=4.0, frames=6, use_sam=True):
        self.g = geometry
        self.available = os.path.isfile(os.path.join(VISION, "detect.py"))
        self.sam = None                                   # known after the first scan reports back
        self.every_s, self.frames = every_s, frames
        c = geometry.corner_centre
        # our tag rectangle as the detector's floor: markers at (0,0), (W,0), (W,H), (0,H)
        self.floor = (float((c[2][0] + c[3][0]) / 2), float((c[3][1] + c[4][1]) / 2))
        R, t, K = geometry.R, geometry.t, geometry.K
        self.camera = {"f": float(K[0, 0]), "cx": float(K[0, 2]), "cy": float(K[1, 2]),
                       "rvec": cv2.Rodrigues(R)[0].ravel().tolist(), "tvec": t.tolist()}
        self.scale = min(1.0, SCAN_WIDTH / geometry.K[0, 2] / 2)
        self.camera_scaled = {**self.camera, "f": self.camera["f"] * self.scale, "cx": self.camera["cx"] * self.scale, "cy": self.camera["cy"] * self.scale}
        self.window = collections.deque(maxlen=frames)
        self.objects = []                                # latest detections in arena cm
        self.mask = np.zeros((geometry.rect["h"], geometry.rect["w"]), bool)
        self.busy = False
        self.last_started = -1e9
        self.last_finished = None
        self.last_ms = None
        self.runs = 0
        self.error = None
        self._tick = 0
        self.proc = None
        if self.available:
            ctx = mp.get_context("spawn")
            self.jobs, self.results = ctx.Queue(), ctx.Queue()
            self.proc = ctx.Process(target=_worker, args=(self.jobs, self.results, use_sam), daemon=True)
            self.proc.start()

    def offer(self, frame, robot, now):
        """Called every tick with the current frame. Keeps every few frames and starts a scan when due."""
        if not self.available:
            return
        self._tick += 1
        if self._tick % 3 == 0:
            small = cv2.resize(frame, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA) if self.scale < 1 else frame
            state = {"camera": self.camera_scaled, "floor": list(self.floor), "floorMarkers": 4, "zUp": True,
                     "robot": {"px": [v * self.scale for v in robot["px"]]} if robot else None}
            self.window.append((small, state))
        self._collect()
        if not self.busy and len(self.window) >= 2 and now - self.last_started >= self.every_s:
            self.busy, self.last_started = True, now
            self.jobs.put((list(self.window), self.floor))

    def _collect(self):
        """Take a finished scan from the worker, if there is one (never blocks)."""
        try:
            status, payload, sam, ms = self.results.get_nowait()
        except queue.Empty:
            return
        self.sam = sam
        if status == "ok":
            mask = np.zeros(self.mask.shape, np.uint8)
            out = []
            for o in payload:
                pts = np.round(self.g.world_to_rect(np.array(o["outline"], dtype=float))).astype(np.int32)
                cv2.fillPoly(mask, [pts], 255)
                out.append({**o, "label": name_of(o)})
            self.objects, self.mask, self.error = out, mask > 0, None
        else:
            self.error = payload
        self.last_ms, self.last_finished, self.runs, self.busy = ms, time.time(), self.runs + 1, False

    def wait(self, timeout=30.0):
        """Block until the running scan (if any) has reported. For tests."""
        end = time.time() + timeout
        while self.busy and time.time() < end:
            self._collect()
            time.sleep(0.05)

    def close(self):
        if self.proc is not None:
            self.jobs.put(None)
            self.proc.join(timeout=2)

    def status(self):
        if not self.available:
            return "objects: vision/ not in this checkout"
        age = None if self.last_finished is None else time.time() - self.last_finished
        return (f"objects {len(self.objects)}  scans {self.runs}  {'SAM' if self.sam else 'no SAM'}  "
                + ("scanning" if self.busy else f"last {self.last_ms:.0f} ms, {age:.0f} s ago" if age is not None else "waiting")
                + (f"  ERR {self.error}" if self.error else ""))
