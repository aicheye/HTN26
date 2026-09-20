"""Run the pipeline over a recording (or the live camera) with the overlay window.

    .venv/bin/python -m nav.run --replay recordings/synth-obstacles [--record out.mp4] [--headless] [--fast]
                                [--goal X Y] [--summary out.json]
    .venv/bin/python -m nav.run --live qnxpi78.local [--unit 3]

Stages 1-4 and the overlay. --objects adds Sean's object detector (vision/detect.py, MobileSAM when its
model files are in vision/models) as a second, labelled obstacle layer scanned every few seconds in a
background thread and OR-ed into the costmap. No robot control yet: the state machine only goes CALIBRATING -> IDLE ->
PLANNING -> NAVIGATING/BLOCKED as labels, and the command shown is always "none".

Live mode polls the Pi tracker's /frame.jpg, which serves about 2 frames a second and pauses tracking for
each one (pi/API.md), so it cannot feed a 20 Hz loop; it exists to see real frames through the overlay.
"""
import argparse
import gc
import json
import os
import sys
import threading
import time
import urllib.request

import numpy as np
import cv2

from .geometry import Geometry, CORNER_IDS, ROBOT_ID
from .segment import Segmenter
from .planner import Planner
from .objects import ObjectLayer
from . import overlay

LOOP_HZ = 20.0
RADIUS_SAMPLES = 5           # robot radius measurements averaged at boot


class ReplaySource:
    def __init__(self, folder, fast=False, loop=False):
        self.folder, self.fast, self.loop = folder, fast, loop
        with open(os.path.join(folder, "states.jsonl")) as f:
            self.records = [json.loads(l) for l in f if l.strip()]
        self.i = 0
        self.t_start = None

    def next(self):
        if self.i >= len(self.records):
            if not self.loop:
                return None
            self.i = 0
        rec = self.records[self.i]; self.i += 1
        frame = cv2.imread(os.path.join(self.folder, rec["file"]))
        t = rec["state"].get("t", self.i * 1000 / 15) / 1000.0
        if not self.fast:                                   # pace playback to the recorded timestamps
            if self.t_start is None:
                self.t_start = time.perf_counter() - t
            wait = self.t_start + t - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
        return frame, t, rec


class LiveSource:
    """Frames from the Pi tracker's HTTP endpoint, fetched on a thread so the loop never waits on the network."""
    def __init__(self, host, unit=3):
        self.url = f"http://{host}:{8000 + unit}/frame.jpg"
        self.frame, self.t, self.seq, self.used = None, 0.0, 0, 0
        threading.Thread(target=self._poll, daemon=True).start()

    def _poll(self):
        while True:
            try:
                data = urllib.request.urlopen(self.url, timeout=3).read()
                img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    self.frame, self.t, self.seq = img, time.perf_counter(), self.seq + 1
            except Exception:
                time.sleep(0.5)

    def next(self):
        while self.frame is None:
            time.sleep(0.05)
        while self.used == self.seq:
            time.sleep(0.005)
        self.used = self.seq
        return self.frame, self.t, None


class Pipeline:
    def __init__(self, goal_xy=None, seed=0, objects=False, use_sam=True, objects_every=4.0):
        self.g = None
        self.want_objects, self.use_sam, self.objects_every = objects, use_sam, objects_every
        self.objects = None
        self.seg = self.planner = None
        self.radius = None
        self._radius_samples = []
        self.valid = None
        self.goal_rc = None
        self.goal_xy = goal_xy
        self.rng = np.random.default_rng(seed)
        self.state = "CALIBRATING"
        self.last_t = None
        self.dt = 1 / 15
        self.blocked_since = None
        self.tick_ms, self.plan_ms, self.det_ms = [], [], []
        self.stage_ms = {k: [] for k in ("gray", "detect", "freeze", "robot", "rectify", "segment", "costmap", "plan")}
        self.tags_per_frame = []
        self.frames = 0
        self.steady_from = None      # first tick index after calibration; loop timing counts from here

    def tick(self, frame, t):
        t0 = time.perf_counter()
        marks = [("start", t0)]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marks.append(("gray", time.perf_counter()))
        if self.g is None:
            self.g = Geometry(frame.shape)
        if self.last_t is not None and t > self.last_t:
            self.dt = 0.8 * self.dt + 0.2 * (t - self.last_t)
        self.last_t = t
        td = time.perf_counter()
        tags = self.g.detect(gray)
        self.det_ms.append(1000 * (time.perf_counter() - td))
        marks.append(("detect", time.perf_counter()))
        self.tags_per_frame.append(sorted(tags))
        was_frozen = self.g.frozen
        self.g.try_freeze(tags, None if was_frozen else gray)
        marks.append(("freeze", time.perf_counter()))
        if self.g.frozen and (not was_frozen or self.valid is None):
            self.valid = self.g.validity_mask(frame.shape)
            self.seg = Segmenter(self.g, self.dt)
        robot = self.g.robot_pose(tags) if self.g.frozen else None
        marks.append(("robot", time.perf_counter()))
        rect = seg_out = plan = None
        lookahead = None
        if self.g.frozen:
            rect = self.g.rectify(frame)
            marks.append(("rectify", time.perf_counter()))
            if self.radius is None:
                if robot is not None and self.g.robot_height is not None:
                    r = self.seg.measure_robot_radius(rect, self.valid, robot)
                    if r is not None:
                        self._radius_samples.append(r)
                    if len(self._radius_samples) >= RADIUS_SAMPLES:
                        self.radius = float(np.median(self._radius_samples))
                        self.planner = Planner(self.g, self.radius)
                        if self.want_objects:
                            self.objects = ObjectLayer(self.g, every_s=self.objects_every, use_sam=self.use_sam)
                            print(self.objects.status())
                        self.state = "IDLE"
                        gc.collect()
                        gc.freeze()          # boot-time objects never get scanned again: no collection pauses in the loop
                        self.steady_from = self.frames + 1
            else:
                seg_out = self.seg.segment(rect, self.valid, robot, self.radius)
                marks.append(("segment", time.perf_counter()))
                occupied = seg_out["persisted"]
                if self.objects is not None:
                    self.objects.offer(frame, robot, t)
                    occupied = occupied | self.objects.mask
                    seg_out["objects"] = self.objects.objects
                self.planner.update(occupied)
                marks.append(("costmap", time.perf_counter()))
                if robot is not None:
                    start_rc = self.planner.index([robot["x"], robot["y"]])
                    if self.goal_rc is None or (self.planner.occupied[self.goal_rc] and self.goal_xy is None):
                        # lock a goal for the run; recompute only when it becomes occupied
                        self.goal_rc = self.planner.index(self.goal_xy) if self.goal_xy else self.planner.pick_goal(start_rc, self.rng)
                    plan = self.planner.plan([robot["x"], robot["y"]], self.goal_rc)
                    marks.append(("plan", time.perf_counter()))
                    self.plan_ms.append(plan["ms"])
                    if plan["blocked"]:
                        self.blocked_since = self.blocked_since or t
                        self.state = "BLOCKED" if t - self.blocked_since > 0.5 else "PLANNING"
                    else:
                        self.blocked_since = None
                        self.state = "NAVIGATING" if len(plan["path"]) > 1 else "ARRIVED"
                        if len(plan["path_cm"]) > 1:
                            lookahead = self.lookahead(plan["path_cm"], 2 * 2 * self.radius)
                else:
                    self.state = "HOLD"
        self.frames += 1
        self.tick_ms.append(1000 * (time.perf_counter() - t0))
        for (name, tm), (_, prev) in zip(marks[1:], marks):
            self.stage_ms[name].append(1000 * (tm - prev))
        goal_xy = None if self.goal_rc is None or self.planner is None else self.planner.cell_to_world([self.goal_rc])[0]
        return {"tags": tags, "robot": robot, "rect": rect, "seg": seg_out, "plan": plan, "goal_xy": goal_xy, "lookahead": lookahead}

    @staticmethod
    def lookahead(path_cm, dist_cm):
        d = np.cumsum(np.r_[0, np.linalg.norm(np.diff(path_cm, axis=0), axis=1)])
        return path_cm[min(len(path_cm) - 1, int(np.searchsorted(d, dist_cm)))]

    def info(self, tags, hz):
        s = self.g.summary() if self.g else {}
        free = None if self.planner is None or self.planner.free is None else 100 * self.planner.free[self.planner.inside].mean()
        return [f"state   {self.state}", "command none (no robot control yet)", f"loop    {hz:5.1f} Hz  tick {np.mean(self.tick_ms[-20:]) if self.tick_ms else 0:5.1f} ms",
                f"robot.z std {s.get('z_std_cm', float('nan')):.3f} cm  (height {s.get('robot_height_cm') or 0:.1f})",
                f"reproj  {s.get('reproj_px') or 0:.2f} px  solves {s.get('solves', 0)}",
                f"tags    {sorted(tags)}  unexpected {s.get('unexpected_ids', 0)}",
                f"board   {tuple(round(v, 1) for v in s['board_cm']) if s.get('board_cm') else '-'} cm  cam {s.get('camera_height_cm') or 0:.0f} cm",
                f"robot r {self.radius or 0:.1f} cm   free {free if free is not None else 0:.0f} %",
                f"plan    {np.mean(self.plan_ms[-20:]) if self.plan_ms else 0:.1f} ms"] + ([self.objects.status()] if self.objects else [])

    def summary(self):
        g = self.g.summary() if self.g else {}
        tick = np.array(self.tick_ms) if self.tick_ms else np.zeros(1)
        return {**g, "frames": self.frames, "robot_radius_cm": self.radius,
                "tick_ms_median": float(np.median(tick)), "tick_ms_p95": float(np.percentile(tick, 95)),
                "detect_ms_median": float(np.median(self.det_ms)) if self.det_ms else None,
                "plan_ms_median": float(np.median(self.plan_ms)) if self.plan_ms else None,
                "plan_ms_max": float(np.max(self.plan_ms)) if self.plan_ms else None,
                "frames_all_tags": int(sum(1 for t in self.tags_per_frame if all(i in t for i in (*CORNER_IDS, ROBOT_ID)))),
                "state": self.state,
                "objects": None if self.objects is None else {"count": len(self.objects.objects), "scans": self.objects.runs, "sam": self.objects.sam is not None,
                                                                "last_ms": self.objects.last_ms, "labels": [o["label"] for o in self.objects.objects],
                                                                "positions": [(o["x"], o["y"]) for o in self.objects.objects]}}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--replay", help="folder of frame-*.jpg + states.jsonl")
    ap.add_argument("--live", help="Pi tracker host, e.g. qnxpi78.local")
    ap.add_argument("--unit", type=int, default=3)
    ap.add_argument("--record", help="write the overlay to this mp4")
    ap.add_argument("--headless", action="store_true", help="no window")
    ap.add_argument("--fast", action="store_true", help="replay as fast as possible instead of at the recorded rate")
    ap.add_argument("--loop", action="store_true", help="loop the replay")
    ap.add_argument("--goal", type=float, nargs=2, metavar=("X", "Y"), help="goal in arena cm (default: picked automatically)")
    ap.add_argument("--summary", help="write the run summary as JSON here")
    ap.add_argument("--overlay-every", type=int, default=2, help="compose the overlay every N ticks (it costs ~15 ms)")
    ap.add_argument("--objects", action="store_true", help="also run Sean's object detector (vision/) as a second obstacle layer")
    ap.add_argument("--no-sam", action="store_true", help="object detector without MobileSAM (colour and parallax cues only)")
    ap.add_argument("--objects-every", type=float, default=4.0, help="seconds between object scans")
    args = ap.parse_args(argv)
    if not args.replay and not args.live:
        ap.error("--replay or --live is required")
    source = ReplaySource(args.replay, args.fast, args.loop) if args.replay else LiveSource(args.live, args.unit)
    pipe = Pipeline(goal_xy=args.goal, objects=args.objects, use_sam=not args.no_sam, objects_every=args.objects_every)
    writer = None
    last_wall, hz = time.perf_counter(), 0.0
    try:
        while True:
            item = source.next()
            if item is None:
                break
            frame, t, _ = item
            out = pipe.tick(frame, t)
            now = time.perf_counter(); hz = 0.8 * hz + 0.2 / max(now - last_wall, 1e-6); last_wall = now
            if pipe.frames % max(1, args.overlay_every):
                continue
            img = overlay.compose(frame, out["tags"], pipe.g, out["rect"], out["seg"], pipe.planner, out["plan"], out["robot"],
                                  out["goal_xy"], out["lookahead"], pipe.info(out["tags"], hz))
            if args.record:
                if writer is None:
                    writer = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*"mp4v"), 15 / max(1, args.overlay_every), (img.shape[1], img.shape[0]))
                writer.write(img)
            if not args.headless:
                cv2.imshow("nav", img)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        if writer is not None:
            writer.release()
        if not args.headless:
            cv2.destroyAllWindows()
    if pipe.objects is not None:
        pipe.objects.wait(5.0)
        pipe.objects.close()
    s = pipe.summary()
    print(json.dumps(s, indent=1, default=float))
    if args.summary:
        with open(args.summary, "w") as f:
            json.dump(s, f, indent=1, default=float)
    return 0


if __name__ == "__main__":
    sys.exit(main())
