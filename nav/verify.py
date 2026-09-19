"""Run the pipeline over recorded clips and check the thresholds from the build brief.

    .venv/bin/python -m nav.verify recordings/synth-empty recordings/synth-obstacles ... [--board-cm W H]
                                   [--obstacles "x,y;x,y"] [--lights-at FRAME] [--hand]

Each clip is run through nav.run.Pipeline exactly as the live loop would. Truth comes from a "truth" object
on each states.jsonl line when the clip is synthetic (nav/synth.py); for real clips give the tag-centre
rectangle with --board-cm (tape measure, centre to centre) and known obstacle centroids in arena cm with
--obstacles. Exit code 1 if any check fails. Checks that need a stage not built yet report SKIP.
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import cv2

from .geometry import CORNER_IDS, ROBOT_ID, TAG_CM
from .run import Pipeline

REQUIRED = (*CORNER_IDS, ROBOT_ID)


def run_clip(folder, goal=None):
    with open(os.path.join(folder, "states.jsonl")) as f:
        records = [json.loads(l) for l in f if l.strip()]
    pipe = Pipeline(goal_xy=goal)
    per_frame = []
    for rec in records:
        frame = cv2.imread(os.path.join(folder, rec["file"]))
        t = rec["state"].get("t", 0) / 1000.0
        out = pipe.tick(frame, t)
        per_frame.append({"tags": sorted(out["tags"]), "robot": out["robot"], "seg": None if out["seg"] is None else
                          {"persisted_frac": float(out["seg"]["persisted"][pipe.seg.arena].mean()),
                           "components": [{k: c[k] for k in ("x", "y", "area_cm2")} for c in out["seg"]["components"]]},
                          "plan_ms": out["plan"]["ms"] if out["plan"] else None, "blocked": out["plan"]["blocked"] if out["plan"] else None,
                          "truth": rec.get("truth")})
    return pipe, per_frame


def check(name, ok, detail, results, skip=False):
    status = "SKIP" if skip else ("PASS" if ok else "FAIL")
    results.append((name, status, detail))
    print(f"  {status:4s} {name:52s} {detail}")


def verify(folder, args):
    print(f"\n== {folder}")
    pipe, frames = run_clip(folder, args.goal)
    s = pipe.summary()
    results = []
    truth = next((f["truth"] for f in frames if f["truth"]), None)
    origin = np.array(truth["corner_centres"]["1"]) if truth else np.zeros(2)

    n = len(frames)
    all_tags = sum(1 for f in frames if all(i in f["tags"] for i in REQUIRED))
    check("all 4 corner tags + robot detected >= 95% of frames", all_tags >= 0.95 * n, f"{all_tags}/{n}", results)
    check("reprojection error < 1.5 px", s["frozen"] and s["reproj_px"] < 1.5, f"{s['reproj_px']:.3f} px" if s["frozen"] else "not calibrated", results)
    check("robot.z standard deviation < 0.3 cm", s["frozen"] and s["z_std_cm"] < 0.3, f"{s['z_std_cm']:.3f} cm over {len(pipe.g.z_log)} frames" if s["frozen"] else "-", results)

    board_ref = args.board_cm or (truth and [truth["board"][0] - 2 * truth["corner_centres"]["1"][0], truth["board"][1] - 2 * truth["corner_centres"]["1"][1]])
    if board_ref and s["frozen"]:
        err = max(abs(s["board_cm"][0] - board_ref[0]), abs(s["board_cm"][1] - board_ref[1]))
        check("derived board size vs tape measure within 1 cm", err < 1.0, f"derived {s['board_cm'][0]:.2f} x {s['board_cm'][1]:.2f}, reference {board_ref[0]:.1f} x {board_ref[1]:.1f}", results)
    else:
        check("derived board size vs tape measure within 1 cm", False, "no reference: pass --board-cm W H", results, skip=True)
    check("unexpected tag ids detected == 0", s["unexpected_ids"] == 0, str(s["unexpected_ids"]), results)

    segs = [f for f in frames if f["seg"] is not None]
    obstacles = args.obstacles if args.obstacles is not None else (truth and [(o["x"] - origin[0], o["y"] - origin[1]) for o in truth["obstacles"]])
    def inside(xy):   # arena cm, inside the tag-centre rectangle
        return truth and 0 <= xy[0] - origin[0] <= truth["board"][0] - 2 * origin[0] and 0 <= xy[1] - origin[1] <= truth["board"][1] - 2 * origin[1]
    hand_frames = [f for f in frames if f["truth"] and f["truth"].get("hand") and inside(f["truth"]["hand"])]
    any_hand = any(f["truth"] and f["truth"].get("hand") for f in frames)
    if segs and obstacles is not None and len(obstacles) == 0 and not any_hand:
        worst = max(f["seg"]["persisted_frac"] for f in segs)
        check("empty arena false-positive cells < 0.5% of arena", worst < 0.005, f"max {100 * worst:.3f}%", results)
    else:
        check("empty arena false-positive cells < 0.5% of arena", False, "not an empty-arena clip", results, skip=True)
    if segs and obstacles:
        last = segs[-1]["seg"]["components"]
        errs = [min(np.hypot(c["x"] - x, c["y"] - y) for x, y in obstacles) for c in last]
        missing = [o for o in obstacles if not any(np.hypot(c["x"] - o[0], c["y"] - o[1]) < 2.0 for c in last)]
        ok = bool(errs) and max(errs) < 2.0 and not missing and len(last) == len(obstacles)
        check("known obstacle centroid error < 2 cm", ok, f"{len(last)} found / {len(obstacles)} known, worst {max(errs) if errs else float('nan'):.2f} cm, missing {len(missing)}", results)
    else:
        check("known obstacle centroid error < 2 cm", False, "no known obstacles: pass --obstacles", results, skip=True)
    lights_at = args.lights_at
    if lights_at is None and truth:
        lights = [f["truth"]["light"] for f in frames if f["truth"]]
        lights_at = next((i for i in range(1, len(lights)) if lights[i] != lights[i - 1]), None)
    if lights_at is not None and segs:
        before = [f for f in frames[:lights_at] if f["seg"]]
        after = [f for f in frames[lights_at + pipe.seg.n_persist + 1:] if f["seg"]]
        if before and after:
            base = np.median([f["seg"]["persisted_frac"] for f in before[-5:]])
            worst = max(f["seg"]["persisted_frac"] for f in after) - base
            check("lights changed mid-clip -> new false positives < 1% of arena", worst < 0.01, f"+{100 * worst:.3f}% of arena after frame {lights_at}", results)
    else:
        check("lights changed mid-clip -> new false positives < 1% of arena", False, "no lighting change in this clip", results, skip=True)
    if segs:
        flagged = sum(1 for f in segs if f["robot"] and any(np.hypot(c["x"] - f["robot"]["x"], c["y"] - f["robot"]["y"]) < (pipe.radius or 8) for c in f["seg"]["components"]))
        check("robot never segmented as an obstacle", flagged == 0, f"flagged in {flagged} frames", results)
    if hand_frames and segs:
        seen = sum(1 for f in hand_frames if f["seg"] and any(np.hypot(c["x"] - (f["truth"]["hand"][0] - origin[0]), c["y"] - (f["truth"]["hand"][1] - origin[1])) < 12 for c in f["seg"]["components"]))
        lag = pipe.seg.n_persist
        check("hand entering is mapped (after the persistence delay)", seen >= len(hand_frames) - 2 * lag, f"seen in {seen}/{len(hand_frames)} frames, persistence {lag} frames", results)
    plans = [f["plan_ms"] for f in frames if f["plan_ms"] is not None][1:]   # the first plan follows the planner's boot
    if plans:
        check("full replan time < 15 ms", max(plans) < 15, f"median {np.median(plans):.1f} ms, max {max(plans):.1f} ms ({pipe.planner.rows}x{pipe.planner.cols} grid)", results)
    tick = np.array(pipe.tick_ms[pipe.steady_from or 0:] or pipe.tick_ms)    # after calibration: the control loop's regime
    check("loop rate >= 20 Hz, p95 jitter < 20 ms", np.median(tick) <= 50 and np.percentile(tick, 95) - np.median(tick) < 20,
          f"tick median {np.median(tick):.1f} ms, p95 {np.percentile(tick, 95):.1f} ms over {len(tick)} steady ticks (pipeline only, frame decode excluded)", results)
    check("heading noise at p95 -> command chatter == 0", False, "policy stage not built yet", results, skip=True)
    if truth and s["frozen"]:
        pos = [np.hypot(f["robot"]["x"] - (f["truth"]["robot"]["x"] - origin[0]), f["robot"]["y"] - (f["truth"]["robot"]["y"] - origin[1]))
               for f in frames if f["robot"] and f["truth"] and f["truth"]["robot"]]
        head = [abs((f["robot"]["heading"] - f["truth"]["robot"]["heading"] + 180) % 360 - 180) for f in frames if f["robot"] and f["truth"] and f["truth"]["robot"]]
        if pos:
            check("(synthetic) robot pose vs truth: position < 2 cm, heading < 2 deg", max(pos) < 2 and max(head) < 2,
                  f"position median {np.median(pos):.2f} max {max(pos):.2f} cm, heading median {np.median(head):.2f} max {max(head):.2f} deg", results)
    return results


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("clips", nargs="+")
    ap.add_argument("--board-cm", type=float, nargs=2, metavar=("W", "H"), help="tag-centre rectangle, tape measured")
    ap.add_argument("--obstacles", type=lambda s: [tuple(map(float, p.split(","))) for p in s.split(";") if p], help='"x,y;x,y" in arena cm')
    ap.add_argument("--lights-at", type=int, help="frame index where the lighting changed")
    ap.add_argument("--goal", type=float, nargs=2)
    args = ap.parse_args(argv)
    failed = 0
    for clip in args.clips:
        results = verify(clip, args)
        failed += sum(1 for _, st, _ in results if st == "FAIL")
    print(f"\nRESULT: {'PASS' if failed == 0 else f'FAIL ({failed} checks)'}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
