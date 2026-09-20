"""The four-panel figure: rectified arena, obstacles, C-space, path. Centimetre axes, one frame of a clip.

    .venv/bin/python -m nav.figure recordings/rec-001 [--frame N] [--goal X Y] [--objects] [--out figure.png]

Default frame: the last one where the robot was seen. Same conventions as the team's earlier figures: tag
centres as black squares with a dashed boundary, the robot as a green triangle, the goal as a star,
obstacles in yellow, C-space with occupied cells dark red and the inflation band in tan, the path in cyan
with magenta waypoints.
"""
import argparse
import json
import os
import sys

import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from .geometry import CORNER_IDS, TAG_CM
from .run import Pipeline

YELLOW, OCC, INFL, OUT = "#f2e21e", "#6b0f1a", "#d9c3b3", "#b07a5f"


def run_until(folder, frame_index=None, goal=None, objects=False, robot_radius=None):
    with open(os.path.join(folder, "states.jsonl")) as f:
        records = [json.loads(l) for l in f if l.strip()]
    pipe = Pipeline(goal_xy=goal, objects=objects, use_sam=True, objects_every=2.0, robot_radius=robot_radius)
    outs = []
    last = frame_index if frame_index is not None else len(records) - 1
    for k, rec in enumerate(records[: last + 1]):
        frame = cv2.imread(os.path.join(folder, rec["file"]))
        outs.append((frame, pipe.tick(frame, rec["state"].get("t", 0) / 1000.0, rec.get("state"))))
    if frame_index is None:                      # the last frame where the robot was seen and a plan exists
        for k in range(len(outs) - 1, -1, -1):
            if outs[k][1]["robot"] is not None and outs[k][1]["plan"] is not None:
                if objects and pipe.objects is not None:
                    pipe.objects.wait()
                return pipe, k, outs[k]
    if objects and pipe.objects is not None:
        pipe.objects.wait()
    return pipe, last, outs[last]


def draw(pipe, out, title_prefix="", path_png=None):
    g, pl = pipe.g, pipe.planner
    r = g.rect
    extent = (r["x0"], r["x1"], r["y0"], r["y1"])           # imshow extent: left, right, bottom, top (world cm)
    rect_rgb = cv2.cvtColor(out["rect"], cv2.COLOR_BGR2RGB)
    fig, axes = plt.subplots(1, 4, figsize=(20, 5.4))
    poly = g.polygon

    def decorate(ax, title):
        ax.set_title(title, fontsize=11)
        ax.plot(list(poly[:, 0]) + [poly[0, 0]], list(poly[:, 1]) + [poly[0, 1]], "k--", lw=1)
        for i in CORNER_IDS:
            cx, cy = g.corner_centre[i]
            ax.add_patch(Rectangle((cx - TAG_CM["corner"] / 2, cy - TAG_CM["corner"] / 2), TAG_CM["corner"], TAG_CM["corner"], color="k"))
        if out["robot"]:
            ax.plot(out["robot"]["x"], out["robot"]["y"], marker="^", ms=13, color="#5ef29a", mec="k")
        if out["goal_xy"] is not None:
            ax.plot(out["goal_xy"][0], out["goal_xy"][1], marker="*", ms=15, color="#ffd400", mec="k")
        ax.set_xlim(extent[0], extent[1]); ax.set_ylim(extent[2], extent[3]); ax.set_aspect("equal")
        ax.tick_params(labelsize=8)

    # 1 rectified
    axes[0].imshow(rect_rgb, extent=extent)
    decorate(axes[0], f"1. rectified arena ({r['cm_per_px']*10:.0f} mm/px, pose from {g.source})")
    # 2 obstacles
    axes[1].imshow(rect_rgb, extent=extent, alpha=0.35)
    seg = out["seg"]
    if seg is not None:
        mask = seg["persisted"] | (pipe.objects.mask if pipe.objects is not None else False)
        over = np.zeros((*mask.shape, 4)); over[mask] = matplotlib.colors.to_rgba(YELLOW)
        axes[1].imshow(over, extent=extent)
        for o in seg.get("objects", []):
            axes[1].text(o["x"], o["y"], o["label"], fontsize=7, ha="center", color="k", bbox=dict(fc="w", ec="none", alpha=0.7, pad=1))
    decorate(axes[1], "2. obstacles: Lab a/b, robust fit" + (" + Sean's detector" if pipe.objects is not None else ""))
    # 3 C-space (the costmap as it was at this frame)
    cm = out.get("costmap")
    if cm is not None:
        free, infl, occ, inside = cm["free"], cm["inflated"], cm["occupied"], cm["inside"]
        cs = np.zeros((*free.shape, 3))
        cs[:] = matplotlib.colors.to_rgb(OUT)
        cs[inside & free] = (1, 0.98, 0.95)
        cs[inside & infl & ~occ] = matplotlib.colors.to_rgb(INFL)
        cs[inside & occ] = matplotlib.colors.to_rgb(OCC)
        gx0, gy1 = cm["x0"], cm["y1"]
        cext = (gx0, gx0 + cm["cols"] * cm["cell"], gy1 - cm["rows"] * cm["cell"], gy1)
        axes[2].imshow(cs, extent=cext)
        free_pct = 100 * free[inside].mean()
        decorate(axes[2], f"3. C-space, r={pipe.radius:.0f}cm  (free {free_pct:.0f}%)")
        # 4 path
        axes[3].imshow(rect_rgb, extent=extent, alpha=0.55)
        axes[3].imshow(np.dstack([cs, np.where(free, 0.0, 0.45)]), extent=cext)
        plan = out["plan"]
        if plan is not None and len(plan.get("path_cm", [])) > 1:
            p = plan["path_cm"]
            axes[3].plot(p[:, 0], p[:, 1], color="#19c7e8", lw=2.5)
            wp = plan["waypoints"]
            axes[3].plot(wp[:, 0], wp[:, 1], "o", ms=6, color="#ff2fb0")
            length = float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))
            decorate(axes[3], f"4. path {length:.0f}cm, {len(wp)} waypoints")
        else:
            why = "robot not seen" if out["robot"] is None else "BLOCKED: no path from the robot to the goal" if plan is not None and plan.get("blocked") else "at the goal" if plan is not None else "no plan"
            decorate(axes[3], "4. path: " + why)
    else:
        for ax, t in ((axes[2], "3. C-space (not calibrated)"), (axes[3], "4. path (not calibrated)")):
            ax.set_title(t)
    fig.tight_layout()
    if path_png:
        fig.savefig(path_png, dpi=110)
    return fig


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("clip")
    ap.add_argument("--frame", type=int)
    ap.add_argument("--goal", type=float, nargs=2)
    ap.add_argument("--objects", action="store_true")
    ap.add_argument("--out", default=None)
    ap.add_argument("--robot-radius", type=float)
    args = ap.parse_args(argv)
    pipe, k, (frame, out) = run_until(args.clip, args.frame, args.goal, args.objects, args.robot_radius)
    if not pipe.g or not pipe.g.frozen:
        print("not calibrated on this clip: no four corner tags and no tracker pose"); return 1
    out_png = args.out or os.path.join(args.clip, "figure.png")
    draw(pipe, out, path_png=out_png)
    s = pipe.g.summary()
    print(f"frame {k}: pose from {s['source']}, board {tuple(round(v,1) for v in s['board_cm'])} cm, robot {'seen' if out['robot'] else 'not seen'}, "
          f"state {pipe.state}, wrote {out_png}")
    if pipe.objects is not None:
        pipe.objects.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
