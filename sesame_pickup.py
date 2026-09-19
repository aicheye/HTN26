"""Press a key: the arm finds the Sesame through the Pi tracker, grips it the demonstrated way, lifts it,
carries it a short way to the side, sets it down, lets go and returns to its ready pose.

Run: .venv/bin/python sesame_pickup.py DEMO [--tracker HOST] [--frame arm_frame.json] [--port PORT]
                                       [--drop-offset DX DY | --drop X Y] [--lift 6] [--once] [--dry-run]

Keys:  space / g  find the Sesame and run the whole pick-and-place
       p          plan only: print every phase and whether it is reachable, no motion
       z          go to the ready pose (calibrated midpoint, gripper open)
       o          open the gripper where it is
       q          quit (torque stays on)

Where it puts the Sesame down: --drop-offset DX DY is in cm relative to the pick point in the arm's base
frame (+y is the arm's left); the default tries a few nearby offsets and takes the first the arm can reach.
--drop X Y is an absolute base-frame position in cm. The end-position logic can replace pick_drop() later.

Phases, all solved through so101_ik.ik and checked before the first move:
  grip      the demo's approach and close, placed around the Sesame's current tag pose (grasp_robot.py)
  lift      straight up by --lift cm. Held vertically the arm reaches nothing above about 12 cm, so the
            gripper tilts on the way up, to the least tilt that reaches the carry height (the Sesame is
            already held, so a tilt does no harm)
  carry     straight line at that height to the drop point, jaws kept at the same angle to the body
  lower     straight down to the grasp height, tilting back to vertical for the release
  release   open the gripper to the demo's open value
  retract   straight up, then a slow joint-space slew to the ready pose
"""
import argparse
import sys
import time

import numpy as np

from arm_frame import ArmFrame
from grasp_robot import grasp_segment, tag_pose_at, relative_offsets, place, solve, PRE_APPROACH_CM
from record_demo import Keys
from replay_demo import Arm, load, FPS
from sesame_tracker import Tracker, Poller
from so101_ik import JOINTS, ik

DEFAULT_PORT = "/dev/tty.usbmodem5AE60798501"
HOLD_S = 0.6            # keep the demo running this long past the grasp mark (the jaws finish closing)
LIFT_CM = 6.0
CARRY_PITCHES = (-92.0, -85.0, -78.0, -70.0, -62.0, -55.0)   # tried in order: the least tilt that reaches the carry height wins
CARRY_CM_PER_S = 5.0
VERTICAL_CM_PER_S = 4.0
RELEASE_S = 1.0
DROP_CANDIDATES = [(0, 10), (0, -10), (0, 7), (0, -7), (-4, 8), (-4, -8), (-6, 0)]
READY = {j: 0.0 for j in JOINTS}


def cartesian(p0, p1, seconds, t_start, gripper):
    """Straight-line poses from p0 to p1 (dicts with x y z jaw_yaw pitch, m and deg) over seconds; the
    pitch blends from p0's to p1's, the jaw heading stays."""
    n = max(2, int(seconds * FPS))
    out = []
    for i in range(n + 1):
        f = i / n
        p = {k: p0[k] + (p1[k] - p0[k]) * f for k in ("x", "y", "z", "pitch")}
        p["jaw_yaw"] = p0["jaw_yaw"]; p["gripper"] = gripper; p["t"] = t_start + seconds * f
        out.append(p)
    return out


def solvable(poses):
    return all(ik(p["x"], p["y"], p["z"], p["jaw_yaw"], p["pitch"]) is not None for p in poses)


def carry_pitch(pick, drop, lift_m):
    """The least tilt (closest to the grasp pitch) at which both the lifted pick and the lifted drop are reachable."""
    for pitch in CARRY_PITCHES:
        if pitch > pick["pitch"] + 40:
            break
        a = {**pick, "z": pick["z"] + lift_m, "pitch": pitch}
        b = {**drop, "z": drop["z"] + lift_m, "pitch": pitch}
        if solvable([a, b]):
            return pitch
    return None


def pick_drop(pick, args):
    """The drop pose (same height and jaw angle as the pick) and the carry pitch.
    Replace this with the end-position logic later."""
    if args.drop:
        cands = [{**pick, "x": args.drop[0] / 100, "y": args.drop[1] / 100}]
    else:
        offsets = [tuple(args.drop_offset)] if args.drop_offset else DROP_CANDIDATES
        cands = [{**pick, "x": pick["x"] + dx / 100, "y": pick["y"] + dy / 100} for dx, dy in offsets]
    for cand in cands:
        if not solvable([cand]):
            continue
        pitch = carry_pitch(pick, cand, args.lift / 100)
        if pitch is not None:
            return cand, pitch
    return None, None


def plan(demo, frame0, obs, args):
    """All phases as one trajectory [(t, joints, gripper)], or (None, reason)."""
    samples, grasp, release = grasp_segment(demo, args.approach, 0.0)
    samples = [s for s in samples if s["t"] <= grasp["t"] + HOLD_S]
    tag_demo_floor = tag_pose_at(grasp, demo)
    if not tag_demo_floor:
        return None, "the demo has no tracker pose at the grasp mark: record it with the tracker running"
    frame_demo = frame0.adjusted_for_arm_tag(demo["samples"][grasp["index"]].get("arm_floor"))
    tag_demo = frame_demo.pose_to_base(tag_demo_floor)
    frame_now = frame0.adjusted_for_arm_tag(obs["arm"])
    tag_now = frame_now.pose_to_base(obs["robot"])
    offsets = relative_offsets(samples, tag_demo)
    poses = place(offsets, tag_now)
    for lift in (PRE_APPROACH_CM, PRE_APPROACH_CM / 2, 1.0):
        pre = dict(poses[0]); pre["z"] += lift / 100; pre["t"] = poses[0]["t"] - 1.5
        if ik(pre["x"], pre["y"], pre["z"], pre["jaw_yaw"], pre["pitch"]) is not None:
            poses = [pre] + poses
            break
    open_value = max(s["gripper"] for s in demo["samples"] if s["t"] <= grasp["t"])   # how far the jaws were open before the grip
    grip_end = poses[-1]
    pick = {k: grip_end[k] for k in ("x", "y", "z", "jaw_yaw", "pitch")}
    drop, pitch = pick_drop(pick, args)
    if drop is None:
        return None, "no reachable drop point near the pick, even tilted for the carry: lower --lift, or pass --drop-offset / --drop"
    closed = max(0.0, grip_end["gripper"] - args.squeeze)
    t = grip_end["t"]
    lifted = {**pick, "z": pick["z"] + args.lift / 100, "pitch": pitch}
    phases = [("grip", poses)]
    seg = cartesian(pick, lifted, args.lift / VERTICAL_CM_PER_S, t, closed); t = seg[-1]["t"]; phases.append(("lift", seg))
    drop_lifted = {**drop, "z": lifted["z"], "pitch": pitch}
    dist = 100 * np.hypot(drop["x"] - pick["x"], drop["y"] - pick["y"])
    seg = cartesian(lifted, drop_lifted, max(1.0, dist / CARRY_CM_PER_S), t, closed); t = seg[-1]["t"]; phases.append(("carry", seg))
    seg = cartesian(drop_lifted, drop, args.lift / VERTICAL_CM_PER_S, t, closed); t = seg[-1]["t"]; phases.append(("lower", seg))
    seg = cartesian(drop, drop, RELEASE_S, t, closed)
    for i, p in enumerate(seg):
        p["gripper"] = closed + (open_value - closed) * i / (len(seg) - 1)
    t = seg[-1]["t"]; phases.append(("release", seg))
    seg = cartesian(drop, drop_lifted, args.lift / VERTICAL_CM_PER_S, t, open_value); phases.append(("retract", seg))

    traj, report = [], []
    t0 = phases[0][1][0]["t"]
    for name, ps in phases:
        tr, failed = solve(ps, grasp["t"] if name == "grip" else -1e9, None if name == "grip" else -1e9, args.squeeze if name == "grip" else 0.0)
        if name != "grip":
            tr = [(p["t"] - ps[0]["t"] + ps[0]["t"] - t0, q, g) for (_, q, g), p in zip(tr, [p for p in ps if ik(p["x"], p["y"], p["z"], p["jaw_yaw"], p["pitch"]) is not None])]
        else:
            tr = [(tt + ps[0]["t"] - t0, q, g) for tt, q, g in tr]
        last = ps[-1]
        report.append(f"  {name:8s} {len(tr):3d}/{len(ps):3d} poses  ends at x={100*last['x']:5.1f} y={100*last['y']:5.1f} z={100*last['z']:5.1f} cm  pitch {last['pitch']:5.0f}  jaws {last['jaw_yaw']:6.1f}  gripper {last['gripper']:4.0f}")
        if failed:
            f = failed[0][1]
            return None, "\n".join(report) + f"\n  {name}: {len(failed)} poses unreachable or unsafe, first x={100*f['x']:.1f} y={100*f['y']:.1f} z={100*f['z']:.1f}"
        traj += tr
    info = {"tag_now": tag_now, "obs": obs, "pick": pick, "drop": drop, "report": report, "seconds": traj[-1][0], "carry_pitch": pitch}
    return traj, info


def execute(arm, traj, speed=1.0):
    _, q0, g0 = traj[0]
    arm.slew(q0, g0)
    time.sleep(0.3)
    start = time.perf_counter(); i = 0
    while i < len(traj):
        now = (time.perf_counter() - start) * speed
        while i + 1 < len(traj) and traj[i + 1][0] <= now:
            i += 1
        t, q, g = traj[i]
        if i + 1 < len(traj):
            t2, q2, g2 = traj[i + 1]
            f = 0.0 if t2 <= t else min(1.0, (now - t) / (t2 - t))
            q = {j: q[j] + (q2[j] - q[j]) * f for j in JOINTS}; g = g + (g2 - g) * f
        arm.send(q, g)
        if i + 1 >= len(traj):
            break
        time.sleep(1 / FPS)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("demo")
    ap.add_argument("--tracker"); ap.add_argument("--frame", default="arm_frame.json"); ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--drop-offset", type=float, nargs=2, metavar=("DX", "DY"), help="cm from the pick point, base frame")
    ap.add_argument("--drop", type=float, nargs=2, metavar=("X", "Y"), help="absolute base-frame cm")
    ap.add_argument("--lift", type=float, default=LIFT_CM); ap.add_argument("--approach", type=float, default=4.0)
    ap.add_argument("--squeeze", type=float, default=8.0); ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--once", action="store_true", help="run once without the key loop")
    ap.add_argument("--dry-run", action="store_true", help="plan only, never connect to the arm")
    args = ap.parse_args()

    demo = load(args.demo); frame0 = ArmFrame.load(args.frame)
    tracker = Tracker(args.tracker); poller = Poller(tracker, period=0.3)
    arm = None if args.dry_run else Arm(args.port)

    def run(dry):
        obs = tracker.observe_steady(1.0)
        if obs is None:
            print("\n  no camera sees the Sesame's tag"); return False
        traj, info = plan(demo, frame0, obs, args)
        if traj is None:
            print("\n  REFUSED:\n" + info); return False
        tn = info["tag_now"]
        print(f"\n  Sesame at base ({tn['x']:.1f}, {tn['y']:.1f}) cm heading {tn['heading']:.0f} [camera {obs['unit']}]; "
              f"pick x={100*info['pick']['x']:.1f} y={100*info['pick']['y']:.1f}, drop x={100*info['drop']['x']:.1f} y={100*info['drop']['y']:.1f}, "
              f"carry tilted to pitch {info['carry_pitch']:.0f}, {info['seconds']:.1f} s")
        print("\n".join(info["report"]))
        if dry or arm is None:
            return True
        print("  running")
        execute(arm, traj, args.speed)
        print("  back to the ready pose")
        arm.slew(READY, traj[-1][2])
        print("  done")
        return True

    if args.once or args.dry_run:
        ok = run(args.dry_run)
        if arm: arm.close(False)
        return 0 if ok else 1

    print("space/g = pick up and move the Sesame   p = plan only   z = ready pose   o = open gripper   q = quit")
    keys = Keys()
    try:
        last_line = ""
        while True:
            key = keys.get()
            if key in (" ", "g"):
                run(False)
            elif key == "p":
                run(True)
            elif key == "z":
                arm.slew(READY, 40.0); print("\n  ready pose")
            elif key == "o":
                q = arm.read(); arm.send({j: q[j] for j in JOINTS}, 40.0); print("\n  gripper opened")
            elif key == "q":
                break
            o = poller.get()
            line = (f"Sesame seen by camera {o['unit']} at floor ({o['robot']['x']:.0f}, {o['robot']['y']:.0f}) heading {o['robot']['heading']:.0f}   "
                    if o else "Sesame not seen by any camera   ")
            if line != last_line:
                sys.stdout.write("\r" + line); sys.stdout.flush(); last_line = line
            time.sleep(0.1)
    finally:
        keys.restore()
        if arm: arm.close(False)
        print("\ntorque left on")
    return 0


if __name__ == "__main__":
    sys.exit(main())
