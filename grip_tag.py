"""Grip the Sesame at the centre of its tag. One target, one grip, every number printed.

    sh run.sh grip [--dry-run] [--grip-z 9.0] [--jaw-angle 90] [--tag-offset AHEAD LEFT] [--hover 5] [--lift 5]

How the target is found:
  1. From one camera, median of several frames holding both the Sesame's tag and the arm's tag (floor cm).
  2. Sesame minus arm tag, rotated by the arm tag's heading -> the tag centre ahead/left of the arm's base.
     The base is where the arm's tag is (--tag-offset shifts that).
  3. Target: x, y = tag centre; z = --grip-z (9.0 cm, the demonstrated grip height); pitch straight down;
     jaw heading = tag heading + --jaw-angle, kept exactly (the moving jaw always on the same side).
  4. Closed-form IK. If straight down cannot reach, the gripper tilts only as far as needed, in 5 deg steps.
Then: rest -> hover above the target, jaws open -> straight down -> close over 1 s -> lift -> rest.
"""
import argparse
import sys
import time

import numpy as np

from replay_demo import Arm, FPS
from sesame_pickup import READY, REST, GRIPPER_REST, go_home
from sesame_tracker import Tracker
from so101_ik import JOINTS, ik, fk
from so101_safe import default_port

OPEN, CLOSED = 20.0, 0.0
PITCHES = [-90, -85, -80, -75, -70, -65, -60, -55, -50, -45, -40]


def solve(x_cm, y_cm, z_cm, jaw):
    """Joints for the target, straight down if possible. Returns (joints, pitch) or (None, None)."""
    for pitch in PITCHES:
        q = ik(x_cm / 100, y_cm / 100, z_cm / 100, jaw, pitch, exact_jaw=True)
        if q is not None:
            return q, pitch
    return None, None


def line(a, b, seconds, grip_a, grip_b=None):
    """Poses along a straight line between two joint solutions' targets are solved per step by the caller;
    here: linear joint-space interpolation between two solved poses, with the gripper blending."""
    n = max(2, int(seconds * FPS)); grip_b = grip_a if grip_b is None else grip_b
    return [({j: a[j] + (b[j] - a[j]) * i / n for j in JOINTS}, grip_a + (grip_b - grip_a) * i / n) for i in range(n + 1)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tracker"); ap.add_argument("--port", default=default_port())
    ap.add_argument("--grip-z", type=float, default=9.0, help="cm above the arm's base where the jaws close (demo: 9.0)")
    ap.add_argument("--jaw-angle", type=float, default=90.0, help="jaw axis relative to the tag heading; 90/-90 across the body with the jaws swapped, 0 along")
    ap.add_argument("--tag-offset", type=float, nargs=2, metavar=("AHEAD", "LEFT"), default=[0.0, 0.0], help="the arm's base relative to its tag (cm)")
    ap.add_argument("--hover", type=float, default=5.0); ap.add_argument("--lift", type=float, default=5.0)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    tracker = Tracker(args.tracker)
    print(f"1. reading both tags from one camera at {tracker.host}...")
    pair = tracker.observe_pair(min_samples=5, timeout=40.0)
    if pair is None:
        print("   no camera reported both the Sesame's tag and the arm's tag"); return 1
    r, a = pair["robot"], pair["arm"]
    print(f"   camera {pair['unit']}, {pair['samples']} frames: Sesame tag at floor ({r['x']:.1f}, {r['y']:.1f}) heading {r['heading']:.1f}; "
          f"arm tag at ({a['x']:.1f}, {a['y']:.1f}) heading {a['heading']:.1f}")

    th = np.radians(a["heading"])
    d = np.array([r["x"] - a["x"], r["y"] - a["y"]])
    ahead = d @ [np.cos(th), np.sin(th)]
    left = d @ [-np.sin(th), np.cos(th)]
    x, y = ahead - args.tag_offset[0], left - args.tag_offset[1]
    jaw = (r["heading"] - a["heading"] + args.jaw_angle + 180) % 360 - 180
    print(f"2. tag centre relative to the arm's tag: {ahead:.1f} cm ahead, {left:+.1f} cm left; Sesame heading {r['heading'] - a['heading']:+.1f} deg relative to the arm")
    print(f"3. target in the arm's frame: x={x:.1f} y={y:.1f} z={args.grip_z:.1f} cm, jaws at {jaw:+.1f} deg, {np.hypot(x, y):.1f} cm from the base")

    q_grip, pitch = solve(x, y, args.grip_z, jaw)
    if q_grip is None:
        print("   IK: unreachable at every tilt from straight down to 40 deg"); return 1
    q_hover, p_hover = solve(x, y, args.grip_z + args.hover, jaw)
    q_lift, p_lift = solve(x, y, args.grip_z + args.lift, jaw)
    if q_hover is None or q_lift is None:
        print("   IK: the hover/lift height above the target is unreachable; lower --hover/--lift"); return 1
    f = fk(q_grip)
    print(f"4. IK: grip pitch {pitch} deg" + (" (straight down)" if pitch == -90 else " (tilted to reach)") + f", hover pitch {p_hover}, lift pitch {p_lift}")
    print(f"   joints at the grip: " + "  ".join(f"{j.split('_')[0]}={q_grip[j]:.1f}" for j in JOINTS))
    print(f"   check, FK of those joints: x={100*f['x']:.1f} y={100*f['y']:.1f} z={100*f['z']:.1f} cm, jaws {f['jaw_yaw']:+.1f} deg")
    if args.dry_run:
        return 0

    arm = Arm(args.port)
    try:
        print("5. moving: rest -> hover")
        arm.slew(READY, OPEN); arm.slew(q_hover, OPEN); time.sleep(0.3)
        print("   down to the target")
        for q, g in line(q_hover, q_grip, 2.0, OPEN):
            arm.send(q, g); time.sleep(1 / FPS)
        print("   closing")
        for q, g in line(q_grip, q_grip, 1.0, OPEN, CLOSED):
            arm.send(q, g); time.sleep(1 / FPS)
        time.sleep(0.5)
        print("   lift")
        for q, g in line(q_grip, q_lift, 2.0, CLOSED):
            arm.send(q, g); time.sleep(1 / FPS)
        time.sleep(0.5)
        after = tracker.wait_for_robot(10.0, say=None)
        if after:
            moved = np.hypot(after["robot"]["x"] - r["x"], after["robot"]["y"] - r["y"])
            print(f"   the Sesame's tag moved {moved:.1f} cm during the lift: " + ("GRIP WORKED" if moved > 2.0 else "grip missed"))
        print("   back to rest (still holding)")
        go_home(arm, CLOSED)
    finally:
        arm.close(False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
