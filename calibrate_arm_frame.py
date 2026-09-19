"""Calibrate the transform from the tracker's floor frame to the arm's base_link frame.

Run: .venv/bin/python calibrate_arm_frame.py [--tracker HOST] [--port PORT] [--out arm_frame.json] [--fake]

Needs the tracker running on the Pi and the quadruped on the board where a camera sees its tag. Torque goes
off so the arm is limp: hold it. For each point:
  1. put the quadruped somewhere on the board in front of the arm
  2. guide the gripper so the fingertips are centred right on top of the quadruped's tag
  3. press space: the gripper's position (forward kinematics) and the tag's tracker position are paired up
Move the quadruped and repeat. Two points fix the transform; take three or four spread out so the residual
means something. Press q to solve and save, x to abort. The arm base tag's floor pose is saved as well, so a
later move of the arm or the board is detected and corrected for.
"""
import argparse
import sys
import time

import numpy as np

from arm_frame import ArmFrame
from record_demo import Arm, FakeArm, Keys, DEFAULT_PORT
from sesame_tracker import Tracker
from so101_ik import fk


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tracker", help="Pi host (default: pi/host or qnxpi78.local)")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--out", default="arm_frame.json")
    ap.add_argument("--fake", action="store_true", help="no arm: a fake pose (for trying the flow)")
    args = ap.parse_args()

    tracker = Tracker(args.tracker)
    if tracker.observe() is None:
        print(f"no tracker sees the robot at {tracker.host} (units {tracker.units}). Start it: sh pi/live.sh, and check the robot's tag is in view.")
        return 1
    arm = FakeArm() if args.fake else Arm(args.port)
    print("torque is OFF. Guide the fingertips onto the quadruped's tag, then press space. q = solve and save, x = abort")
    keys = Keys()
    floor_pts, base_pts, arm_tags, mirrored, units = [], [], [], None, []
    try:
        while True:
            key = keys.get()
            if key == " ":
                obs = tracker.observe_steady(0.8)
                if obs is None:
                    print("\n  the tracker does not see the robot right now, try again")
                    continue
                pose = fk(arm.read())
                floor_pts.append([obs["robot"]["x"], obs["robot"]["y"]])
                base_pts.append([100 * pose["x"], 100 * pose["y"]])
                if obs["arm"]:
                    arm_tags.append(obs["arm"])
                mirrored = not obs["zUp"]
                units.append(obs["unit"])
                print(f"\n  point {len(floor_pts)}: floor ({obs['robot']['x']:.1f}, {obs['robot']['y']:.1f}) cm  <->  base ({100*pose['x']:.1f}, {100*pose['y']:.1f}) cm  "
                      f"[camera {obs['unit']}, {obs['floorMarkers']} floor markers]")
            elif key == "q":
                break
            elif key == "x":
                print("\naborted"); return 1
            pose = fk(arm.read())
            sys.stdout.write(f"\rgripper at x={100*pose['x']:5.1f} y={100*pose['y']:5.1f} z={100*pose['z']:5.1f} cm   [{len(floor_pts)} points]   ")
            sys.stdout.flush()
            time.sleep(0.05)
    finally:
        keys.restore()
        arm.close()
        print()
    if len(floor_pts) < 2:
        print("need at least two points")
        return 1
    arm_tag = None
    if arm_tags:
        arm_tag = {k: float(np.median([a[k] for a in arm_tags])) for k in ("x", "y")}
        h = np.radians([a["heading"] for a in arm_tags]); arm_tag["heading"] = float(np.degrees(np.arctan2(np.median(np.sin(h)), np.median(np.cos(h)))))
    frame = ArmFrame.fit(floor_pts, base_pts, mirrored, arm_tag)
    frame.units = sorted(set(units))
    frame.save(args.out)
    print(frame.describe())
    if frame.residual_cm > 1.0:
        print(f"WARNING: residual {frame.residual_cm:.2f} cm. A point was probably not centred on the tag; redo the calibration.")
    print(f"saved {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
