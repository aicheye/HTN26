"""One command for the arm: get everything running and grip the Sesame.

    sh run.sh go            do whatever is still missing, then wait for a keypress to grip
    sh run.sh go --now      the same, then grip immediately once the Sesame is seen, and exit

In order, skipping what is already done:
  1. trackers: if no camera reports the Sesame, start both Pi trackers in the background and wait for them
  2. arm frame: if arm_frame.json is missing, run calibrate_arm_frame.py (fingertips on the tag, 3 spots)
  3. demo: if no demo has the tracker's tag pose at its grasp mark, run record_demo.py grip
  4. pickup: sesame_pickup.py with the newest tracked demo (space = go, p = plan, q = quit)
Extra arguments after --now / the demo name go to sesame_pickup.py (for example --drop-offset 0 10).
"""
import glob
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
PY = sys.executable


def tracked_demos():
    out = []
    for path in sorted(glob.glob("demos/*.json"), key=os.path.getmtime):
        try:
            with open(path) as f:
                d = json.load(f)
            g = next((k for k in d["keyframes"] if k["label"] == "grasp"), None)
            if g and d["samples"][g["index"]].get("robot_floor"):
                out.append(os.path.basename(path)[:-5])
        except Exception:
            pass
    return out


def sesame_seen(tracker):
    return tracker.observe() is not None


def main():
    args = sys.argv[1:]
    now = "--now" in args
    args = [a for a in args if a != "--now"]
    from sesame_tracker import Tracker, alive_units, open_camera_page
    from so101_safe import default_port

    if not default_port():
        print("no arm found: plug the SO-101 in (USB), then run this again"); return 1
    tracker = Tracker()
    print(f"1. trackers at {tracker.host}: ", end="", flush=True)
    up = alive_units(tracker)
    if sesame_seen(tracker):
        print(f"a camera sees the Sesame (cameras up: {up})")
    elif up:
        # the trackers run; the Sesame's tag is just not in a frame right now (a 3.6 cm tag is missed often)
        print(f"cameras {up} are up but none reports the Sesame. Opening the camera view; put the Sesame where a camera sees its tag.")
        open_camera_page(tracker.host, up[0])
        for i in range(120):
            time.sleep(0.5)
            if sesame_seen(tracker):
                print(f"   a camera sees the Sesame after {(i + 1) / 2:.0f} s"); break
            if i % 20 == 19:
                print(f"   still waiting ({(i + 1) // 2} s). In the page: are the corner tags learned (camera height shown)? Is tag 0 on the Sesame in view?")
        else:
            print("   no camera reported the Sesame in 60 s. Fix what the page shows, then run this again."); return 1
    else:
        print("no tracker answers")
        if not (os.path.isfile("pi/common.sh") and os.path.isdir("pi/tracker")):
            print("   fetching Sean's Pi files from origin/devel/sean (his code, do not commit it from here)")
            subprocess.run(["git", "fetch", "-q", "origin", "devel/sean"], check=False)
            subprocess.run(["git", "restore", "--source=origin/devel/sean", "--", "pi"], check=False)
        if not os.path.exists(os.path.expanduser("~/.ssh/htn_pi")):
            print("   the Pi would ask for a password, which cannot be answered in the background.")
            print("   Run once:  sh pi/setup-key.sh   (the Pi's password is qnxuser), then run this again.")
            return 1
        print("   starting both trackers on the Pi in the background (log: pi/trackers-live.log)")
        with open("pi/trackers-live.log", "ab") as log:
            subprocess.Popen(["sh", "start_trackers.sh"], stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        opened = False
        for i in range(120):
            time.sleep(1)
            if sesame_seen(tracker):
                print(f"   camera sees the Sesame after {i + 1} s"); break
            if not opened and alive_units(tracker):
                opened = open_camera_page(tracker.host, alive_units(tracker)[0])
            if i % 15 == 14:
                print(f"   still waiting ({i + 1} s). Trackers up: {alive_units(tracker) or 'none yet'}. Hold the camera still with 3+ corner tags in view until it learns the floor.")
        else:
            print("   no camera sees the Sesame after 120 s. Check pi/trackers-live.log, the WiFi, and the page."); return 1

    print("2. floor-to-arm frame: ", end="", flush=True)
    if os.path.exists("arm_frame.json"):
        print("arm_frame.json present")
    else:
        print("missing, calibrating now (hold the arm)")
        if subprocess.run([PY, "calibrate_arm_frame.py"]).returncode != 0:
            return 1

    print("3. grasp demo: ", end="", flush=True)
    demos = tracked_demos()
    name = next((a for a in args if not a.startswith("-")), None)
    if name and name not in demos:
        print(f"'{name}' has no tracker pose at its grasp mark; recording it now (hold the arm)")
        if subprocess.run([PY, "record_demo.py", name]).returncode != 0:
            return 1
    elif not demos:
        print("none recorded with the tracker; recording 'grip' now (hold the arm)")
        if subprocess.run([PY, "record_demo.py", "grip"]).returncode != 0:
            return 1
        name = "grip"
    else:
        name = name or demos[-1]
        print(f"using '{name}'")
    rest = [a for a in args if a != name]

    print(f"4. pickup with '{name}'" + (" now" if now else ": space = go, p = plan, q = quit"))
    cmd = [PY, "sesame_pickup.py", name] + rest + (["--once"] if now else [])
    return subprocess.run(cmd).returncode


if __name__ == "__main__":
    sys.exit(main())
