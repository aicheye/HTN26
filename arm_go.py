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
    from sesame_tracker import Tracker
    from so101_safe import default_port

    if not default_port():
        print("no arm found: plug the SO-101 in (USB), then run this again"); return 1
    tracker = Tracker()
    print(f"1. trackers at {tracker.host}: ", end="", flush=True)
    if sesame_seen(tracker):
        print("a camera sees the Sesame")
    else:
        print("no camera sees the Sesame")
        if not (os.path.isfile("pi/common.sh") and os.path.isdir("pi/tracker")):
            print("   fetching Sean's Pi files from origin/devel/sean (his code, do not commit it from here)")
            subprocess.run(["git", "fetch", "-q", "origin", "devel/sean"], check=False)
            subprocess.run(["git", "restore", "--source=origin/devel/sean", "--", "pi"], check=False)
        print("   starting both trackers on the Pi in the background (log: pi/trackers-live.log)")
        with open("pi/trackers-live.log", "ab") as log:
            subprocess.Popen(["sh", "start_trackers.sh"], stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        for i in range(90):
            time.sleep(1)
            if sesame_seen(tracker):
                print(f"   camera sees the Sesame after {i + 1} s"); break
            if i % 10 == 9:
                print(f"   still waiting ({i + 1} s). Is the laptop on the robot's WiFi? Is the Sesame's tag in view?")
        else:
            print("   no camera sees the Sesame after 90 s. Check pi/trackers-live.log, the WiFi, and the tags."); return 1

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
