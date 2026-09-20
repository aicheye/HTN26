"""One command for the arm: get everything running and grip the Sesame.

    sh run.sh go            do whatever is still missing, then grip the Sesame whenever a camera sees it (auto)
    sh run.sh go --manual   the same, but wait for a space press before each grip
    sh run.sh go --now      grip once as soon as the Sesame is seen, then exit

In order, skipping what is already done:
  1. trackers: if no camera reports the Sesame, start both Pi trackers in the background and wait for them
  2. arm frame: derived from the arm base tag (id 5), which sits on the base: no touching. --offset / --turn
     adjust it; sh run.sh calibrate replaces it with a hands-on measured one; --refresh-frame re-derives it
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
    up = alive_units(tracker)
    if not up:
        if not (os.path.isfile("pi/common.sh") and os.path.isdir("pi/tracker")):
            print("fetching Sean's Pi files from origin/devel/sean (his code, do not commit it from here)")
            subprocess.run(["git", "fetch", "-q", "origin", "devel/sean"], check=False)
            subprocess.run(["git", "restore", "--source=origin/devel/sean", "--", "pi"], check=False)
        if not os.path.exists(os.path.expanduser("~/.ssh/htn_pi")):
            print("the Pi would ask for a password. Run once:  sh pi/setup-key.sh   (password qnxuser), then run this again.")
            return 1
        print(f"no tracker answers at {tracker.host}: starting both on the Pi (log: pi/trackers-live.log)")
        with open("pi/trackers-live.log", "ab") as log:
            subprocess.Popen(["sh", "start_trackers.sh"], stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        for i in range(60):
            time.sleep(1)
            up = alive_units(tracker)
            if up:
                break
        if not up:
            print("no tracker answered in 60 s. Check pi/trackers-live.log and the WiFi."); return 1
    # the camera page first, always: orient the camera so all four corner tags and the Sesame's tag are in view
    print(f"camera view for camera {up[0]} (cameras up: {up}). Orient the camera: 4 corner tags in view, then the Sesame.")
    open_camera_page(tracker.host, up[0])
    print(f"1. trackers at {tracker.host}: ", end="", flush=True)
    if sesame_seen(tracker):
        print(f"a camera sees the Sesame (cameras up: {up})")
    else:
        print(f"no camera reports the Sesame yet. Waiting; use the page to orient the camera.")
        for i in range(240):
            time.sleep(0.5)
            if sesame_seen(tracker):
                print(f"   a camera sees the Sesame after {(i + 1) / 2:.0f} s"); break
            if i % 20 == 19:
                print(f"   still waiting ({(i + 1) // 2} s). In the page: are the corner tags learned (camera height shown)? Is tag 0 on the Sesame in view?")
        else:
            print("   no camera reported the Sesame in 120 s. Fix what the page shows, then run this again."); return 1

    print("2. floor-to-arm frame: ", end="", flush=True)
    if "--offset" in args:
        i = args.index("--offset"); ahead, left = args[i + 1], args[i + 2]
        turn = args[args.index("--turn") + 1] if "--turn" in args else "0"
        args = [a for k, a in enumerate(args) if k not in (i, i + 1, i + 2) and a not in ("--turn", turn)]
    else:
        ahead, left, turn = "0", "0", "0"
    if os.path.exists("arm_frame.json") and "--refresh-frame" not in args:
        print("arm_frame.json present (delete it, or pass --refresh-frame, to derive it again)")
    else:
        args = [a for a in args if a != "--refresh-frame"]
        # tag 5 sits on the arm's base itself, its top edge pointing the way the arm faces (seen in the camera
        # frames), so the base is where the tag is: no touching, no ruler. --offset AHEAD LEFT / --turn adjust it;
        # sh run.sh calibrate (three fingertip touches) replaces it with a measured one.
        print(f"from the arm base tag (id 5): base {ahead} cm ahead, {left} cm left of the tag, turned {turn} deg")
        if subprocess.run([PY, "frame_from_arm_tag.py", "--offset", ahead, left, "--turn", turn]).returncode != 0:
            print("   could not read the arm base tag. Is tag 5 in view of a calibrated camera? For a hands-on frame: sh run.sh calibrate")
            return 1

    print("3. grasp: ", end="", flush=True)
    demos = tracked_demos()
    name = next((a for a in args if not a.startswith("-") and (a in demos or os.path.exists(f"demos/{a}.json"))), None)
    if name:
        print(f"recorded demo '{name}'")
    else:
        name = "auto"
        print("from the tag geometry (no demo needed; record one with sh run.sh record NAME to use it instead)")
    rest = [a for a in args if a != name]

    manual = "--manual" in rest
    rest = [a for a in rest if a != "--manual"]
    print(f"4. pickup ({name})" + (" now, once" if now else ": manual, space = go" if manual else ": AUTO, grips whenever the Sesame is seen in reach; q quits"))
    cmd = [PY, "sesame_pickup.py", name] + rest + (["--once"] if now else [] if manual else ["--auto"])
    return subprocess.run(cmd).returncode


if __name__ == "__main__":
    sys.exit(main())
