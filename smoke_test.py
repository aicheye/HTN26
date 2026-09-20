"""Smoke test: one quick pass over every component, each reported PASS / FAIL / SKIP with its time.

    .venv/bin/python smoke_test.py            (or: sh run.sh smoke)

Renders the synthetic recordings first if they are missing (about a minute). Steps that need hardware or
files not present are SKIPped with the reason, never failed: the arm, the real recording, Sean's vision/.
Exit code 1 if any step FAILs.
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
results = []


def step(name, fn):
    t0 = time.time()
    try:
        out = fn()
        status, detail = ("SKIP", out[5:]) if isinstance(out, str) and out.startswith("SKIP ") else ("PASS", out or "")
    except Exception as e:
        status, detail = "FAIL", f"{type(e).__name__}: {str(e)[:160]}"
    results.append((name, status))
    print(f"  {status:4s} {name:34s} {time.time() - t0:5.1f} s  {detail}")


def run(cmd, timeout=600):
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if p.returncode != 0:
        tail = "\n".join((p.stdout + p.stderr).strip().splitlines()[-6:])
        raise RuntimeError(f"exit {p.returncode}: {tail}")
    return p.stdout


def s_env():
    import lerobot, scipy, placo, cv2  # noqa: F401
    return f"lerobot {lerobot.__version__}, opencv {cv2.__version__}"


def s_solver():
    out = run([PY, "-m", "pytest", "-q", "test_so101_ik.py"])
    return out.strip().splitlines()[-1]


def s_pickup():
    out = run([PY, "test_pickup.py"])
    return out.strip().splitlines()[-1]


def s_tracker():
    from fake_tracker import serve
    from sesame_tracker import Tracker
    servers = serve((30, 20, 45), (70, 8, 180), True, 0.05, units=(3, 4))
    try:
        time.sleep(0.2)
        o = Tracker("localhost").observe_steady(0.4)
        assert o and abs(o["robot"]["x"] - 30) < 0.5 and o["unit"] == 3, o
        return f"camera {o['unit']} reports the robot at ({o['robot']['x']:.1f}, {o['robot']['y']:.1f})"
    finally:
        for s in servers:
            s.shutdown()


def s_synth():
    if not os.path.isdir("recordings/synth-obstacles"):
        run([PY, "-m", "nav.synth"], timeout=900)
        return "rendered"
    return "present"


def s_nav():
    out = run([PY, "-m", "nav.verify", "recordings/synth-obstacles"])
    line = [l for l in out.splitlines() if l.startswith("RESULT")][-1]
    if "PASS" not in line:
        raise RuntimeError(line)
    return line


def s_objects():
    if not os.path.isfile("vision/detect.py"):
        return "SKIP vision/ not in this checkout (git restore --source=origin/devel/sean -- vision)"
    out = run([PY, "-m", "nav.verify", "recordings/synth-obstacles", "--objects"], timeout=900)
    line = [l for l in out.splitlines() if "object detector" in l][-1]
    if "PASS" not in line:
        raise RuntimeError(line.strip())
    return line.strip()[5:80]


def s_figure():
    out = run([PY, "-m", "nav.figure", "recordings/synth-obstacles", "--out", "recordings/synth-obstacles/figure.png"])
    assert os.path.getsize("recordings/synth-obstacles/figure.png") > 20000
    return "recordings/synth-obstacles/figure.png"


def s_real():
    clips = sorted(d for d in glob.glob("recordings/rec-*") if os.path.isfile(os.path.join(d, "states.jsonl")))
    if not clips:
        return "SKIP no real recording in recordings/rec-* (bash pi/pull-recordings.sh)"
    clip = clips[-1]
    out = run([PY, "-m", "nav.run", "--replay", clip, "--fast", "--headless", "--summary", "/tmp/nav-smoke-real.json"])
    s = json.load(open("/tmp/nav-smoke-real.json"))
    assert s["frozen"], "never calibrated (no four tags and no tracker pose)"
    run([PY, "-m", "nav.figure", clip, "--out", os.path.join(clip, "figure.png")])
    return f"{clip}: pose from {s['source']}, robot radius {s['robot_radius_cm']:.1f} cm, z std {s['z_std_cm']:.2f} cm, figure written"


def s_live():
    """The live path: a fake Pi replays a recording over HTTP with the X-State header, the loop runs against it."""
    clips = sorted(d for d in glob.glob("recordings/rec-*") + glob.glob("recordings/synth-driving") if os.path.isfile(os.path.join(d, "states.jsonl")))
    if not clips:
        return "SKIP no recording to replay as a live camera"
    from fake_tracker import serve
    from nav.run import LiveSource, Pipeline
    servers = serve(units=(3,), recording=clips[-1], fps=5.0)
    try:
        time.sleep(0.5)
        src, pipe = LiveSource("localhost", 3), Pipeline()
        end = time.time() + 8
        while time.time() < end:
            frame, t, rec = src.next()
            pipe.tick(frame, t, rec["state"])
        s = pipe.summary()
        assert s["frozen"], "never calibrated on the live stream"
        assert pipe.state in ("NAVIGATING", "HOLD", "ARRIVED", "PLANNING", "BLOCKED"), pipe.state
        return f"{os.path.basename(clips[-1])} over HTTP: pose from {s['source']}, {s['frames']} ticks, state {pipe.state}"
    finally:
        for sv in servers:
            sv.shutdown()


def s_preflight():
    run([PY, "preflight.py"]) if os.path.isfile("preflight.py") else None
    return "ran" if os.path.isfile("preflight.py") else "SKIP preflight.py not in this checkout"


def s_arm():
    from so101_safe import default_port
    port = default_port()
    if not port:
        return "SKIP no arm plugged in"
    out = run([PY, "check_arm.py", port], timeout=60)
    n = sum(1 for l in out.splitlines() if "pos=" in l)
    assert n == 6, f"{n} of 6 motors answered"
    return f"6 motors on {port}"


def main():
    print("smoke test")
    step("python environment", s_env)
    step("IK solver tests", s_solver)
    step("pick-and-place chain (offline)", s_pickup)
    step("tracker client (fake tracker)", s_tracker)
    step("synthetic recordings", s_synth)
    step("nav pipeline on synthetic clip", s_nav)
    step("Sean's object detector in nav", s_objects)
    step("four-panel figure", s_figure)
    step("nav on a real recording", s_real)
    step("live path (fake Pi over HTTP)", s_live)
    step("preflight", s_preflight)
    step("arm ping", s_arm)
    failed = [n for n, st in results if st == "FAIL"]
    print("RESULT:", "PASS" if not failed else f"FAIL ({', '.join(failed)})")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
