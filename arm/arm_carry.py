"""Carries the Sesame past an obstacle when the bridge asks for it.

    sh run.sh carry                          the bridge on this machine
    sh run.sh carry http://192.168.1.20:8080 the bridge on another machine (Sean's laptop)

When a goto finds no walkable path, bridge/bridge.mjs picks places on the goal's side of the obstacle where the
robot could be set down, and offers them on GET /carry as {"id", "drops": [[x, y], ...]} in the tracker's floor
frame, cm, nearest to the arm's base first. This script polls for that, and hands the movement to sesame_pickup.py:
it plans each drop point with --dry-run until one is reachable, runs that one for real, and reports the result with
POST /carry {"id", "ok", "reason"}. The bridge then plans the rest of the way, or fails the goto with the reason.

Needs what sesame_pickup.py needs: the arm plugged in, arm_frame.json, and a Pi tracker that sees the Sesame and
the arm's base tag. Extra arguments after -- go to sesame_pickup.py (for example -- --lift 6).
"""
import json
import subprocess
import sys
import time
import urllib.request

PICKUP = [sys.executable, "sesame_pickup.py", "auto", "--once"]


def ask(url, body=None):
    request = urllib.request.Request(url, json.dumps(body).encode() if body is not None else None, {"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=5) as response:
        return json.loads(response.read())


def pickup(drop, extra, dry):
    """Runs sesame_pickup.py for one drop point. Returns (ok, the reason it gave when it refused)."""
    command = PICKUP + ["--drop-floor", str(drop[0]), str(drop[1])] + (["--dry-run"] if dry else []) + extra
    done = subprocess.run(command, capture_output=True, text=True, timeout=180)
    sys.stdout.write(done.stdout)
    if done.returncode == 0:
        return True, ""
    lines = [line.strip() for line in (done.stdout + done.stderr).splitlines() if line.strip()]
    refused = lines.index("REFUSED:") + 1 if "REFUSED:" in lines else len(lines) - 1
    return False, " ".join(lines[refused:refused + 2])[:300] if lines else "sesame_pickup.py gave no output"


def carry(request, extra):
    reason = "no drop point was offered"
    for drop in request["drops"]:
        ok, reason = pickup(drop, extra, dry=True)
        if not ok:
            print(f"  drop ({drop[0]}, {drop[1]}) cm cannot be planned: {reason}")
            # A Sesame that is out of reach stays out of reach for every drop point.
            if "grip point" in reason or "no camera" in reason:
                break
            continue
        print(f"  carrying the Sesame to ({drop[0]}, {drop[1]}) cm on the floor")
        ok, reason = pickup(drop, extra, dry=False)
        return {"id": request["id"], "ok": ok, "reason": reason, "drop": drop}
    return {"id": request["id"], "ok": False, "reason": reason}


def main():
    args = sys.argv[1:]
    extra = args[args.index("--") + 1:] if "--" in args else []
    args = args[:args.index("--")] if "--" in args else args
    bridge = (args[0] if args else "http://localhost:8080").rstrip("/")
    print(f"waiting for carry requests from {bridge} (Ctrl-C to stop)")
    answered = 0
    while True:
        try:
            request = ask(f"{bridge}/carry")
            if request.get("id", 0) > answered:
                print(f"carry request {request['id']}: {len(request['drops'])} drop points offered")
                answer = carry(request, extra)
                answered = request["id"]
                print(f"  {'done' if answer['ok'] else 'refused: ' + answer['reason']}")
                ask(f"{bridge}/carry", answer)
        except OSError as error:
            print(f"bridge not reachable: {error}")
            time.sleep(4)
        time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
