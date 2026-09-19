"""Quick SO-101 follower health check.

Run: .venv/bin/python check_arm.py [port] [--halfway] [--release]

Default: ping the motors and print each one's position and voltage.
--halfway: initial motion test. Slews every joint to the midpoint of its
           calibrated range, checks it arrived, holds for a few seconds, then
           slews back to the starting pose and checks that too.
           Exit code 0 on pass, 1 on fail.
--release: disable torque after the test (arm goes limp, so hold it).
"""
import argparse
import json
import sys
import time
from pathlib import Path

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode

DEFAULT_PORT = "/dev/tty.usbmodem5AE60798501"
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
CALIB = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/follower.json"

# Halfway test settings.
DURATION_S = 3.0   # time to slew from current pose to the midpoint
FPS = 30           # goal updates per second during the slew
SETTLE_S = 0.5     # wait after the last goal before reading back
HOLD_S = 3.0       # pause at the midpoint before returning to the start pose
TOL_TICKS = 40     # pass if within this many encoder ticks (~3.5 deg of 4096/turn)


def halfway_targets() -> tuple[dict[str, int], str]:
    """Midpoint of each joint's calibrated range, or encoder center if uncalibrated."""
    if CALIB.exists():
        cal = json.loads(CALIB.read_text())
        targets = {n: (cal[n]["range_min"] + cal[n]["range_max"]) // 2 for n in NAMES}
        return targets, f"midpoint of calibrated range from {CALIB}"
    return {n: 2048 for n in NAMES}, "no calibration file found, using encoder center 2048"


def slew(bus: FeetechMotorsBus, start: dict[str, int], targets: dict[str, int], names: list[str]) -> dict[str, int]:
    """Linearly move `names` from `start` to `targets` and return the positions read back."""
    steps = max(1, int(DURATION_S * FPS))
    for i in range(1, steps + 1):
        t = i / steps
        goal = {n: round(start[n] + (targets[n] - start[n]) * t) for n in names}
        bus.sync_write("Goal_Position", goal, normalize=False)
        time.sleep(1 / FPS)
    time.sleep(SETTLE_S)
    end = bus.sync_read("Present_Position", names, normalize=False)
    return {n: int(end[n]) for n in names}


def report(label: str, start: dict[str, int], end: dict[str, int], targets: dict[str, int], missing: list[str]) -> bool:
    ok = True
    print(f"  {label}:")
    for n in NAMES:
        if n in missing:
            print(f"    {n:14s} FAIL (missing)")
            ok = False
            continue
        err = abs(end[n] - targets[n])
        passed = err <= TOL_TICKS
        ok &= passed
        print(f"    {n:14s} {start[n]:4d} -> {end[n]:4d}  target {targets[n]:4d}  "
              f"err {err:3d}  {'PASS' if passed else 'FAIL'}")
    return ok


def test_halfway(bus: FeetechMotorsBus, present: dict[str, int], missing: list[str]) -> bool:
    targets, source = halfway_targets()
    print(f"\nHalfway test: {source}")
    if missing:
        print(f"  skipping missing motors: {missing}")
    names = [n for n in NAMES if n not in missing]
    if not names:
        print("  FAIL: no motors to move")
        return False

    bus.enable_torque(names)
    mid = slew(bus, present, targets, names)
    ok = report("to midpoint", present, mid, targets, missing)

    print(f"  holding {HOLD_S:.0f}s")
    time.sleep(HOLD_S)

    back = slew(bus, mid, present, names)
    ok &= report("back to start", mid, back, present, missing)

    print("  RESULT:", "PASS" if ok else "FAIL")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default=DEFAULT_PORT)
    ap.add_argument("--halfway", action="store_true", help="move every joint to its midpoint and verify")
    ap.add_argument("--release", action="store_true", help="disable torque when done (arm goes limp)")
    args = ap.parse_args()

    motors = {n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100) for i, n in enumerate(NAMES, 1)}
    bus = FeetechMotorsBus(port=args.port, motors=motors)
    bus.port_handler.openPort()
    print(f"Opened {args.port}")

    found = None
    for baud in (1_000_000, 500_000, 250_000, 115_200):
        bus.set_baudrate(baud)
        found = bus.broadcast_ping()
        if found:
            print(f"Motors at {baud} baud: {found}")
            break
    if not found:
        print("No motors responded. Check the 12V supply, the power switch, and the servo cable into the board.")
        bus.port_handler.closePort()
        return 1

    present: dict[str, int] = {}
    missing: list[str] = []
    for name, m in motors.items():
        if m.id in found:
            pos = int(bus.read("Present_Position", name, normalize=False))
            v = bus.read("Present_Voltage", name, normalize=False) / 10
            present[name] = pos
            print(f"  {name:14s} id={m.id} pos={pos:4d} {v:.1f}V")
        else:
            missing.append(name)
            print(f"  {name:14s} id={m.id} MISSING")

    ok = True
    if args.halfway:
        try:
            ok = test_halfway(bus, present, missing)
        finally:
            if args.release:
                bus.disable_torque()
                print("  torque released")
            else:
                print("  torque left on; arm holds the start pose (use --release to let it go limp)")

    bus.port_handler.closePort()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
