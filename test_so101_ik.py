"""Validate so101_ik against LeRobot's URDF forward kinematics, then map reachability.

Run:  .venv/bin/python test_so101_ik.py      (prints a report; exit 1 on any failure)
  or  .venv/bin/python -m pytest test_so101_ik.py

Needs lerobot[kinematics] (placo) and so101_new_calib.urdf next to this file
(downloaded from TheRobotStudio/SO-ARM100 if missing). The URDF's meshes are not
needed: visual/collision elements are stripped into a temp copy before loading.
"""
import csv
import os
import sys
import tempfile
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np

from so101_ik import JOINTS, LIMITS, ik

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "so101_new_calib.urdf")
URDF_URL = "https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/Simulation/SO101/so101_new_calib.urdf"
TOL_MM = 1.0
N_ROUNDTRIP = 500
SWEEP_POS = (0.25, 0.05, 0.03)          # fixed target for the roll sweep
GRASP_Z = 0.02                          # gripper_frame height for the reachability map (m)
GRID_STEP = 0.005                       # 5 mm
GRID_HALF = 0.45                        # map covers +-45 cm in x and y


def load_fk():
    if not os.path.exists(URDF):
        print(f"downloading {URDF_URL}")
        urllib.request.urlretrieve(URDF_URL, URDF)
    tree = ET.parse(URDF)
    for link in tree.getroot().iter("link"):
        for tag in ("visual", "collision"):
            for el in link.findall(tag):
                link.remove(el)
    path = os.path.join(tempfile.mkdtemp(), "so101_nomesh.urdf")
    tree.write(path)
    from lerobot.model.kinematics import RobotKinematics
    kin = RobotKinematics(urdf_path=path, target_frame_name="gripper_frame_link", joint_names=JOINTS)
    return lambda sol: kin.forward_kinematics(np.array([sol[j] for j in JOINTS], dtype=float))


FK = None


def fk(sol):
    global FK
    if FK is None:
        FK = load_fk()
    return FK(sol)


def pos_err_mm(sol, target):
    return float(np.linalg.norm(fk(sol)[:3, 3] - np.asarray(target)) * 1000)


def within_limits(sol):
    return all(LIMITS[j][0] <= sol[j] <= LIMITS[j][1] for j in JOINTS)


SOLUTIONS = []   # every solution produced by tests 1 and 2, for the limits test


def test_roundtrip():
    """FK(ik(target)) matches the target within 1 mm for 500 random reachable top-down targets."""
    rng = np.random.default_rng(0)
    errs, orient_errs, tries = [], [], 0
    while len(errs) < N_ROUNDTRIP:
        tries += 1
        x, y = rng.uniform(-GRID_HALF, GRID_HALF, 2)
        z, yaw = rng.uniform(0.0, 0.30), rng.uniform(-180, 180)
        sol = ik(x, y, z, yaw)
        if sol is None:
            continue
        SOLUTIONS.append(sol)
        T = fk(sol)
        errs.append(np.linalg.norm(T[:3, 3] - [x, y, z]) * 1000)
        approach = np.degrees(np.arccos(np.clip(-T[2, 2], -1, 1)))          # z axis should point down
        heading = np.degrees(np.arctan2(T[1, 0], T[0, 0]))                    # x axis heading in the table plane
        dyaw = abs((heading - yaw + 90) % 180 - 90)                            # jaws are symmetric
        orient_errs.append(max(approach, dyaw))
    errs = np.array(errs)
    print(f"test 1 round-trip: {N_ROUNDTRIP} targets ({tries} sampled)  "
          f"max {errs.max():.4f} mm  rms {np.sqrt((errs**2).mean()):.4f} mm  "
          f"orientation max {max(orient_errs):.4f} deg")
    assert errs.max() < TOL_MM, f"round-trip max error {errs.max():.3f} mm exceeds {TOL_MM} mm"
    assert max(orient_errs) < 0.1, "approach axis or jaw heading is wrong"


def test_roll_sweep():
    """Position stays put while jaw_yaw sweeps -180..180. Catches a missing lateral correction (~8 mm)."""
    x, y, z = SWEEP_POS
    worst, worst_yaw, missing = 0.0, None, []
    for yaw in range(-180, 181, 10):
        sol = ik(x, y, z, yaw)
        if sol is None:
            missing.append(yaw)
            continue
        SOLUTIONS.append(sol)
        e = pos_err_mm(sol, (x, y, z))
        if e > worst:
            worst, worst_yaw = e, yaw
    print(f"test 2 roll sweep at {SWEEP_POS}: max {worst:.4f} mm at jaw_yaw={worst_yaw}"
          + (f"  unreachable yaws: {missing}" if missing else ""))
    assert not missing, f"roll sweep: unreachable at jaw_yaw {missing}"
    if worst >= TOL_MM:
        print("  FAIL: position drifts with wrist_roll. The lateral correction is wrong or missing.")
    assert worst < TOL_MM


def test_unreachable():
    """Too far, too close, and below the table all return None."""
    cases = {
        "too far (60 cm out)": (0.60, 0.0, 0.05, 0),
        "too far sideways": (0.0, 0.55, 0.05, 0),
        "too close (on the pan axis)": (0.0388, 0.0, 0.05, 0),
        "below the table": (0.25, 0.0, -0.05, 0),
        "too high for top-down": (0.25, 0.0, 0.45, 0),
    }
    bad = [name for name, args in cases.items() if ik(*args) is not None]
    print(f"test 3 unreachable: {len(cases) - len(bad)}/{len(cases)} returned None"
          + (f"  wrongly solved: {bad}" if bad else ""))
    assert not bad


def test_limits():
    """Every solution returned so far is inside the joint limits."""
    if not SOLUTIONS:
        test_roundtrip()
        test_roll_sweep()
    bad = [s for s in SOLUTIONS if not within_limits(s)]
    print(f"test 4 limits: {len(SOLUTIONS) - len(bad)}/{len(SOLUTIONS)} solutions inside limits")
    assert not bad


def reachability_map():
    """5 mm grid over the table at GRASP_Z with jaw_yaw=0 -> reachability.csv and reachability.png."""
    axis = np.arange(-GRID_HALF, GRID_HALF + GRID_STEP / 2, GRID_STEP)
    grid = np.zeros((len(axis), len(axis)), dtype=bool)
    rows = []
    for i, y in enumerate(axis):
        for j, x in enumerate(axis):
            ok = ik(x, y, GRASP_Z, 0.0) is not None
            grid[i, j] = ok
            if ok:
                rows.append((round(x, 4), round(y, 4)))
    csv_path = os.path.join(HERE, "reachability.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x_m", "y_m"])
        w.writerows(rows)
    xs = np.array([r[0] for r in rows]); ys = np.array([r[1] for r in rows])
    print(f"reachability at z={GRASP_Z} m, jaw_yaw=0: {len(rows)} of {grid.size} grid points  "
          f"x {xs.min():.3f}..{xs.max():.3f}  y {ys.min():.3f}..{ys.max():.3f}  "
          f"area {len(rows) * GRID_STEP**2 * 1e4:.0f} cm^2  -> {csv_path}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    fig, ax = plt.subplots(figsize=(6.4, 6.4), dpi=150)
    fig.patch.set_facecolor("#fcfcfb"); ax.set_facecolor("#fcfcfb")
    ext = (-GRID_HALF * 100, GRID_HALF * 100) * 2
    ax.imshow(grid, origin="lower", extent=ext, cmap=ListedColormap(["#ebeae6", "#2a78d6"]),
              vmin=0, vmax=1, interpolation="nearest")
    ax.plot(3.88, 0, marker="o", ms=6, color="#0b0b0b")
    ax.annotate("pan axis", (3.88, 0), xytext=(-8, 0), textcoords="offset points", ha="right", va="center",
                fontsize=8, color="#52514e")
    ax.set_xlabel("x (cm, base_link)", color="#52514e"); ax.set_ylabel("y (cm, base_link)", color="#52514e")
    ax.set_title(f"SO-101 top-down reach, gripper frame at z = {GRASP_Z*100:.0f} cm, jaw_yaw = 0",
                 fontsize=10, color="#0b0b0b")
    ax.legend(handles=[Patch(color="#2a78d6", label="reachable"), Patch(facecolor="#ebeae6", edgecolor="#b8b7b2", label="unreachable")],
              loc="upper left", fontsize=8, frameon=False)
    ax.tick_params(colors="#52514e", labelsize=8)
    for s in ax.spines.values():
        s.set_color("#d6d5d0")
    ax.grid(True, color="#e6e5e1", lw=0.5); ax.set_axisbelow(True); ax.set_aspect("equal")
    png_path = os.path.join(HERE, "reachability.png")
    fig.savefig(png_path, bbox_inches="tight")
    print(f"wrote {png_path}")


def main():
    failed = []
    for t in (test_roundtrip, test_roll_sweep, test_unreachable, test_limits):
        try:
            t()
        except AssertionError as e:
            failed.append(t.__name__)
            print(f"  FAIL {t.__name__}: {e}")
    print("RESULT:", "PASS" if not failed else f"FAIL ({', '.join(failed)})")
    reachability_map()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
