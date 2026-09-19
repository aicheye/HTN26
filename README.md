# HTN

## Arm: grip the Sesame anywhere

Setup once on a fresh clone (needs [uv](https://docs.astral.sh/uv/)):

```
sh setup.sh                          # .venv, LeRobot with the Feetech and kinematics extras, scipy, pytest
.venv/bin/python test_pickup.py      # the whole pick-and-place chain, offline: no arm, no Pi
```

The arm's motor calibration is in `calibration/so_follower/`, so any laptop drives this arm. The serial port
is found automatically (or set `SO101_PORT`). The Pi tracker and its scripts are on `origin/devel/sean`;
merge or check out that branch to run `start_trackers.sh`.

At the venue, on the robot WiFi:

```
sh start_trackers.sh                                   # both cameras
.venv/bin/python calibrate_arm_frame.py                # fingertips on the Sesame's tag, 3 placements
.venv/bin/python record_demo.py grip2                  # guide the grasp by hand, g when the jaws close, q to save
.venv/bin/python sesame_pickup.py grip2                # p = plan, space = grip, lift, carry, set down, release
```

`docs/INVENTORY.md` describes everything else in the repo; `nav/README.md` the quadruped's vision navigation.
