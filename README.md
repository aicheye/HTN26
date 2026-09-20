# HTN

## Arm: grip the Sesame anywhere

One command. Plug the arm in, join the robot's WiFi, and:

```
sh run.sh go             # starts the trackers, calibrates, records a demo: each only if not done yet; then grips
sh run.sh go --now       # the same, and grips the moment a camera sees the Sesame
```

It sets itself up on first use (installs uv and the Python environment if needed, about a minute). The steps
it runs for you, also available one at a time:

```
sh run.sh check          # what is ready and what is missing, with the fix for each
sh run.sh test           # the whole pick-and-place chain offline: no arm, no Pi
sh run.sh trackers       # both Pi cameras (fetches Sean's Pi files from origin/devel/sean if this checkout lacks them)
sh run.sh calibrate      # fingertips on the Sesame's tag at 3 placements
sh run.sh record grip2   # guide the grasp by hand: g when the jaws close, q to save
sh run.sh pickup grip2   # p = plan, space = find the Sesame, grip, lift, carry, set down, release
```

`sh run.sh` alone lists every command. The arm's motor calibration ships in `calibration/`, the serial port
is found automatically (or set `SO101_PORT`), the Pi's address comes from `pi/host` when its name does not
resolve. `docs/INVENTORY.md` describes everything else in the repo; `nav/README.md` the quadruped's vision
navigation.
