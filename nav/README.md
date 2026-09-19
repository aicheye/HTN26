# nav: vision navigation for the quadruped on the tabletop arena

Python, one file per pipeline stage. Units are centimetres and degrees everywhere.

| File | Stage |
|---|---|
| `geometry.py` | tag detection (`DICT_4X4_50`, tuned adaptive threshold), one-time joint calibration of the camera pose and the corner-tag layout, frozen pose, arena frame, robot pose with the `robot.z` health metric, ground-plane homographies, rectified top-down view at 0.2 cm/px |
| `segment.py` | chroma-only (Lab a, b) obstacle segmentation with a sigma-clipped Gaussian, tag and robot masks, persistence counters, components; robot radius measured at boot |
| `planner.py` | padded 1 cm costmap, inflation by the robot radius plus one cell, soft wall cost, cost-to-go field by Dijkstra from the goal, `nearest_free`, `pick_goal` inside the start's region, steepest-descent path |
| `overlay.py` | the live window: every stage side by side plus a text corner |
| `run.py` | the loop: `--replay` a recording, `--live` the Pi, `--record` an mp4, `--headless`, `--summary` |
| `verify.py` | runs clips through the same `Pipeline` and checks the brief's thresholds |
| `synth.py` | synthetic recordings in the Pi's format, with ground truth, for work without hardware |

Not built yet: robot control (stage 5), the policy (6) and the state machine with commands (7). `run.py` shows
the state as a label and never sends a command.

## Run

```
uv pip install --python .venv/bin/python scipy            # opencv and numpy come with lerobot
.venv/bin/python -m nav.synth                              # ~70 s: recordings/synth-{empty,obstacles,hand,lights,driving}
.venv/bin/python -m nav.run --replay recordings/synth-obstacles            # live window, paced at the recorded rate
.venv/bin/python -m nav.run --replay recordings/synth-driving --record out.mp4 --headless --fast
.venv/bin/python -m nav.verify recordings/synth-*                          # all checks, exit 1 on any FAIL
```

Real clips: `bash pi/record.sh`, `bash pi/pull-recordings.sh`, then the same commands on `recordings/rec-NNN`.
For `verify.py` on a real clip pass `--board-cm W H` (tape measure between the tag centres) and
`--obstacles "x,y;x,y"` (arena cm, origin at tag 1's centre).

## What the numbers mean

- `reproj` is the RMS reprojection error of the frozen camera pose over the 16 corner-tag corners.
- `robot.z std` is the spread of the robot tag's measured height. The tag is at one height, so this is the
  health metric for the whole geometry stack; above 0.3 cm, fix the geometry before trusting anything else.
- `board` is the rectangle spanned by the four tag centres, derived, not typed in. The plywood is one tag
  edge (8 cm) bigger in each direction when the tags are flush with the corners.
- `free %` is the share of the arena the robot can be planned through after inflation. With an 8 cm robot
  half-diagonal on a 55 cm tag rectangle the interior is only 33 cm across, so two small obstacles leave a
  corridor and three do not.
