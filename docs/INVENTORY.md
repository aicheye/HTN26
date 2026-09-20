# Inventory: what exists before the vision-navigation build

Surveyed 2026-09-19 on `nav`, which is `origin/main` with every commit of `origin/devel/sean` replayed on
top (23 commits, nothing on Sean's branch was changed). `origin/frontend` (Angus Sun's three.js frontend and
its hardware-free 3D sim) was left out on purpose and is untouched.

## Branches

| Branch | Author | What it holds | Verdict |
|---|---|---|---|
| `main` | Arjun | `check_arm.py`, SO-101 IK (`so101_ik.py`, `so101_safe.py`, tests, URDF) | base |
| `devel/sean` | Sean Yang | Pi camera tracker (C++), Node bridge with planner, navigator, Kalman pose filter, gait engine and simulator, robot firmware (PlatformIO), Pi scripts | **the tracker base**; replayed onto `nav` |
| `frontend` | Angus Sun | React + three.js controller UI with a 2D and 3D map, WebSocket source, mock world state. Merged `devel/sean` at f9f50f7 | excluded, as asked |
| `so101` | Arjun | one solver tweak past main (PR #2 was merged) | unrelated |
| `arm-halfway-test` | Arjun | merged (PR #1) | unrelated |

No stashes.

## The existing tracker: `pi/tracker/tracker.cpp` (newest working version, Sean, today)

C++ on the QNX Raspberry Pi. It is the only ArUco + `solvePnP` implementation in the repo; there is no
duplicate to delete. It is not Python and it runs on the Pi, not the laptop, so the new stack cannot import
it. What it establishes, and what the new stack keeps compatible:

- `DICT_4X4_50`. Tag 0 = robot (3.6 cm), tag 5 = arm base (3.6 cm), tags 1-4 = floor corners (8.0 cm).
  Detection runs on a half-size frame, corners refined with `cornerSubPix` at full size. Default
  `DetectorParameters`.
- Intrinsics: `f = 1693 px` (Camera Module 3, 2304x1296 binned mode), `cx, cy` = frame centre. No distortion.
- Floor frame: origin at tag 1's centre, `x` toward tag 2, `y` toward tag 4, cm. Heading 0 faces +x,
  90 faces +y. `zUp: false` in the state means tags 1-4 run clockwise seen from above (left-handed frame);
  the bridge mirrors `y` in that case.
- **Camera pose is re-solved every frame** (`SOLVEPNP_ITERATIVE` seeded from the previous frame) from
  whichever corner tags are visible. The board size is an input argument, not derived: the tracker is
  started as `tracker <unit> <W cm> <H cm>` and uses W, H as the known centres of tags 2, 3, 4.
- Robot `z`: from the tag's apparent size it reads 1.5-5 cm with 2 cm of frame-to-frame noise
  (Sean's notes in `bridge/pose-filter.mjs`), so the tracker takes the height as an argument (10.5 cm,
  measured with a ruler) and intersects the tag-centre ray with that plane.
- Output: one JSON per frame over TCP `qnxpi78.local:9003`, SSE at `http://qnxpi78.local:8003/events`,
  `/state.json`, `/frame.jpg[?w=]` (about 2 per second, each request pauses tracking), `/annotated.jpg`,
  `/record/start|stop|status`. Full schema in `pi/API.md`.

### `states.jsonl` (recordings)

Written by the Pi under `~/recordings/rec-NNN/` next to `frame-NNNNNN.jpg` (BGR JPEG, quality 92, full
2304x1296 unless recorded with `w=`). One line per saved frame:

```json
{"file": "frame-000042.jpg", "state": {
  "t": 5012,                 // ms since the tracker started (the Pi has no clock)
  "calibrated": true,        // camera pose known this frame
  "frame": [2304, 1296],     // px
  "floor": [63, 63],         // cm, the W H the tracker was started with (NOT measured)
  "zUp": true,               // false = tags 1-4 clockwise from above
  "learned": 4, "cameraHeight": 115, "floorMarkers": 4, "fps": 15,
  "markers": [0, 1, 2, 3, 4, 5],
  "robot": {"x": 38.0, "y": 30.0, "z": 10.5, "heading": 91.3, "px": [1100.2, 640.8]},  // cm, cm, cm, deg, full-frame px; null when unseen
  "arm":   {"x": 70.2, "y": 8.5, "z": 5.0, "heading": 180.0, "px": [1900.0, 210.4]},   // same; z from apparent size
  "camera": {"f": 1693.0, "cx": 1152.0, "cy": 648.0, "rvec": [..3..], "tvec": [..3.. cm]}  // floor -> camera, pixel = K (R p + t)
}}
```

Copy recordings with `bash pi/pull-recordings.sh` (WiFi) or `pi/qnx6-extract.py` (SD card). **None are on
this laptop and the Pi is not reachable from here right now**, so this build is validated on synthetic
recordings in the same format (`nav/synth.py`) until real clips are pulled.

## The existing planner: `bridge/planner.mjs` (not `planner.py`)

The brief says `planner.py` exists with Dijkstra-from-goal, `nearest_free` and `pick_goal`. **It does not.**
What exists is `bridge/planner.mjs`: A* on an 8-connected 2 cm grid, units metres, obstacles as
circle/rect/polygon shapes (not a mask), no arena padding, goal-inside-obstacle moved to the nearest free
cell, start cell forced free, corner-only path simplification. Tested by `planner.test.mjs` (5 cases). It is
the frontend/bridge's planner and stays as is. The new `nav/planner.py` is written to the brief (cost-to-go
field from the goal, padded grid, inflation with one cell of slack, soft wall cost, `nearest_free`,
`pick_goal` inside the start's connected region) and reuses nothing from it, because nothing in it is a
field planner.

## Robot control today: `bridge/navigator.mjs`, `bridge/gait.mjs`, firmware

- Firmware is `firmware/src/main.cpp` + `firmware/src/movement-sequences.h` (PlatformIO, ESP32-S2), not
  `sesame-firmware-main.ino`. Verified behaviour: `forward/backward/left/right` run `walkCycles` cycles
  (default 10, `frameDelay` 100 ms per frame) and `loop()` restarts them while `currentCommand` is unchanged;
  `pressingCheck(cmd, frameDelay)` services HTTP and the WebSocket between frames and aborts with
  `runStandPose(1)` the moment `currentCommand` differs. So a command change lands within one frame delay.
- HTTP on port 80: `GET /cmd?go=forward`, `/cmd?stop=1`, `/api/status` (`currentCommand`),
  `/getSettings`, `/setSettings?walkCycles=200&frameDelay=100`. WebSocket `ws://192.168.4.1:81` takes
  `{"command": "..."}` and `{"servos": {...}}` (Sean's bridge and gait engine use this).
- `navigator.mjs` already does most of stage 6: hysteresis (turn above 0.45 rad, done below 0.1 rad),
  edge-triggered sends with a 2 s resend, replans every 1 s, stuck detection and back-off recovery, and a
  `calibrate()` that measures walk speed, turn rate, veer and stop leads with the camera. It works in metres
  and drives the bridge, not a 20 Hz vision loop. Its numbers are the best prior for the new policy.
- `pose-filter.mjs`: EKF over the tracker detections, cm and radians. Its measured noise figures
  (0.04 cm spread with four corner tags, 0.25-0.34 cm with two or three, heading spread 0.75-1.0 deg
  standing still) are the only real noise measurements in the repo.

## Hardcoded constants found

| Where | Constant | Value |
|---|---|---|
| `tracker.cpp` | `FOCAL_PX` | 1693 |
| `tracker.cpp` | `TRACKED_MARKER_CM`, `FLOOR_MARKER_CM` | 3.6, 8.0 (printed tag sizes: the permitted constants) |
| `tracker.cpp` | `DETECT_SHRINK`, `SAMPLES_NEEDED`, `STEADY_MAX_MOVE_PX` | 2, 8, 6 |
| `pi/live.sh` | floor W H, robot marker height | 63 63, 10.5 cm |
| `pi/run-tracker.sh` | floor W H | 100 100 |
| `bridge/sim.mjs`, `pi/API.md`, frontend sample data | arena | **76 x 60 cm** (disagrees with 63 x 63) |
| `bridge/sim.mjs` | walk, turn | 5 cm/s, 30 deg/s ("guesses") |
| `bridge/navigator.mjs` | `DEFAULT_MOTION` | 0.04 m/s, 0.5 rad/s, thresholds 0.45 / 0.1 rad |
| `bridge/planner.mjs` | `CELL_M`, `CLEARANCE_M` | 0.02, 0.10 |
| `bridge/bridge.mjs` | `ROBOT_FOOTPRINT`, `ARM_BASE_RADIUS` | 0.105 x 0.125 m, 0.09 m |
| `bridge/pose-filter.mjs` | `markerHeight` and the noise table | 10.5 cm, see file |
| firmware | `frameDelay`, `walkCycles`, `motorCurrentDelay` | 100 ms, 10, 20 ms |

The board is 63 x 63 per the brief and `pi/live.sh`; the bridge, sim and frontend still say 76 x 60. The
new stack derives it from the tags and reports it, so nothing here needs to agree.

## How it is launched today

- Camera + tracker + browser view: `sh pi/live.sh` (laptop on the `Sesame-Controller` WiFi). Builds and
  runs the tracker on the Pi over ssh, serves `pi/client/demo.html`.
- Bridge for the frontend: `cd bridge && npm start`. `npm run sim` for the hardware-free simulator.
  `npm test` runs the planner, navigator, gait, pose-filter and bridge tests against fakes.
- Walk the robot directly: `node bridge/walk.mjs forward 6`.
- Record frames: `bash pi/record.sh`, then `bash pi/pull-recordings.sh`.

## Consequences for the build

1. **Language.** The brief's pipeline is Python (`cv2.aruco`, Lab chroma, `requests`); the tracker is
   C++ on the Pi and the planner is JS. The new stack is Python in `nav/`, one file per stage, and treats
   Sean's tracker as the recording source and as a cross-check (its `robot` and `camera` fields are read
   from `states.jsonl` and compared), not as a dependency.
2. **The laptop cannot see the camera at 20 Hz today.** The Pi serves frames only via `/frame.jpg`
   at about 2 per second over the robot's slow WiFi, and each request pauses tracking. The 20 Hz loop is
   therefore built and verified in `--replay` mode against recorded clips. Live operation needs either
   a frame stream from the Pi or the vision stages running on the Pi; that is flagged, not solved, here.
3. **Board size, robot radius, walk speed and turn rate** are measured at boot, per the brief; the tracker's
   W H argument and the bridge's footprint constants are not used.
4. **Frame convention** follows the tracker (origin at tag 1, x toward tag 2, y toward tag 4, cm,
   heading in degrees) whenever the tags are laid out that way, so recorded `robot` fields compare directly.
   The frame is always right-handed with z up; a clockwise tag layout gets its x axis snapped so all corners
   stay positive, as the brief specifies, rather than the tracker's mirrored `zUp: false` frame.
