# Tracker API

The tracker runs on the QNX Raspberry Pi (`pi/tracker/tracker.cpp`). It reads one camera, finds ArUco markers,
and reports where the robot and the arm base are. Clients must be on the `Sesame-Controller` WiFi.

**The web frontend does not use this API directly.** It talks to `bridge/bridge.mjs`, which converts the
tracker's output into the frontend's own contract (`src/types/world.ts` and `docs/HANDOFF.md` on the
`frontend` branch: WebSocket envelopes, metres, radians). This document is for the bridge, the agent, and
debugging.

| What | Where |
|---|---|
| State, pushed once per frame (about 15 per second) | `http://qnxpi78.local:8003/events` (server-sent events, use `EventSource`) |
| Every line the tracker prints, as `log` events on the same stream (`source.addEventListener("log", ...)`). A new client first receives the last 40 lines | same `/events` URL |
| Record frames on the Pi (`pi/record.sh` drives these with one key) | `/record/start?every=2&w=0`, `/record/stop`, `/record/status` on the same port. Saved under `~/recordings/rec-NNN` with `states.jsonl` |
| The state that belongs to a fetched frame | `X-State` response header of `/frame.jpg` and `/annotated.jpg` |
| State, once | `http://qnxpi78.local:8003/state.json` |
| Camera frame | `http://qnxpi78.local:8003/frame.jpg?w=960` (`w` is optional, full size is 2304x1296) |
| Camera frame with markers drawn | `http://qnxpi78.local:8003/annotated.jpg` |
| State as plain TCP, one JSON per line (for Node or Python) | `qnxpi78.local:9003` |

Ports are `8000 + unit` and `9000 + unit`. Unit 3 is the first camera, unit 4 the second.
All HTTP responses send `Access-Control-Allow-Origin: *`. The Pi serves plain http, so a page served over
https cannot call it. Serve the frontend over http (for example `http://localhost`).
Poll `/frame.jpg` at about 2 per second. Each request pauses tracking for the time it takes to encode and send.

## State

```json
{
  "t": 5012,
  "calibrated": true,
  "frame": [2304, 1296],
  "floor": [76, 60],
  "zUp": true,
  "learned": 4,
  "cameraHeight": 131,
  "floorMarkers": 4,
  "fps": 15,
  "markers": [0, 1, 2, 3, 4, 5],
  "robot": {"x": 38.0, "y": 30.0, "z": 8.1, "heading": 91.3, "px": [1100.2, 640.8]},
  "arm":   {"x": 70.2, "y": 8.5, "z": 5.0, "heading": 180.0, "px": [1900.0, 210.4]},
  "camera": {"f": 1693.0, "cx": 1152.0, "cy": 648.0, "rvec": [0.01, 3.13, 0.02], "tvec": [-38.1, 30.2, 131.0]}
}
```

- `t`: milliseconds since the tracker started.
- `frame`: size in pixels of the full camera frame. **All pixel values in this API are in full-frame pixels**,
  also when the image was fetched with `?w=`. Multiply by `displayed width / frame[0]` to draw on a scaled image.
- `floor`: width and height in cm of the rectangle between floor markers 1 to 4.
- `learned`: how many of the four floor markers the tracker has learned so far. `cameraHeight` is in cm, -1 when unknown.
- `calibrated`: true when this frame's camera position over the floor is known. It needs at least one floor
  marker in view. `floorMarkers` is how many were used, more is steadier.
- `robot`, `arm`: `null` when that marker is not visible. `x`, `y`, `z` are cm, `heading` is degrees.
  When `calibrated` is false they hold only `px` and `headingPx`.
- `camera`: `null` when not calibrated. Used to convert pixels to floor cm, see below.

## Floor coordinates

Origin at floor marker 1. `x` grows toward marker 2, `y` toward marker 4, both in cm. `z` is height above the
floor. `heading` 0 faces +x, 90 faces +y.

## Pixels to floor

`pi/client/floor.js` converts between full-frame pixels and floor cm using the `camera` object:

```js
import { pixelToFloor, floorToPixel } from "./floor.js";
const { x, y } = pixelToFloor(state.camera, u, v);        // cm on the floor
const { u, v } = floorToPixel(state.camera, x, y, z);     // where a floor point appears in the frame
```

Use the `camera` from the same moment as the frame. If the camera moves, a box in pixels no longer lines up
with a new frame, but its floor position stays valid.

## Bridge

`bridge/bridge.mjs` runs on the laptop (`cd bridge && npm install && npm start`) and serves:

- `ws://localhost:8080/ws`: the frontend contract. `state` envelopes at 10 per second, `command` in, `ack` out.
  `goto` is handled here: turn in place until facing the target, walk forward, stop within 6 cm.
- `POST http://localhost:8080/objects`: input from the object detector, turned into `source: "cv"` obstacles.
  ```json
  {"objects": [{"label": "chocolate", "box": [1210, 540, 1330, 640], "confidence": 0.9}], "camera": {...}}
  ```
  `box` is `[left, top, right, bottom]` in full-frame pixels of the `/frame.jpg` the detector looked at.
  `camera` is optional: the tracker's `camera` object from the moment that frame was fetched.
- `GET http://localhost:8080/state`: the current `WorldState` once.

Frame conversion: metres = cm / 100, `yaw` = `heading` in radians. `zUp: false` in the tracker state means
floor markers 1 to 4 run clockwise seen from above. The bridge then mirrors `y` and negates `yaw`, so the
frontend always gets a right-handed frame with yaw counter-clockwise.

- `POST http://localhost:8080/obstacles`: obstacles in the frontend's own format (metres), replacing the previous
  list. `vision/scan.py` sends detected objects here as `source: "cv"`, `shape: "rect"`, with extras outside the
  frontend schema: `label` ("green box"), `color` ("#6ce44e", white-balanced), `points` (the outline), `confidence`,
  and `textureUrl`. The texture is a PNG cut out of the top view and turned upright: image x runs along the box's
  `width` (the `yaw` direction), row 0 is the far side along `length`, and it is transparent outside the outline.

`npm test` in `bridge/` checks all of this against a fake tracker and a fake robot.

`npm run sim` in `bridge/` starts the bridge against a simulated tracker and robot, so the frontend can be
developed without hardware. The simulated robot walks and turns in response to commands, including `goto`.

The web UI at the repository root (`npm run dev`) is the only UI. `sh pi/live.sh` starts it together with the bridge,
the object scan and the tracker.
