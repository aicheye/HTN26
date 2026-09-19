# Camera / backend handoff

## 1. The contract

- Types: [src/types/world.ts](src/types/world.ts)
- Example frame: [src/data/sampleWorldState.json](src/data/sampleWorldState.json)

Every WebSocket message is an `Envelope`:

- backend -> frontend: `{ "type": "state", "data": WorldState }` and `{ "type": "ack", "data": Ack }`
- frontend -> backend: `{ "type": "command", "data": Command }`

Emit `state` at 10–30 Hz. Reply to each `command` with an `ack` carrying the same `id` as `commandId`.

## 2. Coordinate conventions

- Units: meters, angles in radians, time in ms since epoch.
- Origin `(0, 0)` = arena bottom-left corner (the corner tag you pick as origin).
- `+x` to the right, `+y` "up" / away from the origin corner, right-handed, viewed top-down.
- `yaw = 0` means the robot faces `+x`; yaw increases counter-clockwise, range `[-pi, pi]`.
- `arena.width` is the x extent, `arena.length` is the y extent.
- Robot `x, y` is the geometric center of the footprint, not the tag corner.
- Rect obstacles: `width` is the x extent, `length` the y extent, both before applying `yaw`.
- Polygon obstacles: `points` are absolute world coordinates, `x, y` is still the centroid.
- If a tag is not visible this frame, keep the last pose but set `tracking: false` and leave `lastSeen` at the previous detection time. The UI greys the robot out.

## 3. Integration

1. Expose a WebSocket that emits `WorldState` and accepts `Command`.
2. Create `.env.local` with:
   ```
   VITE_SOURCE=ws
   VITE_WS_URL=ws://<host>:<port>/ws
   ```
3. Or flip the `source` toggle in Settings at runtime - the WebSocket URL field right
   below it also updates live (persisted to `localStorage`, no rebuild needed), so a
   teammate can point the same build at a different bridge host without touching `.env`.

[src/sources/WebSocketSource.ts](src/sources/WebSocketSource.ts) implements the same
`StateSource` interface as the mock generator. It drops (and `console.warn`s) any state
frame missing `arena.width/length` or the `robots`/`obstacles` arrays, instead of letting
a malformed frame crash the renderer - a bridge restart mid-write is expected to just
skip a frame, not take down the UI. Commands that haven't received an ack within 4s are
marked `timed out` in the command log (still just a UI hint - there's no retry).

## 4. Commands the frontend sends

| type | fields | expected behavior |
| --- | --- | --- |
| `forward` / `backward` | `speed`, `face` | drive along/against yaw until `stop` |
| `left` / `right` | `speed`, `face` | rotate CCW / CW until `stop` |
| `stop` | — | halt immediately, clear goal |
| `goto` | `target` | drive to `target`, optionally publish `path` |
| `pose` | `pose`, `face` | play a one-shot animation |
| `face` | `face` | update the OLED face only, no movement |

Held keys send one command on press and a `stop` on release, so commands are latched,
not per-frame pulses.

## 5. Mapping onto the Sesame firmware

Reference: [dorianborian/sesame-robot](https://github.com/dorianborian/sesame-robot),
`firmware/README.md`. The firmware speaks HTTP, not WebSocket:

- `GET /api/status` -> `{ currentCommand, currentFace, networkConnected, apIP, networkIP }`
- `POST /api/command` <- `{ "command": "forward", "face": "walk" }`
- Movement commands loop until `{"command":"stop"}`, which matches our hold-to-move model.
- Poses are one-shot: `rest, stand, wave, dance, swim, point, pushup, bow, cute, freaky,
  worm, shake, shrug, dead, crab`.
- Faces include `walk, rest, stand, dance, wave` plus `happy, sad, angry, surprised,
  sleepy, love, excited, confused, thinking` and their `talk_*` variants.

The translation lives in [src/robot/sesameApi.ts](src/robot/sesameApi.ts). `goto` has no
firmware equivalent — the navigation layer must turn it into `forward`/`left`/`right`
pulses using the camera pose.

To drive the real robot straight from this UI while still using mock vision, set
`VITE_ROBOT_URL=http://sesame-robot.local`. Every command is then also POSTed to the
firmware. Note the firmware is plain HTTP with no auth, so serve the UI over HTTP on the
same LAN (a browser on an HTTPS page will block the mixed-content request).

## 6. The reference bridge (`bridge/`, `pi/`)

`bridge/bridge.mjs` is a real implementation of this contract - it bridges the Pi tracker
(TCP, cm/degrees) and the robot firmware (WebSocket) into `ws://localhost:8080/ws` in this
file's exact schema. `bridge/sim.mjs` fakes both of those (`cd bridge && npm run sim`) so
the frontend can be developed against a live, moving robot with no hardware at all.

Fields the bridge sends are sparser than the mock's sample data - the frontend has to
degrade gracefully rather than assume every optional field is present:

- `robots[].confidence` is never sent (mock-only). `calibration.reprojectionError` is
  never sent either - only `calibration.ok`.
- `obstacles[].height` is never sent for CV-detected boxes (no depth info from one
  camera) - height-dependent logic (e.g. "too tall to climb") only runs in `MockSource`,
  never against live bridge data.
- There is **no `arm` field at all** - the real arm has no joint telemetry, only a
  tracked ArUco tag on its base, reported as a plain circle obstacle
  (`id: "so101-base"`). `WebSocketSource.withArm()` turns that into a resting `ArmState`
  (folded rest-pose joints, `mode: "idle"`) so the articulated 3D model still renders -
  it snaps the mount to the exact center of whichever of the arena's two longer edges
  the tag is nearest to, since the physical mount is fixed there and the tracked position
  is noisy. It will never show the pick-and-place animation live; that's a `MockSource`-only
  demo until the bridge reports real joint angles.
- `robot.mode` can be `"lost"` (bridge-only value, no camera lock at all) in addition to
  `!robot.tracking` (stale last-known pose). Telemetry shows both, plus a live
  "last seen Xs ago" instead of a raw timestamp.
