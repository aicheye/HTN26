# Robot firmware

Firmware for the Sesame robot (Lolin S2 Mini, ESP32-S2). Based on
[dorianborian/sesame-robot](https://github.com/dorianborian/sesame-robot), Apache 2.0, see `LICENSE`.
Changes from upstream: PlatformIO project, a WebSocket control API, and readback of commanded servo angles.

## Flash

Needs [PlatformIO](https://platformio.org/install). From this directory:

```
pio run -t upload
```

If the port is not found, hold `0`, tap `RST`, release `0`, and upload again.

## Connect

Join the robot's WiFi: `Sesame-Controller`, password `12345678`. The robot is at `192.168.4.1`.
This network has no internet access.

### Joining another network

The robot can join a second network, such as a phone hotspot, and keeps its own access point as well. The network
must offer 2.4 GHz. From the repo root, on the robot's WiFi: `sh pi/robot-join-wifi.sh "<name>" "<password>"`.

| Request | Effect |
|---|---|
| `POST /api/wifi/connect` with form fields `ssid`, `password`, and optionally `remember=1` | Joins the network. With `remember=1` the network is stored once it has connected, joined at every boot, and retried once a minute while it is out of range and nobody is on the access point. Without it the network is forgotten on restart. |
| `GET /api/wifi/status` | `connected`, `ssid`, `ip`, `host`, and `remembered` (the stored network name, when there is one) |
| `POST /api/wifi/forget` | Drops the stored network. The current connection stays up until the next restart. |

The settings page of the captive portal never sends `remember`, so a password entered there is not stored.

## WebSocket API

`ws://192.168.4.1:81`, JSON text messages.

Send any of these keys:

```json
{"servos": {"R1": 100, "L2": 45}}
{"command": "forward"}
{"command": "stop"}
{"face": "happy"}
```

- Servo names: `R1 R2 R3 R4 L1 L2 L3 L4`. Angles are 0 to 180.
- Commands: `forward backward left right rest stand wave dance swim point pushup bow cute freaky worm shake shrug dead crab stop`.
- Sending `servos` stops a running gait. If poses arrive faster than they can be written, only the newest is applied.

The robot sends its state on connect and on every change, at most once per 50 ms:

```json
{"command":"forward","face":"happy","servos":{"R1":135,"R2":45,"L1":45,"L2":135,"R4":0,"R3":180,"L3":0,"L4":180}}
```

- **`servos` holds the last commanded angles, not measured ones.** The servos have no feedback wire.
- A servo reads `null` until its first write after boot. Send `{"command":"stand"}` at startup to set all 8.
- Invalid input gets `{"error":"..."}`.

Each servo write waits `motorCurrentDelay` (20 ms) to limit current draw, so a full 8-servo pose takes about 160 ms.
Change it at runtime with `curl "http://192.168.4.1/setSettings?motorCurrentDelay=5"`.

The upstream HTTP API on port 80 (`/cmd`, `/api/command`, `/api/status`) and the captive portal still work.

## Test

On the robot's WiFi, with Node 22 or newer:

```
node test-websocket.mjs
```

It moves servo R1 by 10 degrees and back, and checks state messages and error replies.
