"""Scans the arena for objects and sends them to the bridge, which passes them to the frontend as obstacles.

    vision/.venv/bin/python vision/scan.py                       live: pulls frames from the tracker on the Pi
    vision/.venv/bin/python vision/scan.py recordings/rec-006    from a recording

Live mode needs the tracker (sh pi/live.sh) and the laptop on the Sesame-Controller WiFi. Move the camera slowly
over the arena while it collects frames: the change of viewpoint is what reveals objects with height.
Each object is sent with its oriented box, outline, mean colour, a rough name, and a top-down texture (PNG with
transparency outside the outline), so a 3D view can show what is really there.
"""
import base64
import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import detect  # noqa: E402

TRACKER = "http://qnxpi78.local:8003"
BRIDGE = "http://localhost:8080"
LIVE_FRAMES, LIVE_INTERVAL_S, LIVE_WIDTH = 16, 0.6, 1152


def live_frames():
    """Pulls frames with their tracker state. The state travels in the X-State header of the same response."""
    frames = []
    while len(frames) < LIVE_FRAMES:
        with urllib.request.urlopen(f"{TRACKER}/frame.jpg?w={LIVE_WIDTH}", timeout=10) as response:
            state = json.loads(response.headers["X-State"])
            image = cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)
        if state.get("camera") and state["floorMarkers"] >= 3:
            # The camera pose is in full-frame pixels. Scale it to the size that was fetched.
            k = image.shape[1] / state["frame"][0]
            state["camera"] = {**state["camera"], "f": state["camera"]["f"] * k, "cx": state["camera"]["cx"] * k, "cy": state["camera"]["cy"] * k}
            if state.get("robot"):
                state["robot"]["px"] = [v * k for v in state["robot"]["px"]]
            frames.append((image, state))
            print(f"\rframe {len(frames)} of {LIVE_FRAMES}, keep moving the camera slowly", end="", flush=True)
        else:
            print("\rwaiting for 3 or more floor markers in view            ", end="", flush=True)
        time.sleep(LIVE_INTERVAL_S)
    print()
    return frames


def recorded_frames(folder):
    frames = []
    for line in (folder / "states.jsonl").read_text().splitlines():
        entry = json.loads(line)
        if entry["state"].get("camera") and entry["state"]["floorMarkers"] >= 3:
            frames.append((cv2.imread(str(folder / entry["file"])), entry["state"]))
    return frames[:: max(1, len(frames) // 25)]


def name_of(obj):
    r, g, b = obj["colour"]
    hue, sat, val = cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)[0, 0]
    if sat < 50:
        colour = "white" if val > 170 else "black" if val < 70 else "grey"
    else:
        colour = next(name for limit, name in [(8, "red"), (20, "orange"), (34, "yellow"), (85, "green"), (130, "blue"), (165, "purple"), (181, "red")] if hue < limit)
    return f"{colour} {'box' if obj['fill'] >= 0.8 else 'object'}"


def texture_of(obj, picture, mask, view):
    """The object cut out of the top view and turned upright: image x runs along the box's width."""
    corners = np.float32([view.to_view(x, y) for x, y in obj["corners"]])
    w, h = max(8, int(obj["width"] * detect.PX_PER_CM)), max(8, int(obj["length"] * detect.PX_PER_CM))
    # corners run 0 -> 1 along the width and 1 -> 2 along the length. Row 0 of the texture is the far side (+length).
    target = np.float32([[0, h], [w, h], [w, 0], [0, 0]])
    matrix = cv2.getPerspectiveTransform(corners, target)
    rgba = np.dstack([cv2.warpPerspective(picture, matrix, (w, h)), cv2.warpPerspective(mask * 255, matrix, (w, h))])
    return base64.b64encode(cv2.imencode(".png", rgba)[1]).decode()


def main():
    source = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    frames = recorded_frames(source) if source else live_frames()
    from sam import Sam
    started = time.time()
    objects, picture, _, view, masks = detect.detect(frames, tuple(frames[0][1]["floor"]), Sam(), return_masks=True)
    print(f"{len(objects)} objects from {len(frames)} frames in {time.time() - started:.1f} s")

    # Tracker floor frame (cm) -> frontend frame (metres, y up, yaw counter-clockwise), mirrored when zUp is false.
    state = frames[0][1]
    mirrored, floor_h = state.get("zUp") is False, state["floor"][1]
    to_world = lambda x, y: {"x": round(x / 100, 4), "y": round((floor_h - y if mirrored else y) / 100, 4)}
    obstacles = []
    for i, (obj, mask) in enumerate(zip(objects, masks)):
        name = name_of(obj)
        texture = texture_of(obj, picture, mask, view)
        obstacles.append({
            "id": f"{name.replace(' ', '-')}-{i + 1}", "label": name, "source": "cv", "shape": "rect",
            **to_world(obj["x"], obj["y"]), "yaw": round(-obj["yaw"] if mirrored else obj["yaw"], 4),
            "width": round(obj["width"] / 100, 4), "length": round(obj["length"] / 100, 4),
            "points": [to_world(x, y) for x, y in obj["outline"]],
            "color": "#%02x%02x%02x" % tuple(obj["colour"]), "confidence": obj["score"], "texture": texture,
        })
        print(f"  {obstacles[-1]['id']:22s} at ({obstacles[-1]['x']:.2f}, {obstacles[-1]['y']:.2f}) m, {obj['width']:.1f} x {obj['length']:.1f} cm")
    out = source or Path(".")
    cv2.imwrite(str(out / "objects.jpg"), detect.draw(objects, picture, view))
    try:
        request = urllib.request.Request(f"{BRIDGE}/obstacles", json.dumps(obstacles).encode(), {"Content-Type": "application/json"})
        urllib.request.urlopen(request, timeout=5).read()
        print(f"sent to the bridge at {BRIDGE}")
    except OSError as error:
        (out / "obstacles.json").write_text(json.dumps(obstacles))
        print(f"bridge not reachable ({error}). Saved {out / 'obstacles.json'} instead")


if __name__ == "__main__":
    main()
