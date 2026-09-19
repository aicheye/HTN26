"""Scans the arena for objects and sends them to the bridge, which passes them to the frontend as obstacles.

    vision/.venv/bin/python vision/scan.py                       live, once: pulls frames from the tracker on the Pi
    vision/.venv/bin/python vision/scan.py --watch               live, repeating: the frontend updates every few seconds
    vision/.venv/bin/python vision/scan.py recordings/rec-006    from a recording

Live modes need the tracker (sh pi/live.sh). Move the camera slowly over the arena while it collects frames: the
change of viewpoint is what reveals objects with height.
Each object is sent with its oriented box, outline, mean colour, a rough name, and a top-down texture (PNG with
transparency outside the outline), so a 3D view can show what is really there.
"""
import base64
import collections
import json
import os
import sys
import threading
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import detect  # noqa: E402
from sam import Sam  # noqa: E402

HOST_FILE = Path(__file__).parent.parent / "pi" / "host"  # optional: the Pi's address when its name does not resolve
PI_HOST = HOST_FILE.read_text().strip() if HOST_FILE.exists() else "qnxpi78.local"
TRACKER = os.environ.get("TRACKER_URL", f"http://{PI_HOST}:8003")
BRIDGE = os.environ.get("BRIDGE_URL", "http://localhost:8080")
FRAMES, FETCH_INTERVAL_S, FETCH_WIDTH = 12, 0.3, 960
WATCH_WINDOW_S = 4  # watch mode only uses frames this recent, so a moved object wins the vote within seconds
SAME_OBJECT_CM = 4  # an object found within this distance of one from the previous scan keeps its id


def fetch_frame():
    """One frame with the tracker state of that same frame, which travels in the X-State header.
    Returns None when the camera pose is not good enough to use."""
    with urllib.request.urlopen(f"{TRACKER}/frame.jpg?w={FETCH_WIDTH}", timeout=10) as response:
        state = json.loads(response.headers["X-State"])
        image = cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)
    if not (state.get("camera") and state["floorMarkers"] >= 3):
        return None
    # The camera pose is in full-frame pixels. Scale it to the size that was fetched.
    k = image.shape[1] / state["frame"][0]
    state["camera"] = {**state["camera"], "f": state["camera"]["f"] * k, "cx": state["camera"]["cx"] * k, "cy": state["camera"]["cy"] * k}
    if state.get("robot"):
        state["robot"]["px"] = [v * k for v in state["robot"]["px"]]
    return image, state


def recorded_frames(folder):
    frames = []
    for line in (folder / "states.jsonl").read_text().splitlines():
        entry = json.loads(line)
        if entry["state"].get("camera") and entry["state"]["floorMarkers"] >= 3:
            frames.append((cv2.imread(str(folder / entry["file"])), entry["state"]))
    return frames[:: max(1, len(frames) // FRAMES)][:FRAMES]


def name_of(obj):
    r, g, b = obj["colour"]
    hue, sat, val = (int(v) for v in cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)[0, 0])
    if sat < 50:
        colour = "white" if val > 170 else "black" if val < 70 else "grey"
    else:
        colour = next(name for limit, name in [(8, "red"), (20, "orange"), (34, "yellow"), (85, "green"), (130, "blue"), (165, "purple"), (181, "red")] if hue < limit)
    return f"{colour} {'box' if obj['fill'] >= 0.8 else 'object'}"


def texture_of(obj, picture, mask, view, mirrored=False):
    """The object cut out of the top view and turned upright: image x runs along the box's width."""
    corners = np.float32([view.to_view(x, y) for x, y in obj["corners"]])
    w, h = max(8, int(obj["width"] * detect.PX_PER_CM)), max(8, int(obj["length"] * detect.PX_PER_CM))
    # corners run 0 -> 1 along the width and 1 -> 2 along the length. Row 0 of the texture is the far side (+length).
    matrix = cv2.getPerspectiveTransform(corners, np.float32([[0, h], [w, h], [w, 0], [0, 0]]))
    rgba = np.dstack([cv2.warpPerspective(picture, matrix, (w, h)), cv2.warpPerspective(mask * 255, matrix, (w, h))])
    if mirrored:  # the frontend frame flips y for such a floor, which flips the box's own y axis as well
        rgba = cv2.flip(rgba, 0)
    return base64.b64encode(cv2.imencode(".png", rgba)[1]).decode()


def to_obstacles(objects, masks, picture, view, state, previous):
    """Objects in the tracker's floor frame (cm) -> obstacles in the frontend's frame (metres, y up, yaw
    counter-clockwise, mirrored when zUp is false). An object close to one from the previous scan keeps that id."""
    mirrored, floor_h = state.get("zUp") is False, state["floor"][1]
    to_world = lambda x, y: {"x": round(x / 100, 4), "y": round((floor_h - y if mirrored else y) / 100, 4)}
    obstacles, taken = [], set()
    for obj, mask in zip(objects, masks):
        name = name_of(obj)
        position = to_world(obj["x"], obj["y"])
        match = min((o for o in previous if o["id"] not in taken), default=None,
                    key=lambda o: np.hypot(o["x"] - position["x"], o["y"] - position["y"]))
        if match and np.hypot(match["x"] - position["x"], match["y"] - position["y"]) * 100 < SAME_OBJECT_CM:
            identity = match["id"]
        else:
            number = 1 + max([int(o["id"].rsplit("-", 1)[1]) for o in previous + obstacles], default=0)
            identity = f"{name.replace(' ', '-')}-{number}"
        taken.add(identity)
        obstacles.append({
            "id": identity, "label": name, "source": "cv", "shape": "rect", **position,
            "yaw": round(-obj["yaw"] if mirrored else obj["yaw"], 4),
            "width": round(obj["width"] / 100, 4), "length": round(obj["length"] / 100, 4),
            "points": [to_world(x, y) for x, y in obj["outline"]],
            "color": "#%02x%02x%02x" % tuple(obj["colour"]), "colorSource": "camera", "confidence": obj["score"],
            "texture": texture_of(obj, picture, mask, view, mirrored),
        })
    return obstacles


def send(obstacles):
    request = urllib.request.Request(f"{BRIDGE}/obstacles", json.dumps(obstacles).encode(), {"Content-Type": "application/json"})
    urllib.request.urlopen(request, timeout=5).read()


def scan(frames, sam, previous, memory=None):
    started = time.time()
    objects, picture, _, view, masks = detect.detect(frames, tuple(frames[0][1]["floor"]), sam, return_masks=True, memory=memory)
    obstacles = to_obstacles(objects, masks, picture, view, frames[0][1], previous)
    print(f"{len(obstacles)} objects from {len(frames)} frames in {time.time() - started:.1f} s: " + ", ".join(o["id"] for o in obstacles))
    return obstacles, detect.draw(objects, picture, view)


def watch(sam):
    """A background thread fetches frames all the time. The loop rescans as fast as it can, using only the frames of
    the last few seconds, and asks SAM only about what changed since the previous scan."""
    window = collections.deque(maxlen=40)  # (arrival time, image, state)

    def fetch_forever():
        while True:
            try:
                frame = fetch_frame()
                if frame:
                    window.append((time.time(), *frame))
            except OSError as error:
                print(f"tracker not reachable: {error}", flush=True)
                time.sleep(2)
            time.sleep(FETCH_INTERVAL_S)

    threading.Thread(target=fetch_forever, daemon=True).start()
    previous, memory, last_newest = [], {}, 0
    while True:
        recent = [entry for entry in list(window) if time.time() - entry[0] <= WATCH_WINDOW_S]
        if len(recent) < 3 or recent[-1][0] == last_newest:
            if not recent:
                print("waiting for frames with 3 or more floor markers in view", flush=True)
                time.sleep(1.5)
            time.sleep(0.1)
            continue
        last_newest = recent[-1][0]
        rate = (len(recent) - 1) / max(recent[-1][0] - recent[0][0], 1e-6)
        previous, picture = scan([(image, state) for _, image, state in recent[-FRAMES:]], sam, previous, memory)
        print(f"  frames arrive at {rate:.1f} per second, newest is {time.time() - last_newest:.1f} s old", flush=True)
        cv2.imwrite("objects.jpg", picture)
        try:
            send(previous)
        except OSError:
            print("  bridge not reachable, objects not shown. Start it with sh pi/live.sh or npm --prefix bridge start", flush=True)


def main():
    args = [a for a in sys.argv[1:] if a != "--watch"]
    sam = Sam()
    if "--watch" in sys.argv:
        return watch(sam)
    source = Path(args[0]) if args else None
    if source:
        frames = recorded_frames(source)
    else:
        frames = []
        while len(frames) < FRAMES:
            frame = fetch_frame()
            frames += [frame] if frame else []
            print(f"\rframe {len(frames)} of {FRAMES}. Keep moving the camera slowly, with 3 or more floor markers in view", end="", flush=True)
            time.sleep(FETCH_INTERVAL_S)
        print()
    obstacles, picture = scan(frames, sam, [])
    out = source or Path(".")
    cv2.imwrite(str(out / "objects.jpg"), picture)
    try:
        send(obstacles)
        print(f"sent to the bridge at {BRIDGE}")
    except OSError as error:
        (out / "obstacles.json").write_text(json.dumps(obstacles))
        print(f"bridge not reachable ({error}). Saved {out / 'obstacles.json'} instead")


if __name__ == "__main__":
    main()
