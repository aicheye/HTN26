"""Client for Sean's Pi tracker(s): where the quadruped (tag 0) and the arm base tag (tag 5) are.

The tracker runs on the Pi, one process per camera (unit 3 and unit 4), and serves its state at
http://<pi>:800<unit>/state.json (see pi/API.md). Both cameras learn the same floor markers 1-4, so both
report in the same floor frame: cm, origin at marker 1, x toward marker 2, y toward marker 4, heading in
degrees with 0 along +x. zUp false means that frame is left-handed seen from above.

    from sesame_tracker import Tracker
    t = Tracker()                         # host from pi/host, else qnxpi78.local; units 3 and 4
    obs = t.observe()                     # best single observation right now, or None
    obs = t.observe_steady(seconds=1.0)   # median over a second of polling, for a pose to act on
"""
import json
import os
import sys
import threading
import time
import urllib.request

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def default_host():
    try:
        with open(os.path.join(HERE, "pi", "host")) as f:
            return f.read().strip() or "qnxpi78.local"
    except OSError:
        return "qnxpi78.local"


ARM_TAG_HEIGHT_CM = 6.0    # tag 5 sits on top of the arm's base housing, this high above the table


def pixel_to_floor(camera, u, v, z_cm):
    """Floor point (cm) seen at pixel (u, v) at height z, from the tracker's camera pose (pi/client/floor.js)."""
    R = cv2_rodrigues(camera["rvec"])
    t = np.array(camera["tvec"], dtype=float)
    ray = np.array([(u - camera["cx"]) / camera["f"], (v - camera["cy"]) / camera["f"], 1.0])
    d = R.T @ ray
    c = -R.T @ t
    plane_z = z_cm if c[2] > 0 else -z_cm            # the floor z axis points away from the camera when zUp is false
    k = (plane_z - c[2]) / d[2]
    return float(c[0] + k * d[0]), float(c[1] + k * d[1])


def floor_to_pixel(camera, x, y, z_cm):
    """Pixel where the floor point (x, y) at height z appears, from the tracker's camera pose (floor.js)."""
    R = cv2_rodrigues(camera["rvec"])
    t = np.array(camera["tvec"], dtype=float)
    c = -R.T @ t
    p = np.array([x, y, z_cm if c[2] > 0 else -z_cm])
    v = R @ p + t
    return float(camera["cx"] + camera["f"] * v[0] / v[2]), float(camera["cy"] + camera["f"] * v[1] / v[2])


def cv2_rodrigues(rvec):
    r = np.array(rvec, dtype=float)
    a = np.linalg.norm(r)
    if a < 1e-12:
        return np.eye(3)
    k = r / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * (K @ K)


def arm_tag_pose(state, height_cm=ARM_TAG_HEIGHT_CM):
    """The arm base tag's floor pose with its x, y taken at its true height. The tracker's own x, y for tag 5
    come from its apparent size, which reads near floor level for this tag and shifts it away from the camera."""
    arm = state.get("arm")
    if not arm or "x" not in arm:
        return None
    cam = state.get("camera")
    if cam and "px" in arm:
        x, y = pixel_to_floor(cam, arm["px"][0], arm["px"][1], height_cm)
        return {"x": x, "y": y, "z": height_cm, "heading": arm["heading"]}
    return {k: arm[k] for k in ("x", "y", "z", "heading")}


class Tracker:
    HOLD_S = 10.0    # a sighting counts for this long: a small tag is missed in most frames while standing still

    def __init__(self, host=None, units=(3, 4), timeout=1.5):
        self.host = host or default_host()
        self.units = tuple(units)
        self.timeout = timeout
        self.last = {}
        self.last_seen = {}      # unit -> (time, observation) of the last frame that had the robot

    def state(self, unit):
        """The raw tracker state of one camera, or None if it cannot be reached."""
        try:
            with urllib.request.urlopen(f"http://{self.host}:{8000 + unit}/state.json", timeout=self.timeout) as r:
                s = json.loads(r.read())
            s["unit"] = unit
            s["received"] = time.time()
            self.last[unit] = s
            return s
        except Exception:
            return None

    def observe(self):
        """Best current observation: {"robot": {x, y, z, heading}, "arm": {...} or None, "zUp", "unit",
        "floorMarkers", "floor"}. Prefers the camera that sees the robot with the most floor markers."""
        best = None
        now = time.time()
        for unit in self.units:
            s = self.state(unit)
            if not s or not s.get("calibrated"):
                continue
            if s.get("robot") and "x" in s["robot"]:
                obs = {"robot": {k: s["robot"][k] for k in ("x", "y", "z", "heading")},
                       "arm": arm_tag_pose(s),
                       "zUp": s.get("zUp", True), "unit": unit, "floorMarkers": s.get("floorMarkers", 0), "floor": s.get("floor"), "age": 0.0}
                self.last_seen[unit] = (now, obs)
            elif unit in self.last_seen and now - self.last_seen[unit][0] <= self.HOLD_S:
                obs = dict(self.last_seen[unit][1]); obs["age"] = now - self.last_seen[unit][0]
                if s.get("arm") and "x" in s["arm"]:                       # the arm tag is usually seen; keep it fresh
                    obs["arm"] = arm_tag_pose(s)
            else:
                continue
            key = (obs["age"] == 0.0, obs["floorMarkers"], 1 if obs["arm"] else 0)
            if best is None or key > best[0]:
                best = (key, obs)
        return None if best is None else best[1]

    def wait_for_arm_tag(self, timeout=60.0, period=0.2, say=print):
        """Poll until a calibrated camera reports the arm base tag (id 5), then return the median of a second
        of sightings: {"x", "y", "z", "heading", "unit", "mirrored"}. None after timeout. Like the Sesame's
        tag it is 3.6 cm and reported in few frames, so this collects sightings over time."""
        end = time.time() + timeout
        sightings = []
        unit_seen, mirrored = None, False
        last_word = 0.0
        while time.time() < end:
            for u in self.units:
                s = self.state(u)
                if s and s.get("calibrated") and s.get("arm") and "x" in s["arm"]:
                    sightings.append((u, arm_tag_pose(s), not s.get("zUp", True)))
            if sightings:
                unit_seen = sightings[-1][0]
                if len(sightings) >= 5 or time.time() - end + timeout > 8 and len(sightings) >= 2:
                    break
            if say and time.time() - last_word > 5:
                say(f"   waiting for a calibrated camera to report the arm base tag (id 5)... {len(sightings)} sightings so far")
                last_word = time.time()
            time.sleep(period)
        if not sightings:
            return None
        same = [a for u, a, m in sightings if u == unit_seen]
        h = np.radians([a["heading"] for a in same])
        return {"x": float(np.median([a["x"] for a in same])), "y": float(np.median([a["y"] for a in same])),
                "z": float(np.median([a.get("z", 0.0) for a in same])),
                "heading": float(np.degrees(np.arctan2(np.median(np.sin(h)), np.median(np.cos(h))))),
                "unit": unit_seen, "mirrored": sightings[-1][2], "sightings": len(same)}

    def wait_for_robot(self, timeout=30.0, period=0.2, say=print):
        """Poll until some camera reports the Sesame (or the last sighting is still fresh), then return the
        steady median over the next second. None after timeout. The tag is missed in most frames at this
        size, so this waits patiently instead of sampling briefly."""
        end = time.time() + timeout
        told = False
        while time.time() < end:
            if self.observe() is not None:
                return self.observe_steady(1.0)
            if not told and say:
                say("   waiting for a camera to report the Sesame's tag (hold it still)...")
                told = True
            time.sleep(period)
        return None

    def observe_steady(self, seconds=1.0, period=0.15):
        """Median of the observations over a short window (position and heading), for a pose to act on."""
        obs = []
        end = time.time() + seconds
        while time.time() < end:
            o = self.observe()
            if o:
                obs.append(o)
            time.sleep(period)
        if not obs:
            return None
        out = dict(obs[-1])
        for key in ("robot", "arm"):
            vals = [o[key] for o in obs if o.get(key)]
            if not vals:
                continue
            h = np.radians([v["heading"] for v in vals])
            out[key] = {"x": float(np.median([v["x"] for v in vals])), "y": float(np.median([v["y"] for v in vals])),
                        "z": float(np.median([v["z"] for v in vals])),
                        "heading": float(np.degrees(np.arctan2(np.median(np.sin(h)), np.median(np.cos(h)))))}
        out["samples"] = len(obs)
        return out


def alive_units(tracker):
    """The camera units whose tracker answers at all, whether or not they see anything."""
    return [u for u in tracker.units if tracker.state(u)]


_page_server = None


def _port_open(port):
    import socket
    with socket.socket() as sk:
        sk.settimeout(0.3)
        return sk.connect_ex(("127.0.0.1", port)) == 0


def open_camera_page(host=None, unit=3, units=None):
    """Pop up Sean's live camera page (pi/client/demo.html) in the browser, one tab per camera. Serves
    pi/client on localhost:5500 (reusing a server already there). Prints the URLs, so they can be opened by
    hand if the browser does not come up. Does nothing if pi/client is not in this checkout."""
    global _page_server
    import subprocess
    import webbrowser
    client = os.path.join(HERE, "pi", "client")
    if not os.path.isfile(os.path.join(client, "demo.html")):
        print("   (Sean's camera page is not in this checkout: pi/client missing)")
        return False
    if not _port_open(5500):
        _page_server = subprocess.Popen([sys.executable, "-m", "http.server", "5500", "--bind", "127.0.0.1"], cwd=client,
                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        for _ in range(20):
            if _port_open(5500):
                break
            time.sleep(0.1)
    host = host or default_host()
    for u in (units or [unit]):
        url = f"http://localhost:5500/demo.html?host={host}&unit={u}"
        print(f"   camera view: {url}")
        opened = False
        if sys.platform == "darwin":
            opened = subprocess.run(["open", url], capture_output=True).returncode == 0
        if not opened:
            try:
                webbrowser.open(url)
            except Exception:
                print("   (could not open a browser: open the link above by hand)")
    return True


class Poller:
    """Keeps the latest observation fresh on a thread, so a 20 Hz loop never waits on the network."""
    def __init__(self, tracker, period=0.5):
        self.tracker, self.period = tracker, period
        self.latest, self.at = None, 0.0
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            o = self.tracker.observe()
            if o:
                self.latest, self.at = o, time.time()
            time.sleep(self.period)

    def get(self, max_age=2.0):
        return self.latest if self.latest and time.time() - self.at < max_age else None
