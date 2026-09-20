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
                       "arm": None if not s.get("arm") or "x" not in s["arm"] else {k: s["arm"][k] for k in ("x", "y", "z", "heading")},
                       "zUp": s.get("zUp", True), "unit": unit, "floorMarkers": s.get("floorMarkers", 0), "floor": s.get("floor"), "age": 0.0}
                self.last_seen[unit] = (now, obs)
            elif unit in self.last_seen and now - self.last_seen[unit][0] <= self.HOLD_S:
                obs = dict(self.last_seen[unit][1]); obs["age"] = now - self.last_seen[unit][0]
                if s.get("arm") and "x" in s["arm"]:                       # the arm tag is usually seen; keep it fresh
                    obs["arm"] = {k: s["arm"][k] for k in ("x", "y", "z", "heading")}
            else:
                continue
            key = (obs["age"] == 0.0, obs["floorMarkers"], 1 if obs["arm"] else 0)
            if best is None or key > best[0]:
                best = (key, obs)
        return None if best is None else best[1]

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


def open_camera_page(host=None, unit=3):
    """Pop up Sean's live camera page (pi/client/demo.html) for one camera in the browser. Serves pi/client on
    localhost:5500 once. Does nothing if pi/client is not in this checkout."""
    global _page_server
    import subprocess
    import webbrowser
    client = os.path.join(HERE, "pi", "client")
    if not os.path.isfile(os.path.join(client, "demo.html")):
        print("   (Sean's camera page is not in this checkout: pi/client missing)")
        return False
    if _page_server is None:
        _page_server = subprocess.Popen([sys.executable, "-m", "http.server", "5500", "--bind", "127.0.0.1"], cwd=client,
                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.5)
    url = f"http://localhost:5500/demo.html?host={host or default_host()}&unit={unit}"
    print(f"   camera view: {url}")
    webbrowser.open(url)
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
