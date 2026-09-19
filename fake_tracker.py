"""A stand-in for the Pi tracker, for testing on the laptop: serves /state.json for units 3 and 4 on
localhost with a fixed robot and arm-tag pose in the floor frame.

    .venv/bin/python fake_tracker.py --robot 30 20 45 --arm 70 8 180 [--zup 1] [--noise 0.2]
then point the clients at it:  --tracker localhost
"""
import argparse
import json
import random
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer


def serve(robot=(30.0, 20.0, 45.0), arm=(70.0, 8.0, 180.0), zup=True, noise=0.1, units=(3, 4), host="127.0.0.1"):
    """Start the fake tracker on daemon threads and return the servers (call .shutdown() on each to stop)."""
    t0 = time.time()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if not self.path.startswith("/state.json"):
                self.send_response(404); self.end_headers(); return
            n = lambda: random.gauss(0, noise)
            state = {"t": int(1000 * (time.time() - t0)), "calibrated": True, "frame": [2304, 1296], "floor": [63, 63],
                     "zUp": bool(zup), "learned": 4, "cameraHeight": 115, "floorMarkers": 4 if self.server.unit == 3 else 3, "fps": 15,
                     "markers": [0, 1, 2, 3, 4, 5],
                     "robot": {"x": robot[0] + n(), "y": robot[1] + n(), "z": 10.5, "heading": robot[2] + 3 * n(), "px": [0, 0]},
                     "arm": {"x": arm[0] + n(), "y": arm[1] + n(), "z": 5.0, "heading": arm[2] + 3 * n(), "px": [0, 0]},
                     "camera": {"f": 1693.0, "cx": 1152.0, "cy": 648.0, "rvec": [3.14, 0, 0], "tvec": [-31, 31, 115]}}
            body = json.dumps(state).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.send_header("Content-Length", str(len(body))); self.end_headers()
            self.wfile.write(body)

    servers = []
    for unit in units:
        s = HTTPServer((host, 8000 + unit), H); s.unit = unit
        threading.Thread(target=s.serve_forever, daemon=True).start(); servers.append(s)
    return servers


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", type=float, nargs=3, metavar=("X", "Y", "HEADING"), default=[30.0, 20.0, 45.0])
    ap.add_argument("--arm", type=float, nargs=3, metavar=("X", "Y", "HEADING"), default=[70.0, 8.0, 180.0])
    ap.add_argument("--zup", type=int, default=1)
    ap.add_argument("--noise", type=float, default=0.1, help="cm of jitter on the poses")
    ap.add_argument("--units", type=int, nargs="+", default=[3, 4])
    args = ap.parse_args()
    serve(args.robot, args.arm, bool(args.zup), args.noise, args.units)
    print(f"fake tracker: units {args.units} on ports {[8000 + u for u in args.units]}, robot {args.robot}, arm {args.arm}, zUp {bool(args.zup)}")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
