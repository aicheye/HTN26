"""Synthetic recordings in the Pi tracker's format (frame-NNNNNN.jpg + states.jsonl), for running and
verifying the pipeline without hardware. Every line of states.jsonl also carries a "truth" object.

    .venv/bin/python -m nav.synth [out_root] [clip names...]        default recordings/, all clips

Clips: synth-empty, synth-obstacles, synth-hand, synth-lights, synth-driving. Board 63 x 63 cm, 8 cm corner
tags 4 cm in from the edges, robot tag 3.6 cm at 10.5 cm height, camera 115 cm up with a small tilt, 2304 x 1296,
f = 1693. Rendering is layered: the floor (board, tags, shadows) is one texture warped through the z = 0
homography; every raised thing (robot body and tag, obstacle tops, the hand) is warped through the homography
of its own height, so parallax is real.
"""
import json
import os
import sys

import numpy as np
import cv2

W, H, F = 2304, 1296, 1693.0
BOARD = (63.0, 63.0)
TAG_IN = 4.0                       # corner tag centres this far in from the board edges
CORNER = {1: (TAG_IN, TAG_IN), 2: (BOARD[0] - TAG_IN, TAG_IN), 3: (BOARD[0] - TAG_IN, BOARD[1] - TAG_IN), 4: (TAG_IN, BOARD[1] - TAG_IN)}
ROBOT_H, ROBOT_TAG, CORNER_TAG = 10.5, 3.6, 8.0
ROBOT_BODY = (10.5, 12.5)          # width (across), length (along heading)
TEX_S = 0.05                       # cm per texture px
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
FPS = 15


def rot(ax, ay, az):
    cx, sx, cy, sy, cz, sz = np.cos(ax), np.sin(ax), np.cos(ay), np.sin(ay), np.cos(az), np.sin(az)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]]); Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rx @ Ry @ Rz


class Camera:
    def __init__(self, centre=(34.0, 29.0, 115.0), tilt=(0.04, -0.03, 0.06)):
        self.K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1]])
        R0 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)   # straight down, image x = world x
        self.R = rot(*tilt) @ R0
        self.C = np.array(centre, dtype=float)
        self.t = -self.R @ self.C

    def H(self, z=0.0):
        return self.K @ np.column_stack([self.R[:, 0], self.R[:, 1], z * self.R[:, 2] + self.t])

    def px(self, x, y, z=0.0):
        p = self.H(z) @ np.array([x, y, 1.0])
        return p[:2] / p[2]

    def state(self):
        return {"f": F, "cx": W / 2, "cy": H / 2, "rvec": cv2.Rodrigues(self.R)[0].ravel().tolist(), "tvec": self.t.tolist()}


def marker(id_, edge_cm, margin_cm):
    """White square with the black marker centred, at TEX_S cm/px."""
    side, m = int(round(edge_cm / TEX_S)), int(round(margin_cm / TEX_S))
    img = np.full((side + 2 * m, side + 2 * m), 255, np.uint8)
    img[m:m + side, m:m + side] = cv2.aruco.generateImageMarker(DICT, id_, side)
    return img


def tex_matrix(x0, y1, s=TEX_S):
    """Texture px (u right, v down) -> world cm, texture's top-left at world (x0, y1), y up."""
    return np.array([[s, 0, x0], [0, -s, y1], [0, 0, 1]])


def paste(frame, tex, mask, world_from_tex, cam, z):
    Hm = cam.H(z) @ world_from_tex
    warped = cv2.warpPerspective(tex, Hm, (W, H), flags=cv2.INTER_LINEAR)
    wmask = cv2.warpPerspective(mask, Hm, (W, H), flags=cv2.INTER_NEAREST)
    frame[wmask > 0] = warped[wmask > 0]


class Scene:
    def __init__(self, seed=0):
        self.rng = np.random.default_rng(seed)
        self.cam = Camera()
        self.floor = self._floor_texture()

    def _floor_texture(self):
        """Grey carpet with the plywood board, its grain, a shading gradient and the four corner tags."""
        x0, y0, x1, y1 = -20.0, -20.0, BOARD[0] + 20, BOARD[1] + 20
        w, h = int((x1 - x0) / TEX_S), int((y1 - y0) / TEX_S)
        tex = np.full((h, w, 3), (95, 95, 92), np.uint8)
        bx0, by0 = int((0 - x0) / TEX_S), int((y1 - BOARD[1]) / TEX_S)
        bw, bh = int(BOARD[0] / TEX_S), int(BOARD[1] / TEX_S)
        board = np.empty((bh, bw, 3), np.float32)
        board[:] = (128, 172, 208)                                          # plywood, BGR
        grain = cv2.GaussianBlur(self.rng.normal(0, 1, (bh, bw)).astype(np.float32), (0, 0), 3)
        streaks = cv2.resize(self.rng.normal(0, 1, (bh // 40, bw // 4)).astype(np.float32), (bw, bh), interpolation=cv2.INTER_CUBIC)
        board += (grain * 6 + streaks * 5)[..., None]
        shade = np.linspace(0.78, 1.0, bw, dtype=np.float32)[None, :, None]   # a soft shadow across the board
        board *= shade
        tex[by0:by0 + bh, bx0:bx0 + bw] = np.clip(board, 0, 255).astype(np.uint8)
        for id_, (cx, cy) in CORNER.items():
            m = marker(id_, CORNER_TAG, 1.2)
            u, v = int((cx - x0) / TEX_S - m.shape[1] / 2), int((y1 - cy) / TEX_S - m.shape[0] / 2)
            tex[v:v + m.shape[0], u:u + m.shape[1]] = m[..., None]
        self.floor_world = tex_matrix(x0, y1)
        return tex

    def render(self, robot=None, obstacles=(), hand=None, light=1.0, tint=(1.0, 1.0, 1.0)):
        key = tuple((o["x"], o["y"], o["r"]) for o in obstacles)
        if getattr(self, "_floor_key", None) != key:
            self._floor_key, self._floor_frame = key, self._render_floor(obstacles)
        frame = self._floor_frame.copy()
        for o in obstacles:
            r_px = int(o["r"] / TEX_S)
            tex = np.zeros((2 * r_px + 2, 2 * r_px + 2, 3), np.uint8); mask = np.zeros(tex.shape[:2], np.uint8)
            if o.get("shape") == "rect":
                cv2.rectangle(tex, (1, 1), (2 * r_px, 2 * r_px), o["bgr"], -1); cv2.rectangle(mask, (1, 1), (2 * r_px, 2 * r_px), 255, -1)
            else:
                cv2.circle(tex, (r_px + 1, r_px + 1), r_px, o["bgr"], -1); cv2.circle(mask, (r_px + 1, r_px + 1), r_px, 255, -1)
            paste(frame, tex, mask, tex_matrix(o["x"] - o["r"], o["y"] + o["r"]), self.cam, o["h"])
        if robot is not None:
            self._paste_robot(frame, robot)
        if hand is not None:
            hw, hl = 9.0, 16.0
            tex = np.zeros((int(hl / TEX_S), int(hw / TEX_S), 3), np.uint8); mask = np.zeros(tex.shape[:2], np.uint8)
            cv2.ellipse(tex, (tex.shape[1] // 2, tex.shape[0] // 2), (tex.shape[1] // 2 - 1, tex.shape[0] // 2 - 1), 0, 0, 360, (140, 170, 225), -1)
            cv2.ellipse(mask, (tex.shape[1] // 2, tex.shape[0] // 2), (tex.shape[1] // 2 - 1, tex.shape[0] // 2 - 1), 0, 0, 360, 255, -1)
            paste(frame, tex, mask, tex_matrix(hand[0] - hw / 2, hand[1] + hl / 2), self.cam, 4.0)
        out = frame.astype(np.float32) * light * np.array(tint, np.float32)
        out += self.rng.normal(0, 2.5, out.shape).astype(np.float32)
        return np.clip(out, 0, 255).astype(np.uint8)

    def _render_floor(self, obstacles):
        frame = np.full((H, W, 3), (95, 95, 92), np.uint8)
        floor = self.floor.copy()
        for o in obstacles:                                                 # a soft shadow beside each obstacle
            cx, cy = o["x"] + 1.5, o["y"] - 1.0
            u, v = int((cx - (-20)) / TEX_S), int((BOARD[1] + 20 - cy) / TEX_S)
            layer = np.zeros(floor.shape[:2], np.float32)
            cv2.circle(layer, (u, v), int((o["r"] + 1.5) / TEX_S), 1.0, -1)
            layer = cv2.GaussianBlur(layer, (0, 0), 25)
            floor = np.clip(floor * (1 - 0.35 * layer)[..., None], 0, 255).astype(np.uint8)
        paste(frame, floor, np.full(floor.shape[:2], 255, np.uint8), self.floor_world, self.cam, 0.0)
        return frame

    def _paste_robot(self, frame, robot):
        bw, bl = ROBOT_BODY
        tex = np.zeros((int(bl / TEX_S), int(bw / TEX_S), 3), np.uint8)
        tex[:] = (60, 60, 65)                                                # dark plastic body
        tex[:, :4] = tex[:, -4:] = (30, 30, 30)
        m = marker(0, ROBOT_TAG, 0.6)
        u, v = tex.shape[1] // 2 - m.shape[1] // 2, tex.shape[0] // 2 - m.shape[0] // 2
        tex[v:v + m.shape[0], u:u + m.shape[1]] = m[..., None]
        mask = np.full(tex.shape[:2], 255, np.uint8)
        # texture "up" (the tag's top edge) points along the heading: rotate by heading - 90 about the centre
        a = np.radians(robot["heading"] - 90)
        c, s = np.cos(a), np.sin(a)
        local = tex_matrix(-bw / 2, bl / 2)                                  # texture -> body frame, y up
        R = np.array([[c, -s, robot["x"]], [s, c, robot["y"]], [0, 0, 1]])
        paste(frame, tex, mask, R @ local, self.cam, ROBOT_H)

    def robot_state(self, robot):
        px = self.cam.px(robot["x"], robot["y"], ROBOT_H)
        return {"x": robot["x"], "y": robot["y"], "z": ROBOT_H, "heading": robot["heading"], "px": px.tolist()}


def write_clip(root, name, frames):
    """frames: iterable of (image, robot, obstacles, extra_truth). Writes JPEGs and states.jsonl."""
    d = os.path.join(root, name)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "states.jsonl"), "w") as f:
        for i, (img, scene, robot, obstacles, extra) in enumerate(frames):
            fn = f"frame-{i:06d}.jpg"
            cv2.imwrite(os.path.join(d, fn), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
            state = {"t": int(i * 1000 / FPS), "calibrated": True, "frame": [W, H], "floor": list(BOARD), "zUp": True,
                     "learned": 4, "cameraHeight": 115, "floorMarkers": 4, "fps": FPS,
                     "markers": [0, 1, 2, 3, 4] if robot else [1, 2, 3, 4],
                     "robot": scene.robot_state(robot) if robot else None, "arm": None, "camera": scene.cam.state()}
            truth = {"board": list(BOARD), "corner_centres": {str(k): list(v) for k, v in CORNER.items()},
                     "robot": robot, "robot_height": ROBOT_H, "robot_body": list(ROBOT_BODY),
                     "obstacles": [{k: o[k] for k in ("x", "y", "r", "h")} for o in obstacles], **extra}
            f.write(json.dumps({"file": fn, "state": state, "truth": truth}) + "\n")
    print(f"wrote {i + 1} frames to {d}")


# Two small obstacles well apart. With the robot's 8 cm half-diagonal inflated on a 55 cm tag rectangle only a
# 33 cm interior is free, so three 4 cm obstacles leave no path at all; that is the real arena's budget too.
OBSTACLES = [
    {"x": 20.0, "y": 44.0, "r": 2.5, "h": 6.0, "bgr": (40, 60, 200)},                       # red cup
    {"x": 46.0, "y": 20.0, "r": 2.5, "h": 5.0, "bgr": (190, 110, 40), "shape": "rect"},     # blue box
]


def main(root="recordings", only=None):
    scene = Scene()
    still = {"x": 47.0, "y": 46.0, "heading": 120.0}   # away from every tag and obstacle

    def clip(name, n, robot_fn, obstacles=(), hand_fn=lambda i: None, light_fn=lambda i: (1.0, (1, 1, 1))):
        if only and name not in only:
            return
        def gen():
            for i in range(n):
                light, tint = light_fn(i)
                robot = robot_fn(i)
                img = scene.render(robot, obstacles, hand_fn(i), light, tint)
                yield img, scene, robot, obstacles, {"hand": hand_fn(i), "light": light}
        write_clip(root, name, gen())

    clip("synth-empty", 45, lambda i: still)
    clip("synth-obstacles", 60, lambda i: still, OBSTACLES)
    clip("synth-hand", 75, lambda i: still, OBSTACLES, lambda i: (68.0 - (i - 25) * 1.6, 33.0) if 25 <= i < 55 else None)
    clip("synth-lights", 60, lambda i: still, OBSTACLES, light_fn=lambda i: (1.0, (1, 1, 1)) if i < 30 else (0.72, (0.92, 0.98, 1.06)))

    def drive(i):   # walk +x, turn, walk +y, with a gentle heading wobble; starts clear of tag 1's line of sight
        t = i / FPS
        if t < 4:
            return {"x": 16.0 + 4.0 * t, "y": 14.0, "heading": 0.0 + 2 * np.sin(3 * t)}
        if t < 6:
            return {"x": 32.0, "y": 14.0, "heading": 45.0 * (t - 4)}
        return {"x": 32.0, "y": 14.0 + 4.0 * (t - 6), "heading": 90.0 + 2 * np.sin(3 * t)}
    clip("synth-driving", 150, drive, OBSTACLES)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "recordings", sys.argv[2:] or None)
