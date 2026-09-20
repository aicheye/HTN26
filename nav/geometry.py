"""Stage 1 and 2: tag detection, frozen camera pose, arena frame, robot pose, rectification.

Units: centimetres and degrees everywhere. Pixels are full-frame pixels.

Arena frame: origin at the centre of the lowest-ID corner tag, x and y from that tag's own axes snapped
by 90 degrees so the other three corners have positive coordinates, z up toward the camera. When the tags
are laid out as the Pi tracker expects (1 -> 2 along x, 1 -> 4 along y) this is the tracker's frame.

The only geometry constants are the printed tag edge lengths. The board size, camera height and the
robot tag height are all measured from the tags.
"""
import numpy as np
import cv2
from scipy.optimize import least_squares

TAG_CM = {"corner": 8.0, "robot": 3.6, "arm": 3.6}      # printed edge lengths
CORNER_IDS = (1, 2, 3, 4)
ROBOT_ID, ARM_ID = 0, 5
KNOWN_IDS = {ROBOT_ID, ARM_ID, *CORNER_IDS}
FOCAL_PX = 1693.0                                        # Camera Module 3, 2304x1296 binned mode
RECT_CM_PER_PX = 0.2
RECT_MARGIN_CM = 6.0                                     # rectified view extends this far past the tag centres
HEIGHT_LOCK_FRAMES = 20                                  # robot.z samples before its height is fixed
CALIB_WAIT_FRAMES = 45                                   # frames to wait for the precise detector to see all four corners


def make_detector(precise=False):
    """precise=True uses AprilTag corner refinement: corners good to 0.2 px but about 350 ms a frame, so it
    is used only for the one-time calibration. The runtime detector uses subpixel refinement (about 14 ms).
    A dark object touching a tag can pull one raw contour corner 10+ px onto the object; the precise
    detector does not have that failure, and the calibration fit is robust to it as well."""
    p = cv2.aruco.DetectorParameters()
    # Verified on real frames: the 50 px robot tag is missed at the defaults; relaxing further hallucinates ids.
    p.adaptiveThreshWinSizeMin, p.adaptiveThreshWinSizeMax, p.adaptiveThreshWinSizeStep = 3, 33, 4
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG if precise else cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), p)


def square(edge_cm):
    """Object points of a tag in its own frame, ArUco corner order: top-left, top-right, bottom-right, bottom-left."""
    h = edge_cm / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


def edge_px(quad):
    return float(np.mean([np.linalg.norm(quad[i] - quad[(i + 1) % 4]) for i in range(4)]))


def tag_pose(quad, edge_cm, K):
    """Rotation matrix and translation of a tag in the camera frame from its known size."""
    ok, rvec, tvec = cv2.solvePnP(square(edge_cm), quad.astype(np.float64), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    return cv2.Rodrigues(rvec)[0], tvec.reshape(3)


class Geometry:
    """Detects tags, freezes the camera pose from the corner tags, and converts between px and cm."""

    def __init__(self, frame_shape, focal_px=FOCAL_PX):
        h, w = frame_shape[:2]
        self.K = np.array([[focal_px, 0, w / 2], [0, focal_px, h / 2], [0, 0, 1]], dtype=np.float64)
        self.detector = make_detector()
        self.precise_detector = make_detector(precise=True)
        self.frozen = False
        self.R = self.t = None            # world -> camera: p_cam = R p_world + t
        self.C = None                     # camera centre in world (cm)
        self.corner_world = {}            # id -> 4x3 world corners of each corner tag
        self.corner_centre = {}           # id -> (x, y) cm
        self.polygon = None               # 4x2 arena polygon through the tag centres, in id order
        self.board_cm = None              # (w, h) of the rectangle spanned by the tag centres
        self.reproj_err = None            # RMS px of the frozen pose
        self.solves = 0
        self.robot_height = None          # locked robot tag height (cm)
        self._heights = []
        self.z_log = []                   # every robot.z measurement (cm), the geometry health metric
        self.unexpected_ids = 0
        self.rejected = 0
        self.calib_dropped = 0            # calibration corners dropped as outliers
        self.calib_waited = 0             # frames skipped waiting for a clean view of all four corner tags
        self.calib_missing = []           # corner tags the precise detector last failed to see
        self.rect = None                  # set by build_rectifier
        self.source = None                # "tags" (calibrated here from all four corners) or "tracker" (pose taken from the Pi tracker)

    # --- detection ---------------------------------------------------------------------------------------
    def detect(self, gray, precise=False):
        """{id: 4x2 float32 corners} after rejecting unknown ids, undersized tags and displaced static tags."""
        corners, ids, _ = (self.precise_detector if precise else self.detector).detectMarkers(gray)
        found = {}
        if ids is None:
            return found
        quads = {int(i): c.reshape(4, 2) for c, i in zip(corners, ids.ravel())}
        self.unexpected_ids += sum(1 for i in quads if i not in KNOWN_IDS)
        quads = {i: q for i, q in quads.items() if i in KNOWN_IDS}
        # expected px size: from the frozen camera height, else from the corner tags seen this frame
        corner_px = [edge_px(quads[i]) for i in CORNER_IDS if i in quads]
        if self.frozen:
            floor_px = self.K[0, 0] * TAG_CM["corner"] / self.C[2]
        elif corner_px:
            floor_px = float(np.median(corner_px))
        else:
            floor_px = None
        for i, q in quads.items():
            size = TAG_CM["corner"] if i in CORNER_IDS else TAG_CM["robot"]
            if floor_px is not None and edge_px(q) < 0.5 * floor_px * size / TAG_CM["corner"]:
                self.rejected += 1
                continue
            if self.frozen and i in CORNER_IDS:
                expected = self.project(self.corner_world[i]).mean(axis=0)
                if np.linalg.norm(q.mean(axis=0) - expected) > 2 * edge_px(q):
                    self.rejected += 1
                    continue
            found[i] = q
        return found

    # --- camera pose -------------------------------------------------------------------------------------
    def try_freeze(self, tags, gray=None):
        """Solve the camera pose from all four corner tags. The first success calibrates the arena frame and
        freezes the pose; later ones only replace the pose when the reprojection error improves.
        Pass the frame for the calibration so it can use the precise detector. Returns True when solved."""
        if not all(i in tags for i in CORNER_IDS):
            return False
        if not self.frozen:
            # Calibrate from the precise detector's corners. It can miss a corner tag that the runtime
            # detector finds (a dark object touching it); then wait for a clean frame rather than calibrate on
            # a corner that may be pulled onto the object, up to CALIB_WAIT_FRAMES, then use what there is.
            if gray is not None:
                precise = self.detect(gray, precise=True)
                if all(i in precise for i in CORNER_IDS):
                    tags = precise
                elif self.calib_waited < CALIB_WAIT_FRAMES:
                    self.calib_waited += 1
                    self.calib_missing = [i for i in CORNER_IDS if i not in precise]
                    return False
            return self._calibrate(tags)
        obj = np.concatenate([self.corner_world[i] for i in CORNER_IDS])
        img = np.concatenate([tags[i] for i in CORNER_IDS]).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, img, self.K, None, cv2.Rodrigues(self.R)[0], self.t.reshape(3, 1),
                                      useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return False
        err = self._reproj(obj, img, rvec, tvec)
        if self.reproj_err == self.reproj_err and err >= self.reproj_err:     # nan: pose came from the tracker, replace it
            return False
        self._set_pose(cv2.Rodrigues(rvec)[0], tvec.reshape(3), err)
        return True

    def _reproj(self, obj, img, rvec, tvec):
        proj = cv2.projectPoints(obj, rvec, tvec, self.K, None)[0].reshape(-1, 2)
        return float(np.sqrt(np.mean(np.sum((proj - img) ** 2, axis=1))))

    def _set_pose(self, R, t, err):
        self.R, self.t, self.C, self.reproj_err, self.frozen = R, t, -R.T @ t, err, True
        self.solves += 1
        self.build_rectifier()

    def _calibrate(self, tags):
        """Arena frame and camera pose from the four corner tags at once.

        Unknowns: the camera pose (6) and, on the floor plane, the position and in-plane rotation of tags
        2-4 relative to tag 1 (9). Tag 1 sits at the origin with its own axes as the plane axes. All 16
        corners are 8 cm squares on z = 0, so the 32 image observations pin the 15 unknowns down. This is
        solved jointly by least squares, because single-tag poses (IPPE) carry a few degrees of orientation
        ambiguity each and a frame built from them lands several cm off. The plane axes are then snapped
        by 90 degrees so the other three corners have positive coordinates."""
        ids = list(CORNER_IDS)
        poses = {i: tag_pose(tags[i], TAG_CM["corner"], self.K) for i in ids}
        if any(p is None for p in poses.values()):
            return False
        # initial guess from the single-tag poses
        n = sum(p[0][:, 2] for p in poses.values()); n /= np.linalg.norm(n)
        centres = {i: p[1] for i, p in poses.items()}
        if n @ np.mean(list(centres.values()), axis=0) > 0:
            n = -n
        o_id = ids[0]
        R1, origin = poses[o_id]
        ex = R1[:, 0] - n * (n @ R1[:, 0]); ex /= np.linalg.norm(ex); ey = np.cross(n, ex)
        Rw = np.vstack([ex, ey, n])                       # world axes in camera coords
        layout0 = []
        for i in ids[1:]:
            Ri, ci = poses[i]
            x, y = Rw[:2] @ (ci - origin)
            ax = Rw[:2] @ Ri[:, 0]
            layout0 += [x, y, np.arctan2(ax[1], ax[0])]
        rvec0 = cv2.Rodrigues(Rw.T)[0].ravel()
        img = np.concatenate([tags[i] for i in ids]).astype(np.float64)
        local = square(TAG_CM["corner"])

        def world_points(layout):
            pts = [local]
            for k in range(len(ids) - 1):
                x, y, th = layout[3 * k: 3 * k + 3]
                c, s_ = np.cos(th), np.sin(th)
                pts.append(local @ np.array([[c, s_, 0], [-s_, c, 0], [0, 0, 1]]) + [x, y, 0])
            return np.concatenate(pts)

        def residual(p, keep=None):
            proj = cv2.projectPoints(world_points(p[6:]), p[:3], p[3:6], self.K, None)[0].reshape(-1, 2)
            r = proj - img
            return (r if keep is None else r[keep]).ravel()

        p0 = np.concatenate([rvec0, origin, layout0])
        sol = least_squares(residual, p0, loss="soft_l1", f_scale=1.0, xtol=1e-10, ftol=1e-10)
        # drop corners the robust fit left more than 3 px out (a contour pulled onto a neighbouring object)
        keep = np.linalg.norm(residual(sol.x).reshape(-1, 2), axis=1) <= 3.0
        if keep.sum() >= 12 and not keep.all():
            sol = least_squares(residual, sol.x, method="lm", xtol=1e-10, ftol=1e-10, args=(keep,))
        self.calib_dropped = int((~keep).sum())
        rvec, tvec, layout = sol.x[:3], sol.x[3:6], sol.x[6:]
        self.calib_dropped = int((~keep).sum())
        world = world_points(layout)
        # snap the axes by 90 degrees so tags 2-4 have positive coordinates
        cents = world.reshape(-1, 4, 3).mean(axis=1)[1:, :2]
        best = max(range(4), key=lambda k: min((cents @ np.array([[np.cos(k * np.pi / 2), np.sin(k * np.pi / 2)],
                                                                  [-np.sin(k * np.pi / 2), np.cos(k * np.pi / 2)]])).ravel()))
        a = best * np.pi / 2
        Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        world = world @ Rz.T
        R = cv2.Rodrigues(rvec)[0] @ Rz.T
        proj = cv2.projectPoints(world, cv2.Rodrigues(R)[0], tvec, self.K, None)[0].reshape(-1, 2)
        err = float(np.sqrt(np.mean(np.sum((proj - img)[keep] ** 2, axis=1))))
        for k, i in enumerate(ids):
            self.corner_world[i] = world[4 * k: 4 * k + 4]
            self.corner_centre[i] = tuple(float(v) for v in world[4 * k: 4 * k + 4, :2].mean(axis=0))
        self.polygon = np.array([self.corner_centre[i] for i in ids], dtype=np.float32)
        self.board_cm = (float(self.polygon[:, 0].max() - self.polygon[:, 0].min()),
                         float(self.polygon[:, 1].max() - self.polygon[:, 1].min()))
        self._set_pose(R, tvec, err)
        self.source = "tags"
        return True

    def freeze_from_tracker(self, state):
        """Fallback when the four corner tags are never all in view: take the Pi tracker's camera pose for
        this frame (pi/API.md "camera": pixel = K (R p + t), floor frame origin at tag 1, x toward tag 2)
        and its floor rectangle. The board size is then the tracker's argument, not derived, and the
        summary says so. Returns True when adopted."""
        cam = state.get("camera") if state else None
        if not cam or not state.get("floor"):
            return False
        R = cv2.Rodrigues(np.array(cam["rvec"], dtype=np.float64))[0]
        t = np.array(cam["tvec"], dtype=np.float64)
        W, H = float(state["floor"][0]), float(state["floor"][1])
        # the tracker's floor: tag centres at (0,0), (W,0), (W,H), (0,H); corner squares for the static-tag check
        for i, (cx, cy) in zip(CORNER_IDS, ((0, 0), (W, 0), (W, H), (0, H))):
            self.corner_world[i] = square(TAG_CM["corner"]) + [cx, cy, 0]
            self.corner_centre[i] = (float(cx), float(cy))
        self.polygon = np.array([self.corner_centre[i] for i in CORNER_IDS], dtype=np.float32)
        self.board_cm = (W, H)
        self.K = np.array([[cam["f"], 0, cam["cx"]], [0, cam["f"], cam["cy"]], [0, 0, 1]], dtype=np.float64)
        img_pts = []; obj_pts = []
        self._set_pose(R, t, float("nan"))
        self.source = "tracker"
        self.calib_missing = []
        return True
    # --- conversions -------------------------------------------------------------------------------------
    def homography(self, height_cm=0.0):
        """World (X, Y) on the plane z = height -> pixel, as a 3x3 matrix."""
        return self.K @ np.column_stack([self.R[:, 0], self.R[:, 1], height_cm * self.R[:, 2] + self.t])

    def project(self, world_xyz):
        p = (self.R @ np.asarray(world_xyz, dtype=np.float64).T).T + self.t
        return (p[:, :2] / p[:, 2:3]) @ np.diag([self.K[0, 0], self.K[1, 1]]) + [self.K[0, 2], self.K[1, 2]]

    def px_to_world(self, px, height_cm=0.0):
        """Pixels (N x 2) -> world (N x 2) on the plane z = height."""
        Hinv = np.linalg.inv(self.homography(height_cm))
        p = np.column_stack([np.asarray(px, dtype=np.float64).reshape(-1, 2), np.ones(len(np.atleast_2d(px)))]) @ Hinv.T
        return p[:, :2] / p[:, 2:3]

    def world_to_cam(self, world_xyz):
        return (self.R @ np.asarray(world_xyz, dtype=np.float64).reshape(-1, 3).T).T + self.t

    def cam_to_world(self, cam_xyz):
        return (self.R.T @ (np.asarray(cam_xyz, dtype=np.float64).reshape(-1, 3) - self.t).T).T

    # --- robot -------------------------------------------------------------------------------------------
    def robot_pose(self, tags):
        """{"x", "y", "z", "heading", "px"} for the robot tag, or None. z is this frame's measurement;
        x, y and heading come from the tag centre ray at the locked height once it is known."""
        if ROBOT_ID not in tags or not self.frozen:
            return None
        quad = tags[ROBOT_ID]
        pose = tag_pose(quad, TAG_CM["robot"], self.K)
        if pose is None:
            return None
        z = float(self.cam_to_world(pose[1])[0, 2])
        self.z_log.append(z)
        if self.robot_height is None:
            self._heights.append(z)
            if len(self._heights) >= HEIGHT_LOCK_FRAMES:
                self.robot_height = float(np.median(self._heights))
        h = self.robot_height if self.robot_height is not None else float(np.median(self._heights))
        centre = quad.mean(axis=0)
        front = (quad[0] + quad[1]) / 2
        (cx, cy), (fx, fy) = self.px_to_world(np.array([centre, front]), h)
        heading = float(np.degrees(np.arctan2(fy - cy, fx - cx)))
        return {"x": float(cx), "y": float(cy), "z": z, "heading": heading, "px": [float(centre[0]), float(centre[1])]}

    def z_std(self):
        return float(np.std(self.z_log)) if len(self.z_log) > 1 else float("nan")

    # --- rectification -----------------------------------------------------------------------------------
    def build_rectifier(self):
        """Top-down view at RECT_CM_PER_PX covering the arena plus a margin; y up in the world, rows down."""
        m = RECT_MARGIN_CM
        x0, y0 = self.polygon[:, 0].min() - m, self.polygon[:, 1].min() - m
        x1, y1 = self.polygon[:, 0].max() + m, self.polygon[:, 1].max() + m
        s = RECT_CM_PER_PX
        w, h = int(np.ceil((x1 - x0) / s)), int(np.ceil((y1 - y0) / s))
        M = np.array([[1 / s, 0, -x0 / s], [0, -1 / s, y1 / s], [0, 0, 1]])      # world -> rect px
        self.rect = {"x0": x0, "y0": y0, "x1": x1, "y1": y1, "w": w, "h": h, "M": M, "Minv": np.linalg.inv(M),
                     "cm_per_px": s, "px_to_rect": M @ np.linalg.inv(self.homography(0.0))}

    def rectify(self, frame):
        r = self.rect
        return cv2.warpPerspective(frame, r["px_to_rect"], (r["w"], r["h"]), flags=cv2.INTER_LINEAR)

    def validity_mask(self, frame_shape):
        """Where the rectified view is covered by the camera frame."""
        r = self.rect
        ones = np.full(frame_shape[:2], 255, np.uint8)
        return cv2.warpPerspective(ones, r["px_to_rect"], (r["w"], r["h"]), flags=cv2.INTER_NEAREST) > 0

    def world_to_rect(self, xy):
        p = np.column_stack([np.asarray(xy, dtype=np.float64).reshape(-1, 2), np.ones(len(np.atleast_2d(xy)))]) @ self.rect["M"].T
        return p[:, :2]

    def rect_to_world(self, uv):
        p = np.column_stack([np.asarray(uv, dtype=np.float64).reshape(-1, 2), np.ones(len(np.atleast_2d(uv)))]) @ self.rect["Minv"].T
        return p[:, :2]

    def summary(self):
        return {"frozen": self.frozen, "source": self.source, "reproj_px": self.reproj_err, "board_cm": self.board_cm,
                "camera_height_cm": None if self.C is None else float(self.C[2]),
                "robot_height_cm": self.robot_height, "z_std_cm": self.z_std(), "solves": self.solves,
                "unexpected_ids": self.unexpected_ids, "rejected": self.rejected, "calib_dropped": self.calib_dropped, "calib_waited": self.calib_waited}
