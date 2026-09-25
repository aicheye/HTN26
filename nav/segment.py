"""Stage 3: obstacle segmentation on the rectified top-down view.

Chroma only (Lab a and b, L discarded), so shadows and lighting changes barely register. The arena's chroma
is modelled as one Gaussian fitted with sigma clipping to convergence, so obstacles covering a third of the
board do not contaminate it. No reference frame of an empty arena is needed.

Units: the rectified image is RECT_CM_PER_PX cm per pixel; areas are reported in cm^2.
"""
import numpy as np
import cv2

from .geometry import RECT_CM_PER_PX, CORNER_IDS, TAG_CM

CHI2_95, CHI2_99 = 5.991, 9.210          # chi-square, 2 dof
MIN_AREA_CM2 = 4.0
PERSIST_S = 0.15                         # consecutive occupancy to enter, and to leave, the map
ROBOT_MASK_SCALE = 1.35                  # robot body mask dilation: swallows its shadow and servo wires
TAG_MASK_SCALE = 0.85                    # corner tags are masked by a disc this many tag edges across
ROBOT_RADIUS_MAX_TAGS = 3.0              # a body cannot be wider than this many robot-tag edges: the blob merged with something else


class Segmenter:
    def __init__(self, geometry, dt_s):
        self.g = geometry
        r = geometry.rect
        self.shape = (r["h"], r["w"])
        self.px_per_cm = 1.0 / r["cm_per_px"]
        self.n_persist = max(1, int(round(PERSIST_S / max(dt_s, 1e-3))))
        self.on = np.zeros(self.shape, np.int16)      # consecutive frames a cell was occupied
        self.off = np.zeros(self.shape, np.int16)     # consecutive frames a cell was free
        self.persisted = np.zeros(self.shape, bool)
        self.arena = np.zeros(self.shape, np.uint8)
        cv2.fillPoly(self.arena, [np.round(geometry.world_to_rect(geometry.polygon)).astype(np.int32)], 255)
        self.arena = self.arena > 0
        self.tag_mask = np.zeros(self.shape, bool)
        for i in CORNER_IDS:
            u, v = geometry.world_to_rect([geometry.corner_centre[i]])[0]
            m = np.zeros(self.shape, np.uint8)
            cv2.circle(m, (int(round(u)), int(round(v))), int(TAG_MASK_SCALE * TAG_CM["corner"] * self.px_per_cm), 255, -1)
            self.tag_mask |= m > 0
        self.mu = self.cov = None
        self.last = None

    def robot_mask(self, robot, radius_cm, scale=ROBOT_MASK_SCALE):
        m = np.zeros(self.shape, np.uint8)
        if robot is not None and radius_cm:
            u, v = self.g.world_to_rect([[robot["x"], robot["y"]]])[0]
            cv2.circle(m, (int(round(u)), int(round(v))), int(round(scale * radius_cm * self.px_per_cm)), 255, -1)
        return m > 0

    def chroma_outliers(self, rect_bgr, fit_mask):
        """Mahalanobis^2 of every pixel's (a, b) from the arena chroma Gaussian. The Gaussian is fitted on
        fit_mask by sigma clipping to convergence, on a 1-in-9 pixel subsample (the estimate is the same and
        the loop is 9x cheaper); the final map is then evaluated once for every pixel in closed form."""
        ab = cv2.cvtColor(rect_bgr, cv2.COLOR_BGR2LAB)[..., 1:].astype(np.float32)
        a, b = ab[..., 0], ab[..., 1]
        sub = fit_mask[1::3, 1::3]
        sa, sb = a[1::3, 1::3][sub], b[1::3, 1::3][sub]
        if len(sa) < 50:
            return np.zeros(self.shape, np.float32)
        keep = np.ones(len(sa), bool)
        for _ in range(6):
            mu = np.array([np.median(sa[keep]), np.median(sb[keep])])
            cov = np.cov(np.vstack([sa[keep], sb[keep]])) + np.eye(2) * 0.25   # a floor so a flat board cannot make it singular
            inv = np.linalg.inv(cov)
            da, db = sa - mu[0], sb - mu[1]
            m2s = inv[0, 0] * da * da + 2 * inv[0, 1] * da * db + inv[1, 1] * db * db
            new = m2s < CHI2_95
            if np.array_equal(new, keep):
                break
            keep = new
        self.mu, self.cov = mu, cov
        da, db = a - mu[0], b - mu[1]
        return (inv[0, 0] * da * da + 2 * inv[0, 1] * da * db + inv[1, 1] * db * db).astype(np.float32)

    def segment(self, rect_bgr, valid, robot=None, robot_radius_cm=None):
        """One frame. Returns dict with raw and persisted masks and the obstacle components."""
        excluded = self.tag_mask | self.robot_mask(robot, robot_radius_cm)
        fit_mask = self.arena & valid & ~excluded
        m2 = self.chroma_outliers(rect_bgr, fit_mask)
        raw = ((m2 > CHI2_99) & fit_mask).astype(np.uint8)
        raw = cv2.morphologyEx(raw, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        raw = cv2.morphologyEx(raw, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8)) > 0
        # persistence counters: enter after n_persist consecutive occupied frames, leave after n_persist free ones
        self.on = np.where(raw, np.minimum(self.on + 1, 32000), 0).astype(np.int16)
        self.off = np.where(raw, 0, np.minimum(self.off + 1, 32000)).astype(np.int16)
        self.persisted = (self.persisted | (self.on >= self.n_persist)) & ~(self.off >= self.n_persist)
        comps = self.components(self.persisted)
        self.last = {"m2": m2, "raw": raw, "persisted": self.persisted.copy(), "components": comps, "fit_mask": fit_mask}
        return self.last

    def components(self, mask):
        n, labels, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
        out = []
        min_px = MIN_AREA_CM2 * self.px_per_cm ** 2
        keep = np.zeros(mask.shape, bool)
        for k in range(1, n):
            if stats[k, cv2.CC_STAT_AREA] < min_px:
                continue
            keep |= labels == k
            x, y = self.g.rect_to_world([cents[k]])[0]
            contours, _ = cv2.findContours((labels == k).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            out.append({"x": float(x), "y": float(y), "area_cm2": float(stats[k, cv2.CC_STAT_AREA] / self.px_per_cm ** 2),
                        "bbox": [int(v) for v in stats[k, :4]], "contour": contours[0] if contours else None})
        self.persisted &= keep
        return out

    def measure_robot_radius(self, rect_bgr, valid, robot):
        """Robot body radius (cm) at boot, while the robot stands still: the radius of the blob under the
        robot tag that is darker than the board (the body and legs are dark; shadows are handled by the
        margin the planner adds). Chroma is not used here because projected light and coloured objects
        next to the robot merge into a chroma blob. Capped at ROBOT_RADIUS_MAX_TAGS tag edges.
        None when nothing dark sits under the tag."""
        lab = cv2.cvtColor(rect_bgr, cv2.COLOR_BGR2LAB)
        L = lab[..., 0].astype(np.float32)
        fit_mask = self.arena & valid & ~self.tag_mask
        board = np.median(L[fit_mask]) if fit_mask.any() else 128.0
        spread = np.median(np.abs(L[fit_mask] - board)) * 1.4826 + 1.0 if fit_mask.any() else 10.0
        dark = ((L < board - max(4 * spread, 25.0)) & self.arena & valid).astype(np.uint8)
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        n, labels = cv2.connectedComponents(dark, connectivity=8)
        u, v = self.g.world_to_rect([[robot["x"], robot["y"]]])[0]
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= vi < self.shape[0] and 0 <= ui < self.shape[1]):
            return None
        k = labels[vi, ui]
        if k == 0:                                     # the tag itself is white/black: take the nearest dark blob within a tag edge
            ys, xs = np.nonzero(labels)
            if len(xs) == 0:
                return None
            d = np.hypot(xs - u, ys - v)
            if d.min() > TAG_CM["robot"] * self.px_per_cm:
                return None
            k = labels[ys[d.argmin()], xs[d.argmin()]]
        ys, xs = np.nonzero(labels == k)
        r = np.percentile(np.hypot(xs - u, ys - v), 95) / self.px_per_cm
        cap = ROBOT_RADIUS_MAX_TAGS * TAG_CM["robot"]
        if r > cap:                                    # the blob ran into a shadow, a dark object or the board edge
            print(f"robot radius blob is {r:.1f} cm, capped at {cap:.1f} cm (the body merged with something else)")
            r = cap
        return float(max(r, TAG_CM["robot"]))
