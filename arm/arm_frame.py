"""The rigid transform between the tracker's floor frame and the arm's base_link frame.

Floor frame: cm, from the Pi tracker (origin at floor marker 1). Base frame: cm here, metres in so101_ik.
The table is TABLE_Z below base_link's origin, so a floor point at height h cm is at base z = h + 100*TABLE_Z.

Fitted by calibrate_arm_frame.py from points seen in both frames (the gripper held over the quadruped's tag):
p_base = R p_floor + t, with R orthogonal. The tracker's zUp flag says whether the floor frame is mirrored
(left-handed seen from above), which fixes det(R), so two points suffice and a third checks the fit.

The arm base tag (id 5) is taped to the floor at the arm, so its floor pose at calibration is stored too.
If it is visible later at a different pose, the arm or the board moved, and the transform is re-derived
from it (base_link sits at a fixed offset from that tag).
"""
import json

import numpy as np

from so101_ik import TABLE_Z

FLOOR_Z_IN_BASE_CM = 100 * TABLE_Z


def rot(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


class ArmFrame:
    def __init__(self, R, t, mirrored=False, arm_tag=None, residual_cm=None, units=None):
        self.R, self.t = np.asarray(R, float), np.asarray(t, float)
        self.mirrored = bool(mirrored)
        self.arm_tag = arm_tag            # {"x", "y", "heading"} floor pose of tag 5 at calibration, or None
        self.residual_cm = residual_cm
        self.units = units

    # --- fitting ---------------------------------------------------------------------------------------
    @classmethod
    def fit(cls, floor_xy, base_xy, mirrored=False, arm_tag=None):
        """Least-squares rigid fit p_base = R p_floor + t over matching points (N >= 2), det(R) = -1 if mirrored."""
        P, Q = np.asarray(floor_xy, float), np.asarray(base_xy, float)
        pc, qc = P.mean(axis=0), Q.mean(axis=0)
        H = (P - pc).T @ (Q - qc)
        U, _, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T)) or 1.0
        R = Vt.T @ np.diag([1.0, (-1.0 if mirrored else 1.0) * d]) @ U.T     # det(R) = -1 if mirrored else +1
        t = qc - R @ pc
        res = np.linalg.norm((P @ R.T + t) - Q, axis=1)
        return cls(R, t, mirrored, arm_tag, residual_cm=float(res.max()) if len(res) else 0.0)

    # --- use ---------------------------------------------------------------------------------------
    def to_base(self, xy):
        return np.asarray(xy, float).reshape(-1, 2) @ self.R.T + self.t

    def to_floor(self, xy):
        return (np.asarray(xy, float).reshape(-1, 2) - self.t) @ self.R

    def heading_to_base(self, deg):
        d = self.R @ np.array([np.cos(np.radians(deg)), np.sin(np.radians(deg))])
        return float(np.degrees(np.arctan2(d[1], d[0])))

    def pose_to_base(self, floor_pose):
        """{"x","y","z","heading"} in floor cm -> the same in base cm (z relative to base_link)."""
        x, y = self.to_base([[floor_pose["x"], floor_pose["y"]]])[0]
        return {"x": float(x), "y": float(y), "z": float(floor_pose.get("z", 0.0) + FLOOR_Z_IN_BASE_CM),
                "heading": self.heading_to_base(floor_pose["heading"])}

    def adjusted_for_arm_tag(self, arm_tag_now, tolerance_cm=1.5, tolerance_deg=3.0):
        """If tag 5 is seen somewhere else than at calibration, the arm or the board moved: return a frame
        re-derived from the tag's new pose. Otherwise return self."""
        if self.arm_tag is None or arm_tag_now is None:
            return self
        dx = arm_tag_now["x"] - self.arm_tag["x"]; dy = arm_tag_now["y"] - self.arm_tag["y"]
        dh = (arm_tag_now["heading"] - self.arm_tag["heading"] + 180) % 360 - 180
        if abs(dx) < tolerance_cm and abs(dy) < tolerance_cm and abs(dh) < tolerance_deg:
            return self
        # floor' = T(tag_now) T(tag_calib)^-1 floor  =>  base = R (T^-1 floor') + t
        Rc, tc = rot(self.arm_tag["heading"]), np.array([self.arm_tag["x"], self.arm_tag["y"]])
        Rn, tn = rot(arm_tag_now["heading"]), np.array([arm_tag_now["x"], arm_tag_now["y"]])
        M = Rc @ Rn.T                          # maps the new floor coordinates back to the calibration ones
        m = tc - M @ tn
        return ArmFrame(self.R @ M, self.R @ m + self.t, self.mirrored, arm_tag_now, self.residual_cm, self.units)

    # --- storage -----------------------------------------------------------------------------------
    def save(self, path):
        with open(path, "w") as f:
            json.dump({"R": self.R.tolist(), "t": self.t.tolist(), "mirrored": self.mirrored, "arm_tag": self.arm_tag,
                       "residual_cm": self.residual_cm, "units": self.units,
                       "frame": "p_base_cm = R p_floor_cm + t; floor z + FLOOR_Z_IN_BASE_CM = base z"}, f, indent=1)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = json.load(f)
        return cls(d["R"], d["t"], d.get("mirrored", False), d.get("arm_tag"), d.get("residual_cm"), d.get("units"))

    def describe(self):
        ang = np.degrees(np.arctan2(self.R[1, 0], self.R[0, 0]))
        return (f"floor -> base: rotate {ang:.1f} deg{' (mirrored)' if self.mirrored else ''}, shift ({self.t[0]:.1f}, {self.t[1]:.1f}) cm"
                + (f", fit residual {self.residual_cm:.2f} cm" if self.residual_cm is not None else "")
                + (f", arm tag at ({self.arm_tag['x']:.1f}, {self.arm_tag['y']:.1f}, {self.arm_tag['heading']:.0f} deg)" if self.arm_tag else ""))
