"""Closed-form inverse kinematics for the SO-101 arm (LeRobot SOFollower joint order).

Frames and conventions
- Positions are metres in base_link. Angles in and out are degrees.
- shoulder_pan rotates about -z, so azimuth = -pan.
- approach_pitch is the pitch of the gripper_frame_link z axis (the approach
  direction): 0 = horizontal forward, -90 = straight down (top-down grasp).
  In the URDF, lift + elbow + wrist_flex = -approach_pitch.
- jaw_yaw_deg is the table-plane heading of the gripper_frame_link x axis, the
  direction the jaws open and close. Jaws are symmetric, so if the direct
  wrist_roll is outside its limits the 180-degree-flipped roll is used.

Geometry (from so101_new_calib.urdf, verified against placo FK to 0.005 mm)
The lift/elbow/wrist_flex chain lives in a plane 18.28 mm beside the pan
axis; the wrist_roll origin hops back to 0.18 mm off the pan plane. Only the
in-plane components matter here, so R0 and L3 below are in-plane lengths and
are shorter than the 3D distances between the same frames.
"""
import numpy as np

P0 = np.array([0.0388353, 0.0, 0.0624])   # pan axis origin in base_link
R0, Z0 = 0.0303992, 0.0542               # shoulder_lift pivot: radial and height from P0 (in-plane)
L1, A1 = 0.1160000, np.radians(76.03225)  # shoulder_lift -> elbow_flex, and its angle at q=0
L2, A2 = 0.1350002, np.radians(2.207492)  # elbow_flex -> wrist_flex, and its angle at q=0
L3 = 0.0610999                            # wrist_flex -> wrist_roll origin, along the roll axis
LA = 0.0981274                            # wrist_roll origin -> gripper_frame, along the roll axis
LAT_R = -0.0001768                        # lateral offset of the roll axis from the pan plane
# gripper_frame offset perpendicular to the roll axis at roll=0: (lateral, in-plane) components.
# With roll: lateral = N0*cos(roll) + V0*sin(roll); in-plane = -N0*sin(roll) + V0*cos(roll).
N0, V0 = 0.0001676, -0.0079006
JAW_PHASE = np.radians(2.7896)            # jaw axis is rotated this much about the roll axis at roll=0
TABLE_Z = 0.0                             # targets below this height are rejected

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
LIMITS = {"shoulder_pan": (-110.0, 110.0), "shoulder_lift": (-100.0, 100.0),
          "elbow_flex": (-96.8, 96.8), "wrist_flex": (-95.0, 95.0), "wrist_roll": (-157.0, 163.0)}


def _wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def _roll_for(jaw_yaw, azimuth, pitch):
    """wrist_roll (rad) that points the jaw axis at heading jaw_yaw, given the pan azimuth and pitch."""
    h = jaw_yaw - azimuth
    s = np.sin(pitch)
    if abs(s) < 1e-9:
        return None                                       # horizontal approach: heading not set by roll
    direct = np.arctan2(-abs(s) * np.sin(h), np.sign(s) * np.cos(h)) + JAW_PHASE
    lo, hi = np.radians(LIMITS["wrist_roll"])
    for cand in (direct, direct + np.pi, direct - np.pi):  # direct first, then the flipped jaws
        cand = np.radians(_wrap(np.degrees(cand)))
        if lo <= cand <= hi:
            return cand
    return None


def ik(x, y, z, jaw_yaw_deg, approach_pitch_deg=-90.0):
    """Joint angles (deg) placing gripper_frame_link at (x, y, z), or None if unreachable."""
    if z < TABLE_Z:
        return None
    dx, dy, dz = x - P0[0], y - P0[1], z - P0[2]
    rho = np.hypot(dx, dy)
    yaw, pitch = np.radians(jaw_yaw_deg), np.radians(approach_pitch_deg)
    heading = np.arctan2(dy, dx)

    # The gripper frame sits off the pan plane by an amount that depends on wrist_roll,
    # and wrist_roll depends on the pan azimuth. Fixed-point iterate; converges in 2-3 steps.
    azimuth = heading
    for _ in range(8):
        roll = _roll_for(yaw, azimuth, pitch)
        if roll is None:
            return None
        lateral = LAT_R + N0 * np.cos(roll) + V0 * np.sin(roll)
        if abs(lateral) >= rho:
            return None
        azimuth = heading - np.arcsin(lateral / rho)

    # In-plane target for the wrist_flex pivot: walk back along the roll axis and the
    # in-plane part of the perpendicular offset.
    r_g = np.sqrt(rho**2 - lateral**2)
    u = np.array([np.cos(pitch), np.sin(pitch)])          # roll/approach axis in (radial, z)
    v = np.array([-np.sin(pitch), np.cos(pitch)])         # in-plane perpendicular
    inplane = -N0 * np.sin(roll) + V0 * np.cos(roll)
    w = np.array([r_g, dz]) - (L3 + LA) * u - inplane * v

    # 2R planar IK from the shoulder_lift pivot, elbow-up branch.
    d = w - np.array([R0, Z0])
    dist = np.linalg.norm(d)
    if dist > L1 + L2 or dist < abs(L1 - L2):
        return None
    beta = np.arctan2(d[1], d[0])
    alpha = np.arccos(np.clip((dist**2 + L1**2 - L2**2) / (2 * dist * L1), -1.0, 1.0))
    gamma = np.arccos(np.clip((L1**2 + L2**2 - dist**2) / (2 * L1 * L2), -1.0, 1.0))
    psi1 = beta + alpha                                    # link 1 steeper than the chord: elbow up
    lift = A1 - psi1
    elbow = A2 - A1 + np.pi - gamma
    wrist_flex = -pitch - lift - elbow

    sol = dict(zip(JOINTS, map(_wrap, np.degrees([-azimuth, lift, elbow, wrist_flex, roll]))))
    for name, val in sol.items():
        lo, hi = LIMITS[name]
        if not lo <= val <= hi:
            return None
    return sol


def reachable(x, y, z, jaw_yaw_deg, approach_pitch_deg=-90.0):
    return ik(x, y, z, jaw_yaw_deg, approach_pitch_deg) is not None
