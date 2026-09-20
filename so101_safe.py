"""Send actions to the SO-101 follower only if the pose is inside the safe range.

    from so101_safe import send
    send(robot, {"shoulder_pan.pos": 10.0, ..., "gripper.pos": 30.0})

send() checks the five arm joints with so101_ik.check_pose (SAFE_LIMITS box plus
table clearance). A pose past the limit is not sent: the message says which joint
or link is out of range and UnsafePoseError is raised, because driving the arm past
that position risks breaking the robot. Actions use LeRobot's "<joint>.pos" keys in
degrees (SO101FollowerConfig.use_degrees=True), where 0 is the calibrated midpoint.
"""
import glob
import os

from so101_ik import JOINTS, UnsafePoseError, check_pose


def default_port():
    """The arm's serial port: $SO101_PORT if set, else the one USB serial device present. The SO-101's
    adapter shows up as /dev/tty.usbmodem* on macOS and /dev/ttyACM* on Linux, and its number changes."""
    if os.environ.get("SO101_PORT"):
        return os.environ["SO101_PORT"]
    for pattern in ("/dev/tty.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return None


def send(robot, action):
    """robot.send_action(action) after the safety check. Raises UnsafePoseError instead of sending."""
    joints = {j: float(action[f"{j}.pos"]) for j in JOINTS}
    msg = check_pose(joints)
    if msg:
        print(msg)
        raise UnsafePoseError(msg)
    return robot.send_action(action)
