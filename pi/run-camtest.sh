# Runs on the laptop while on the Sesame-Controller WiFi, from the HTN repo root:
#   sh pi/run-camtest.sh
# Copies the camera test to the Pi, builds and runs it there, and copies back the log,
# the captured frames, and the QNX camera headers.
. "$(dirname "$0")/common.sh"
scp -q $SSH_OPTS -r pi/camtest $PI: || exit 1
ssh $SSH_OPTS $PI 'cd camtest && sh build-and-run.sh' > pi/camtest.log 2>&1
scp -q $SSH_OPTS "$PI:camtest/frame-*.jpg" pi/ 2>/dev/null
scp -q $SSH_OPTS -r $PI:/usr/include/camera pi/qnx-camera-headers
tail -5 pi/camtest.log
