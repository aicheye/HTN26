# Runs on the laptop while on the Sesame-Controller WiFi, from the HTN repo root:
#   sh pi/run-camtest.sh
# Copies the camera test to the Pi, builds and runs it there, and copies back the log,
# the captured frames, and the QNX camera headers. Asks for the Pi password once.
PI=qnxuser@qnxpi78.local
SHARE="-o ControlMaster=auto -o ControlPath=/tmp/htn-pi-%C -o ControlPersist=120"
scp -q $SHARE -r pi/camtest $PI: || exit 1
ssh $SHARE $PI 'cd camtest && sh build-and-run.sh' > pi/camtest.log 2>&1
scp -q $SHARE "$PI:camtest/frame-*.jpg" pi/ 2>/dev/null
scp -q $SHARE -r $PI:/usr/include/camera pi/qnx-camera-headers
tail -5 pi/camtest.log
