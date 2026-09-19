# Runs on the laptop while on the Sesame-Controller WiFi, from the HTN repo root:
#   sh pi/run-tracker.sh <unit> <floor width cm> <floor height cm> [focus step]
# Copies the tracker to the Pi, builds it, runs it for 20 seconds, and copies back
# the log and a colour snapshot with detections drawn. Asks for the Pi password once.
UNIT=${1:-3}; W=${2:-100}; H=${3:-100}; FOCUS=${4:-}; SECS=20
PI=qnxuser@qnxpi78.local
SHARE="-o ControlMaster=auto -o ControlPath=/tmp/htn-pi-%C -o ControlPersist=120"
scp -q $SHARE -r pi/tracker $PI: || exit 1
ssh $SHARE $PI "cd tracker && clang++ -std=c++17 -O2 tracker.cpp -o tracker \$(pkg-config --cflags --libs opencv4) -lcamapi -lsocket 2>&1 | head -40 && (./tracker $UNIT $W $H $FOCUS & sleep 10; curl -s -m 5 -o http-frame.jpg http://127.0.0.1:800$UNIT/frame.jpg; echo \"http frame: \$(wc -c < http-frame.jpg) bytes\"; sleep 10; slay -f tracker >/dev/null 2>&1)" > pi/tracker-unit$UNIT.log 2>&1
scp -q $SHARE "$PI:tracker/snapshot-unit$UNIT.jpg" "$PI:tracker/http-frame.jpg" pi/ 2>/dev/null
tail -12 pi/tracker-unit$UNIT.log
