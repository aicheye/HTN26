# Runs on the laptop while on the Sesame-Controller WiFi, from the HTN repo root:
#   sh pi/run-focus.sh <unit> [lens code]
# Builds the focus tool on the Pi and runs it: a sweep that keeps the sharpest lens position, or one fixed
# code. Aim the camera at the markers from its working distance first. Brings back the log and a frame.
UNIT=${1:-3}; CODE=${2:-}
PI=qnxuser@qnxpi78.local
SHARE="-o ControlMaster=auto -o ControlPath=/tmp/htn-pi-%C -o ControlPersist=120"
scp -q $SHARE -r pi/tracker pi/focus $PI: || exit 1
ssh $SHARE $PI "slay -f tracker >/dev/null 2>&1; cd focus && clang++ -std=c++17 -O2 focus.cpp -o focus \$(pkg-config --cflags --libs opencv4) -lcamapi 2>&1 | head -30 && ./focus $UNIT $CODE" 2>&1 | tee pi/focus-unit$UNIT.log
scp -q $SHARE "$PI:focus/focus-unit$UNIT.jpg" pi/ 2>/dev/null
