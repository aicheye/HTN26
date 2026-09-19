# Runs on the laptop while on the Sesame-Controller WiFi, from the HTN repo root:
#   sh pi/start-tracker.sh <unit> <floor width cm> <floor height cm> [focus step]
# Copies the tracker to the Pi, builds it, and runs it until Ctrl-C. Status prints once per second.
UNIT=${1:-3}; W=${2:-100}; H=${3:-100}; FOCUS=${4:-}
PI=qnxuser@qnxpi78.local
SHARE="-o ControlMaster=auto -o ControlPath=/tmp/htn-pi-%C -o ControlPersist=120"
scp -q $SHARE -r pi/tracker $PI: || exit 1
ssh -t $SHARE $PI "cd tracker && slay -f tracker >/dev/null 2>&1; clang++ -std=c++17 -O2 tracker.cpp -o tracker \$(pkg-config --cflags --libs opencv4) -lcamapi -lsocket && ./tracker $UNIT $W $H $FOCUS"
