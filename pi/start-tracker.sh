# Runs on the laptop while on the Sesame-Controller WiFi, from the HTN repo root:
#   sh pi/start-tracker.sh <unit> <floor width cm> <floor height cm> [lens code]
# Copies the tracker to the Pi, builds it, and runs it until Ctrl-C. Status prints once per second
# and is also written to pi/tracker-live.log.
UNIT=${1:-3}; W=${2:-100}; H=${3:-100}; FOCUS=${4:-}
. "$(dirname "$0")/common.sh"
scp -q $SSH_OPTS -r pi/tracker $PI: || exit 1
ssh -t $SSH_OPTS $PI "cd tracker && slay -f tracker >/dev/null 2>&1; clang++ -std=c++17 -O2 tracker.cpp -o tracker \$(pkg-config --cflags --libs opencv4) -lcamapi -lsocket && ./tracker $UNIT $W $H $FOCUS" 2>&1 | tee pi/tracker-live.log
