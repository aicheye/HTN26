# Runs on the laptop while on the Sesame-Controller WiFi, from the HTN repo root:
#   sh pi/start-tracker.sh <unit> <floor width cm> <floor height cm> [lens code | -] [robot marker height cm]
# Copies the tracker to the Pi, builds it, and runs it until Ctrl-C. Status prints once per second
# and is also written to pi/tracker-live.log.
UNIT=${1:-3}; W=${2:-100}; H=${3:-100}; FOCUS=${4:-}; ROBOT_HEIGHT=${5:-}
. "$(dirname "$0")/common.sh"
scp -q $SSH_OPTS -r pi/tracker $PI: || exit 1
# The camera's I2C buses belong to root again after every Pi reboot, and the tracker needs them to set the lens.
# sudo asks for the Pi password (qnxuser) the first time after a reboot.
ssh -t $SSH_OPTS $PI "[ -w /dev/i2c6 ] && [ -w /dev/i2c4 ] || { echo 'lens access was reset by a reboot, fixing it with sudo:'; sudo chmod 666 /dev/i2c4 /dev/i2c6; }; cd tracker && slay -f tracker >/dev/null 2>&1; clang++ -std=c++17 -O2 tracker.cpp -o tracker \$(pkg-config --cflags --libs opencv4) -lcamapi -lsocket && ./tracker $UNIT $W $H $FOCUS $ROBOT_HEIGHT" 2>&1 | tee pi/tracker-live.log
