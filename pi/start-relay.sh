# Starts pi/relay.py on the Pi, in the background, so the laptop reaches the robot through the Pi.
# Run from the HTN repo root:  sh pi/start-relay.sh        (it keeps running until the Pi reboots)
# Then start the bridge with:  ROBOT_URL=ws://qnxpi78.local:8181 npm start
. "$(dirname "$0")/common.sh"
scp -q $SSH_OPTS pi/relay.py $PI: || exit 1
ssh $SSH_OPTS $PI 'slay -f python3 >/dev/null 2>&1; nohup python3 relay.py > relay.log 2>&1 & sleep 1; cat relay.log'
