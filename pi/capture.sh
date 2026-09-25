# Saves one full-size colour frame from each camera. Runs on the laptop while on the Pi's WiFi, from the HTN repo root:
#   sh pi/capture.sh [units]     default: "3 4". Unit 3 is the first camera, unit 4 the second.
# Writes pi/capture-unit3.jpg and pi/capture-unit4.jpg, and keeps the Pi's output in pi/capture.log.
# A unit whose tracker is already running (sh pi/live.sh) is read from that tracker, which keeps running.
# For the other units the tracker is built under its own name and run for 8 seconds, because it sets the saved
# lens code and the exposure needs a few seconds to settle. The floor size it is given does not affect the image.
UNITS=${1:-3 4}; SECS=8
. "$(dirname "$0")/common.sh"
cd "$(dirname "$0")/.." || exit 1
: > pi/capture.log

NEED=
for unit in $UNITS; do
  if curl -s -f -m 10 -o "pi/capture-unit$unit.jpg" "http://$PI_HOST:$((8000 + unit))/frame.jpg"; then
    echo "unit $unit: from the running tracker"
  else
    NEED="$NEED $unit"
  fi
done

if [ -n "$NEED" ]; then
  scp -q $SSH_OPTS -r pi/tracker $PI: || { echo "cannot reach the Pi at $PI_HOST"; exit 1; }
  # Same lens access fix as pi/start-tracker.sh: the I2C buses belong to root again after a Pi reboot.
  ssh -t $SSH_OPTS $PI "[ -w /dev/i2c6 ] && [ -w /dev/i2c4 ] || { echo 'lens access was reset by a reboot, fixing it with sudo:'; sudo chmod 666 /dev/i2c4 /dev/i2c6; }
    cd tracker && clang++ -std=c++17 -O2 tracker.cpp -o capture-tracker \$(pkg-config --cflags --libs opencv4) -lcamapi -lsocket 2>&1 | head -40
    [ -x ./capture-tracker ] || { echo 'build failed'; exit 1; }
    for unit in $NEED; do
      echo \"== unit \$unit\"
      rm -f capture-unit\$unit.jpg
      ./capture-tracker \$unit 63 63 & pid=\$!
      sleep $SECS
      curl -s -f -m 10 -o capture-unit\$unit.jpg http://127.0.0.1:\$((8000 + unit))/frame.jpg || rm -f capture-unit\$unit.jpg
      kill \$pid; wait \$pid 2>/dev/null
    done" 2>&1 | tee pi/capture.log
  for unit in $NEED; do
    rm -f "pi/capture-unit$unit.jpg"
    scp -q $SSH_OPTS "$PI:tracker/capture-unit$unit.jpg" pi/ 2>/dev/null
  done
fi

FAILED=0
for unit in $UNITS; do
  if [ -s "pi/capture-unit$unit.jpg" ]; then
    echo "unit $unit: pi/capture-unit$unit.jpg ($(wc -c < "pi/capture-unit$unit.jpg") bytes)"
  else
    echo "unit $unit: no frame, see pi/capture.log"
    FAILED=1
  fi
done
exit $FAILED
