# One command for the live view. Runs on the laptop while on the Sesame-Controller WiFi, from the HTN repo root:
#   sh pi/live.sh [unit] [floor width cm] [floor height cm] [lens code | -] [robot marker height cm]
#   defaults: 3 63 63, the saved lens code, 10.5 cm (measured on the standing robot)
# Serves the web page, opens it in the browser, starts object detection on the laptop when it is set up, then builds
# and runs the tracker on the Pi.
# Ctrl-C stops the tracker and the web server. The tracker's output is also kept in pi/tracker-live.log.
UNIT=${1:-3}; W=${2:-63}; H=${3:-63}; LENS=${4:--}; ROBOT_HEIGHT=${5:-10.5}
PORT=5500
cd "$(dirname "$0")/.." || exit 1

python3 -m http.server $PORT --bind 127.0.0.1 --directory pi/client >/dev/null 2>&1 &
SERVER=$!
# Object detection for the page: rescans every few seconds and writes pi/client/objects.json. Needs sh vision/setup.sh.
SCAN=
rm -f pi/client/objects.json
if [ -x vision/.venv/bin/python ] && [ -f vision/models/mobile_sam.encoder.onnx ]; then
  vision/.venv/bin/python -u vision/scan.py --watch > vision/scan.log 2>&1 &
  SCAN=$!
  echo "object detection: on, log in vision/scan.log"
else
  echo "object detection: off. Run sh vision/setup.sh once, with internet, to turn it on"
fi
trap 'kill $SERVER $SCAN 2>/dev/null' EXIT INT TERM

. pi/common.sh
URL="http://localhost:$PORT/demo.html?unit=$UNIT&host=$PI_HOST"
echo "live view: $URL"
(xdg-open "$URL" >/dev/null 2>&1 || open "$URL" >/dev/null 2>&1) &

sh pi/start-tracker.sh $UNIT $W $H $LENS $ROBOT_HEIGHT
