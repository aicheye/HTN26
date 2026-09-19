# One command for the live view. Runs on the laptop while on the Sesame-Controller WiFi, from the HTN repo root:
#   sh pi/live.sh [unit] [floor width cm] [floor height cm] [lens code]      defaults: 3 63 63, saved lens code
# Serves the web page, opens it in the browser, then builds and runs the tracker on the Pi.
# Ctrl-C stops the tracker and the web server. The tracker's output is also kept in pi/tracker-live.log.
UNIT=${1:-3}; W=${2:-63}; H=${3:-63}; LENS=${4:-}
PORT=5500
cd "$(dirname "$0")/.." || exit 1

python3 -m http.server $PORT --bind 127.0.0.1 --directory pi/client >/dev/null 2>&1 &
SERVER=$!
trap 'kill $SERVER 2>/dev/null' EXIT INT TERM

URL="http://localhost:$PORT/demo.html?unit=$UNIT"
echo "live view: $URL"
(xdg-open "$URL" >/dev/null 2>&1 || open "$URL" >/dev/null 2>&1) &

sh pi/start-tracker.sh $UNIT $W $H $LENS
