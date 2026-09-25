# One command for the whole live system. Runs on the laptop while on the Sesame-Controller WiFi, from the HTN repo root:
#   sh pi/live.sh [unit] [floor width cm] [floor height cm] [lens code | -] [robot marker height cm]
#   defaults: 3 63.5 63.5, the saved lens code, 10.5 cm (measured on the standing robot)
# It starts, in this order:
#   1. the bridge (bridge/bridge.mjs): tracker and robot in, the frontend's WebSocket out on port 8080
#   2. object detection (vision/scan.py --watch), when sh vision/setup.sh has been run. It posts to the bridge
#   3. the web UI (vite dev server) on http://localhost:5173, connected to the bridge, and opens it in the browser
#   4. the tracker on the Pi, in the foreground. Its output is also kept in pi/tracker-live.log
# Ctrl-C stops everything. Logs: bridge/bridge.log, vision/scan.log, ui.log.
UNIT=${1:-3}; W=${2:-63.5}; H=${3:-63.5}; LENS=${4:--}; ROBOT_HEIGHT=${5:-10.5}
cd "$(dirname "$0")/.." || exit 1
[ -d node_modules ] || { echo "the web UI is not installed. Run once, with internet: npm install"; exit 1; }
[ -d bridge/node_modules ] || { echo "the bridge is not installed. Run once, with internet: npm --prefix bridge install"; exit 1; }

# bridge/.env holds GROQ_API_KEY for voice control (see bridge/.env.example).
node --env-file-if-exists=bridge/.env bridge/bridge.mjs > bridge/bridge.log 2>&1 &
BRIDGE=$!
SCAN=
if [ -x vision/.venv/bin/python ] && [ -f vision/models/mobile_sam.encoder.onnx ]; then
  vision/.venv/bin/python -u vision/scan.py --watch > vision/scan.log 2>&1 &
  SCAN=$!
  echo "object detection: on, log in vision/scan.log"
else
  echo "object detection: off. Run sh vision/setup.sh once, with internet, to turn it on"
fi
VITE_SOURCE=ws VITE_WS_URL=ws://localhost:8080/ws npx vite --port 5173 --strictPort > ui.log 2>&1 &
UI=$!
trap 'kill $BRIDGE $SCAN $UI 2>/dev/null' EXIT INT TERM

echo "web UI: http://localhost:5173   manual controller: http://localhost:8080/"
(sleep 3; xdg-open "http://localhost:5173" >/dev/null 2>&1 || open "http://localhost:5173" >/dev/null 2>&1) &

sh pi/start-tracker.sh $UNIT $W $H $LENS $ROBOT_HEIGHT
