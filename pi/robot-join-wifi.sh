# Tells the robot to join a WiFi network, such as a phone hotspot, and to remember it. With the Pi on the same
# network, camera frames and robot commands no longer cross the robot's slow access point.
# Run from the HTN repo root while the laptop's WiFi is on Sesame-Controller:
#   sh pi/robot-join-wifi.sh "<network name>" "<password>"
# The network must offer 2.4 GHz (iPhone: turn on "Maximize Compatibility"), because the robot has no 5 GHz radio.
# The robot keeps its own access point as well. With the firmware in this repo it joins the remembered network at
# every boot and retries once a minute while nobody is on its access point. Older firmware ignores "remember" and
# forgets the network on restart. To make it forget: curl -X POST http://192.168.4.1/api/wifi/forget
# The robot's address on the new network is saved in pi/robot-host, which bridge/bridge.mjs reads.
SSID=$1; PASS=$2
[ -n "$SSID" ] && [ -n "$PASS" ] || { echo "usage: sh pi/robot-join-wifi.sh \"<network name>\" \"<password>\""; exit 1; }
cd "$(dirname "$0")/.." || exit 1
ROBOT=http://192.168.4.1
field() { sed -n "s/.*\"$1\":\"\{0,1\}\([^\",}]*\).*/\1/p"; }

status=$(curl -s -m 3 $ROBOT/api/wifi/status)
[ -n "$status" ] || { echo "robot not reachable at 192.168.4.1. Join Sesame-Controller on this laptop first"; exit 1; }
# The network name and password can contain quotes and spaces, so curl encodes them.
curl -s -m 5 -X POST $ROBOT/api/wifi/connect --data-urlencode "ssid=$SSID" --data-urlencode "password=$PASS" --data-urlencode "remember=1" >/dev/null
echo "robot: joining $SSID"
# Joining a network on another channel moves the robot's access point there too, which drops this laptop for a
# few seconds. The status requests simply fail during that time.
for i in 1 2 3 4 5 6 7 8 9 10 11 12; do
  sleep 2
  status=$(curl -s -m 3 $ROBOT/api/wifi/status)
  case "$status" in
    *'"connected":true'*) break ;;
    *'"connecting":false'*'"lastError"'*) echo "robot: $(echo "$status" | field lastError)"; exit 1 ;;
  esac
done
case "$status" in *'"connected":true'*) ;; *) echo "robot: no answer about the result. Last status: $status"; exit 1 ;; esac
ip=$(echo "$status" | field ip)
echo "$ip" > pi/robot-host
echo "robot: on $SSID at $ip (saved in pi/robot-host)"
case "$status" in
  *'"remembered"'*) echo "robot: remembers $SSID and joins it at every boot" ;;
  *) echo "robot: this firmware does not remember networks. Flash firmware/ to keep it across restarts" ;;
esac
echo "now join $SSID on this laptop and start: sh pi/live.sh"
