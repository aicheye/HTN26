# Moves the Pi and the robot onto another WiFi network, such as a phone hotspot, so the camera frames no longer
# cross the robot's slow access point and the laptop can keep its internet. Run from the HTN repo root while the
# laptop is still on the Sesame-Controller WiFi:
#   sh pi/join-wifi.sh "<network name>" "<password>"
# The network must offer 2.4 GHz (iPhone: turn on "Maximize Compatibility"), because the robot has no 5 GHz radio.
# What it does:
#  1. Robot: asks the firmware to join the network as well. It keeps its own access point. The firmware forgets
#     this when the robot restarts, so run the script again after a robot restart.
#  2. Pi: adds the network to /boot/wpa_supplicant.conf with the highest priority and reboots the Pi.
# Afterwards join the same network on the laptop. The robot is then at ws://sesame-robot.local:81.
SSID=$1; PASS=$2
[ -n "$SSID" ] && [ -n "$PASS" ] || { echo "usage: sh pi/join-wifi.sh \"<network name>\" \"<password>\""; exit 1; }
. "$(dirname "$0")/common.sh"

case "$(curl -s -m 3 http://192.168.4.1/api/wifi/status)" in
  *'"connected":true'*"\"ssid\":\"$SSID\""*) echo "robot: already on $SSID" ;;
  *)
    echo "robot: joining $SSID"
    curl -s -m 5 -X POST http://192.168.4.1/api/wifi/connect --data-urlencode "ssid=$SSID" --data-urlencode "password=$PASS" || echo "robot not reachable at 192.168.4.1"
    echo
    for i in 1 2 3 4 5 6 7 8; do
      status=$(curl -s -m 3 http://192.168.4.1/api/wifi/status); echo "  $status"
      case "$status" in *'"connected":true'*) break;; esac
      sleep 2
    done ;;
esac

# The network name and password can contain quotes and spaces (a first run with "Sean's iPhone" broke the remote
# command), so they travel to the Pi in files and never inside a shell command line.
echo "pi: adding $SSID to /boot/wpa_supplicant.conf (sudo asks for the Pi password, qnxuser)"
TMP=$(mktemp -d)
printf '    ssid="%s"\n' "$SSID" > "$TMP/wifi-ssid-line.txt"
printf '\nnetwork={\n    ssid="%s"\n    key_mgmt=WPA-PSK\n    psk="%s"\n    priority=30\n}\n' "$SSID" "$PASS" > "$TMP/wifi-block.conf"
scp -q $SSH_OPTS "$TMP/wifi-ssid-line.txt" "$TMP/wifi-block.conf" $PI: || exit 1
rm -rf "$TMP"
ssh -t $SSH_OPTS $PI 'if grep -qFf wifi-ssid-line.txt /boot/wpa_supplicant.conf; then echo "already listed"; else sudo sh -c "cat wifi-block.conf >> /boot/wpa_supplicant.conf" && echo added; fi; rm -f wifi-block.conf wifi-ssid-line.txt; echo "rebooting the Pi"; sudo shutdown -b'
echo "now join $SSID on this laptop, wait a minute, then: ping qnxpi78.local and ping sesame-robot.local"
