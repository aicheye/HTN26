# Collects why the Pi is not joining another WiFi network. Run from the HTN repo root while the laptop is on the
# same network as the Pi (Sesame-Controller):
#   sh pi/wifi-check.sh
# It writes pi/wifi-check.txt. Every step has a time limit, so it finishes in under a minute.
. "$(dirname "$0")/common.sh"
# QNX has no uptime or wpa_cli. Its clock starts at 1970 on every boot (there is no clock chip), so "date" is the
# time since boot. The WiFi service logs to the system log, which only root can read, so sudo asks for the Pi
# password (qnxuser).
ssh -t $SSH_OPTS $PI '
echo "== time since boot (the clock starts at 1970-01-01 00:00 on every boot)"; date
echo "== wifi address"; ifconfig bcm0 | grep "inet "
echo "== networks in /boot/wpa_supplicant.conf"; grep -n -E "ssid=|priority=" /boot/wpa_supplicant.conf
echo "== wifi service in the system log (last 60 lines)"
sudo slog2info 2>/dev/null | grep -i -E "wpa|ssid|bcm|wlan|assoc|auth|scan" | tail -60
' 2>&1 | tr -d "\r" > pi/wifi-check.txt
echo "wrote pi/wifi-check.txt ($(wc -l < pi/wifi-check.txt) lines)"
