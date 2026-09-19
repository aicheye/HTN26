# Collects why the Pi is not joining another WiFi network. Run from the HTN repo root while the laptop is on the
# same network as the Pi (Sesame-Controller):
#   sh pi/wifi-check.sh
# It writes pi/wifi-check.txt. Every step has a time limit, so it finishes in under a minute.
. "$(dirname "$0")/common.sh"
ssh $SSH_OPTS $PI '
echo "== uptime (a recent boot shows whether the reboot happened)"; uptime
echo "== wifi interfaces"; ifconfig | grep -E "^[a-z]|inet "
echo "== how wpa_supplicant was started"; pidin ar 2>/dev/null | grep -i wpa | grep -v grep
echo "== networks in /boot/wpa_supplicant.conf"; grep -n -E "ssid=|priority=|key_mgmt=" /boot/wpa_supplicant.conf
echo "== other copies of the config"; ls -l /etc/wpa_supplicant.conf /var/etc/wpa_supplicant.conf /data/var/etc/wpa_supplicant.conf 2>/dev/null
for i in bcm0 bcm1; do
  echo "== wpa_cli status on $i"; wpa_cli -i $i status 2>&1 | head -12
  echo "== networks wpa_supplicant loaded on $i"; wpa_cli -i $i list_networks 2>&1 | head -10
done
echo "== scan"; wpa_cli -i bcm0 scan 2>&1 | tail -1; sleep 5
echo "== what the Pi can see (all networks, strongest first)"; wpa_cli -i bcm0 scan_results 2>&1 | sort -t"	" -k3 -n -r | head -25
' > pi/wifi-check.txt 2>&1
echo "wrote pi/wifi-check.txt ($(wc -l < pi/wifi-check.txt) lines)"
