# Shared by the pi/*.sh scripts: how to reach the Pi.
# With the key from pi/setup-key.sh there are no password prompts. Without it, every ssh and scp asks.
# The scripts used to share one ssh connection to avoid repeated prompts. That shared connection went stale
# whenever the laptop changed WiFi network, and later commands then hung on it for ever while a plain ssh
# still worked. A key needs no sharing. Dead connections are also detected within 15 seconds now.
# The Pi's address. Its name does not resolve on every network (it fails on the iPhone hotspot), so pi/host can
# hold an address to use, for example: echo 172.20.10.4 > pi/host
PI_HOST=$(cat "$(dirname "$0")/host" 2>/dev/null || echo qnxpi78.local)
PI=qnxuser@$PI_HOST
PI_KEY="$HOME/.ssh/htn_pi"
SSH_OPTS="-o ControlMaster=no -o ControlPath=none -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3"
[ -f "$PI_KEY" ] && SSH_OPTS="$SSH_OPTS -i $PI_KEY -o IdentitiesOnly=yes"
