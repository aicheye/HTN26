# Shared by the pi/*.sh scripts: how to reach the Pi.
# With the key from pi/setup-key.sh there are no password prompts. Without it, every ssh and scp asks.
# The scripts used to share one ssh connection to avoid repeated prompts. That shared connection went stale
# whenever the laptop changed WiFi network, and later commands then hung on it for ever while a plain ssh
# still worked. A key needs no sharing. Dead connections are also detected within 15 seconds now.
PI=qnxuser@qnxpi78.local
PI_KEY="$HOME/.ssh/htn_pi"
SSH_OPTS="-o ControlMaster=no -o ControlPath=none -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3"
[ -f "$PI_KEY" ] && SSH_OPTS="$SSH_OPTS -i $PI_KEY -o IdentitiesOnly=yes"
