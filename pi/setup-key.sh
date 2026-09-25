# One-time setup so the pi/*.sh scripts stop asking for the Pi password. Run on the Sesame-Controller WiFi:
#   sh pi/setup-key.sh
# Creates the key ~/.ssh/htn_pi on this laptop (no passphrase, used only for the Pi) and adds its public half
# to the Pi's authorized_keys. Asks for the Pi password one last time.
. "$(dirname "$0")/common.sh"
[ -f "$PI_KEY" ] || ssh-keygen -q -t ed25519 -N "" -C "htn laptop" -f "$PI_KEY" || exit 1
ssh -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=8 $PI 'mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys' < "$PI_KEY.pub" || exit 1
if ssh -o BatchMode=yes -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=8 -i "$PI_KEY" -o IdentitiesOnly=yes $PI true; then
  echo "key login works. The pi/*.sh scripts will not ask for a password any more."
else
  echo "the key was installed but key login did not work. The scripts still work, with password prompts."
fi
