# Sets up a fresh QNX workshop SD card once the Pi has booted from it. Run on the laptop while on the Pi's WiFi,
# from the HTN repo root:
#   sh pi/provision.sh [lens code for unit 3]      default 455, the code pi/run-focus.sh found on the first card
# Before the first boot, the card's boot partition needs two edits, made with the card in the laptop:
#   wpa_supplicant.conf  one network block for the WiFi the Pi joins
#   network              HOSTNAME=qnxpi78, the name every script and pi/API.md use
# What this script does. It asks for the Pi password (qnxuser) for the key, and again for sudo:
#  1. Forgets the previous card's ssh host key, which otherwise makes ssh refuse to connect to the same name.
#  2. Installs the laptop's ssh key (pi/setup-key.sh), so the other scripts stop asking for the password.
#  3. Installs OpenCV from /boot/aiworkshop.tar.gz and compiles a test program (pi/setup.sh). Log: pi/setup.log
#  4. Saves the lens code for unit 3. Unit 4 has none yet: sh pi/run-focus.sh 4
#  5. Takes one frame from each camera (pi/capture.sh).
LENS3=${1:-455}
. "$(dirname "$0")/common.sh"
cd "$(dirname "$0")/.." || exit 1

for name in qnxpi78.local qnxpi78 "$PI_HOST"; do ssh-keygen -R "$name" >/dev/null 2>&1; done
ssh $SSH_OPTS -o StrictHostKeyChecking=accept-new $PI true || { echo "cannot log in to the Pi at $PI_HOST"; exit 1; }

sh pi/setup-key.sh || exit 1
. pi/common.sh  # again, so that SSH_OPTS picks up the new key

scp -q $SSH_OPTS pi/setup.sh $PI: || exit 1
ssh -t $SSH_OPTS $PI 'sh setup.sh 2>&1 | tee setup.log'
scp -q $SSH_OPTS $PI:setup.log pi/
grep -q "aruco_test exit=0" pi/setup.log || { echo "OpenCV is not working on the Pi, see pi/setup.log"; exit 1; }

ssh $SSH_OPTS $PI "echo $LENS3 > lens-unit3.txt" && echo "lens code $LENS3 saved for unit 3"
sh pi/capture.sh
