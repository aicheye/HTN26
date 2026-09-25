# Prints what the QNX Pi has installed. Run from the laptop while on the Sesame-Controller WiFi:
#   ssh qnxuser@qnxpi78.local 'sh -s' < pi/survey.sh > pi/survey.txt 2>&1
echo "== system"; uname -a; df -h 2>/dev/null
echo "== network"; ifconfig 2>/dev/null | grep -E "^[a-z]|inet "
echo "== robot reachable"; ping -c 2 192.168.4.1 2>&1 | tail -2
echo "== tools"; for t in python3 pip3 clang clang++ gcc g++ qcc cc make cmake git curl node; do printf "%s: " $t; command -v $t || echo none; done
echo "== python"; python3 --version 2>&1
for m in numpy cv2 tflite_runtime websockets websocket; do printf "%s: " $m; python3 -c "import $m; print(getattr($m,'__version__','ok'))" 2>&1 | tail -1; done
echo "== apk packages (vision)"; apk info 2>/dev/null | grep -i -E "opencv|tflite|numpy|camera|sensor|websocket|python3$" | sort
echo "== apk repositories"; cat /etc/apk/repositories 2>/dev/null
echo "== boot partition and workshop archive"; ls -d /boot /fs/* 2>/dev/null; find / -name "aiworkshop*" -maxdepth 4 2>/dev/null
echo "== camera"; ls /dev | grep -i -E "cam|sensor|video|i2c"; pidin ar 2>/dev/null | grep -i -E "sensor|camera"; command -v camera_example3_viewfinder
ls /usr/lib 2>/dev/null | grep -i -E "libcamapi|libopencv_(core|aruco|videoio)" | head
echo "== home"; ls -la ~
