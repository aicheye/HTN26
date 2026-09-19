# Reports how the cameras are configured on the QNX Pi. Every command here exits on its own.
# Run from the laptop while on the Sesame-Controller WiFi:
#   ssh qnxuser@qnxpi78.local 'sh -s' < pi/camera-survey.sh > pi/camera-survey.txt 2>&1
echo "== stop leftover viewfinder processes"; slay -f camera_example3_viewfinder 2>&1; echo "slay exit=$?"
echo "== processes"; pidin -F "%a %n %A" 2>/dev/null | grep -i -E "sensor|camera|screen" | grep -v grep
echo "== /dev/sensor"; ls -la /dev/sensor 2>&1 | head
echo "== sensor_rpi5.conf"; cat /etc/config/sensor/sensor_rpi5.conf 2>/dev/null || cat /usr/etc/config/sensor/sensor_rpi5.conf
echo "== rpi5_camera_module3.conf"; cat /usr/etc/config/sensor/rpi5/rpi5_camera_module3.conf
echo "== camera startup section"; sed -n '/post_start.~30.sensor_framework/,/post_start.~96/p' /system/etc/startup/post_startup.sh
echo "== system log (camera)"; slog2info 2>/dev/null | grep -i -E "sensor|camera|imx708|csi|cfe" | tail -40
echo "== camera_api.h units and frame types"; grep -n -E "CAMERA_UNIT_[0-9]+|CAMERA_FRAMETYPE_[A-Z0-9_]+ *=" /usr/include/camera/camera_api.h | head -40
echo "== opencv videoio backends"
cat > /tmp/b.cpp <<'CPP'
#include <opencv2/videoio/registry.hpp>
#include <iostream>
int main(){ for (auto b : cv::videoio_registry::getCameraBackends()) std::cout << cv::videoio_registry::getBackendName(b) << " "; std::cout << std::endl; }
CPP
clang++ -std=c++17 /tmp/b.cpp -o /tmp/b $(pkg-config --cflags --libs opencv4) 2>&1 | tail -3; /tmp/b
echo "== done"
