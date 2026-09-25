# Installs the offline AI workshop packages (OpenCV 4.12, TFLite, numpy) on the QNX Pi,
# then compiles a small OpenCV ArUco program to confirm the toolchain works.
# Run from the laptop while on the Sesame-Controller WiFi:
#   scp pi/setup.sh qnxuser@qnxpi78.local: && ssh -t qnxuser@qnxpi78.local 'sh setup.sh 2>&1 | tee setup.log'; scp qnxuser@qnxpi78.local:setup.log pi/
set -u
WORK=$HOME/aiworkshop

echo "== unpack"
if [ ! -d "$WORK/apks" ]; then
  tar -xzf /boot/aiworkshop.tar.gz -C "$HOME" || exit 1
fi
ls "$WORK"; ls "$WORK/apks" | wc -l
echo "== apk_list.txt (first 5 lines)"; head -5 "$WORK/apks/apk_list.txt"

echo "== install (asks for the qnxuser password if sudo needs one)"
sudo apk add --allow-untrusted --no-network "$WORK"/apks/*.apk 2>&1 | tail -15

echo "== installed"
apk info 2>/dev/null | grep -i -E "opencv|tflite|numpy" | sort | tr '\n' ' '; echo
for m in numpy cv2 tflite_runtime; do printf "%s: " $m; python3 -c "import $m; print(getattr($m,'__version__','ok'))" 2>&1 | tail -1; done

echo "== opencv headers"
ls /usr/include/opencv4/opencv2 2>/dev/null | grep -i -E "aruco|objdetect|videoio" 
pkg-config --modversion opencv4 2>&1

echo "== compile test"
mkdir -p "$HOME/tmp" && cd "$HOME/tmp"
cat > aruco_test.cpp <<'CPP'
#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <iostream>
int main() {
  cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  cv::Mat marker;
  cv::aruco::generateImageMarker(dict, 7, 200, marker, 1);
  cv::Mat scene(400, 400, CV_8UC1, cv::Scalar(255));
  marker.copyTo(scene(cv::Rect(100, 100, 200, 200)));
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::ArucoDetector(dict).detectMarkers(scene, corners, ids);
  std::cout << "OpenCV " << CV_VERSION << ", detected " << ids.size() << " marker(s)";
  if (!ids.empty()) std::cout << ", id " << ids[0];
  std::cout << std::endl;
  return ids.size() == 1 && ids[0] == 7 ? 0 : 1;
}
CPP
clang++ -std=c++17 aruco_test.cpp -o aruco_test $(pkg-config --cflags --libs opencv4 2>/dev/null || echo "-I/usr/include/opencv4 -lopencv_core -lopencv_imgproc -lopencv_objdetect") 2>&1 | tail -10
./aruco_test; echo "aruco_test exit=$?"

echo "== workshop sample code"
for d in projects repos solutions; do echo "-- ~/$d"; find "$HOME/$d" -maxdepth 2 2>/dev/null | head -40; done

echo "== camera config"
ls /system/etc/sensor* /etc/system/config/sensor* /usr/share/sensor* 2>/dev/null | head -20
pidin ar 2>/dev/null | grep -i sensor | grep -v grep
