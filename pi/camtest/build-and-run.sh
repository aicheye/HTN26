# Runs on the Pi. Builds camtest and tries both Camera Module 3 units.
echo "== /dev/sensor"; ls /dev/sensor
echo "== groups"; id
echo "== build"
clang++ -std=c++17 -O2 camtest.cpp -o camtest $(pkg-config --cflags --libs opencv4) -lcamapi 2>&1 | head -40
[ -x ./camtest ] || { echo "build failed"; exit 1; }
for unit in 3 4; do echo "== unit $unit"; ./camtest $unit 5; echo "exit=$?"; done
echo "== done"
