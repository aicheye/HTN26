// Top-down robot tracker for QNX. Reads one Camera Module 3, detects ArUco markers (DICT_4X4_50),
// and streams the robot's pose as one JSON object per line over TCP.
//   ./tracker <unit> <floor width cm> <floor height cm> [lens code | -] [robot marker height cm]
// The robot marker height is the distance from the floor to the robot's marker. Without it the height comes from
// the marker's apparent size, which recordings showed reading several cm low, and that shifts x, y when the camera
// is not straight above.
// QNX's driver for this camera has no autofocus (tried on the Pi: error 22, no manual focus steps).
// A lens code (0 to 1023, see lens.h) sets the focus motor directly. pi/run-focus.sh finds the best code and
// saves it on the Pi. Without a lens code argument the tracker uses the saved one.
// Marker 0 is on the robot. Marker 5 is taped flat on the floor against the base of the SO-101 arm.
// Markers 1 to 4 are taped flat on the floor at the corners of a
// rectangle: 1 = (0, 0), 2 = (W, 0), 3 = (W, H), 4 = (0, H).
// The camera may move, and it never has to see all four floor markers at once. The floor markers' centres
// are known from the two measurements. What is learned is how each one is rotated on the floor: any steady
// frame with two or more floor markers gives the floor's x and y directions (from where the markers lie
// relative to each other, using their known 8 cm size for distance), and from those the floor position of
// every corner of the markers in view. One frame with three markers settles whether markers 1, 2, 3, 4 run
// clockwise or counter-clockwise. After that, every frame solves the camera's position from
// whichever floor markers are visible (one is enough, more is steadier). Each tracked marker's
// height then comes from its known printed size, and its x, y from where its pixel ray crosses
// that height. Frames with no floor marker in view report pixel positions only.
//
// Ports: 9000 + unit streams the JSON lines over plain TCP. 8000 + unit is a small HTTP server
// for browsers and other clients (all responses allow cross-origin requests):
//   GET /events             the same JSON, one server-sent event per frame (EventSource in a browser)
//   GET /state.json         the latest JSON once
//   GET /frame.jpg[?w=960]  current colour frame, optionally scaled down to the given width
//   GET /annotated.jpg      the same with detected markers drawn
//   GET /record/start[?every=2&w=0]  save frames on the Pi under ~/recordings/rec-NNN (every Nth frame, optional width)
//   GET /record/stop        finish the recording. /record/status reports progress. pi/record.sh drives these.
// Each JSON object carries the camera pose for that frame, so a pixel found in /frame.jpg can be
// converted to floor centimetres by the client. See pi/API.md.
#include <camera/camera_api.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "lens.h"

static const int ROBOT_ID = 0;
static const int ARM_BASE_ID = 5;
static const float TRACKED_MARKER_CM = 3.6f;  // printed side length of markers 0 and 5
static const float FLOOR_MARKER_CM = 8.0f;    // printed side length of markers 1 to 4

// Camera Module 3, standard lens, 2304x1296 mode (2x2 binned): 4.74 mm / (2 * 1.4 um) = 1693 px.
// The wide-angle module is 2.75 mm, which gives 982 px.
static const double FOCAL_PX = 1693.0;

// Markers are searched in a half-size copy of the frame, which is about 4 times faster than full size
// (measured on the Pi at full size: about 16 frames per second against the camera's 30). The corners found
// are then refined in the full-size frame, so positions keep full-resolution accuracy.
static const int DETECT_SHRINK = 2;

static std::mutex frameMutex;
static std::condition_variable frameReady;
static cv::Mat latestGray;
static cv::Mat latestUv;  // NV12 colour plane of the same frame, for saved and served images

// Runs on a camera library thread. The buffer is only valid during the call, so copy it out.
static void onFrame(camera_handle_t, camera_buffer_t* buf, void*) {
  if (buf->frametype != CAMERA_FRAMETYPE_NV12) return;
  const auto& d = buf->framedesc.nv12;
  std::lock_guard<std::mutex> lock(frameMutex);
  cv::Mat(d.height, d.width, CV_8UC1, buf->framebuf, d.stride).copyTo(latestGray);
  cv::Mat(d.height / 2, d.width / 2, CV_8UC2, buf->framebuf + d.uv_offset, d.uv_stride).copyTo(latestUv);
  frameReady.notify_one();
}

static void onStatus(camera_handle_t, camera_devstatus_t, uint16_t, void*) {}

// What the HTTP threads read. The tracking loop writes it once per frame. HTTP requests are handled on their
// own threads, so a browser that connects and sends nothing, or a slow image transfer, never pauses tracking.
struct Shared {
  std::mutex mutex;
  cv::Mat gray, uv;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  std::string state = "{}\n";
  std::vector<int> eventClients;
  std::vector<std::string> recentLog, unsentLog;
};
static Shared shared;

// Saves camera frames on the Pi while a recording is on. The tracking loop only queues frames. A writer thread
// converts and writes them, and frames are dropped when the SD card cannot keep up, so tracking never waits.
// Each recording is a folder of JPEGs plus states.jsonl, one line per saved frame with the tracker state.
struct Recorder {
  std::mutex mutex;
  std::condition_variable wake;
  struct Item { cv::Mat gray, uv; long index; std::string state; };
  std::deque<Item> queue;
  bool recording = false;
  std::string dir;
  int every = 2, width = 0;
  long seen = 0, saved = 0, dropped = 0;

  std::string statusJson() {  // call with the mutex held
    return std::string("{\"recording\":") + (recording ? "true" : "false") + ",\"dir\":\"" + dir + "\",\"frames\":" +
           std::to_string(saved) + ",\"queued\":" + std::to_string(queue.size()) + ",\"dropped\":" + std::to_string(dropped) + "}\n";
  }

  std::string start(int everyNth, int scaledWidth) {
    std::lock_guard<std::mutex> lock(mutex);
    if (recording) return statusJson();
    const char* home = std::getenv("HOME");
    std::string base = std::string(home ? home : ".") + "/recordings";
    mkdir(base.c_str(), 0755);
    for (int n = 1; n < 10000; n++) {  // the Pi has no clock, so recordings are numbered
      char name[32];
      std::snprintf(name, sizeof(name), "/rec-%03d", n);
      dir = base + name;
      if (mkdir(dir.c_str(), 0755) == 0) break;
    }
    every = everyNth < 1 ? 1 : everyNth;
    width = scaledWidth;
    seen = saved = dropped = 0;
    recording = true;
    return statusJson();
  }

  std::string stop() {
    std::unique_lock<std::mutex> lock(mutex);
    recording = false;
    wake.wait_for(lock, std::chrono::seconds(10), [this] { return queue.empty(); });  // let the writer finish
    return statusJson();
  }

  void offer(const cv::Mat& gray, const cv::Mat& uv, const std::string& state) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!recording || seen++ % every != 0) return;
    if (queue.size() >= 8) { dropped++; return; }
    queue.push_back({gray, uv, seen - 1, state});
    wake.notify_all();
  }

  void writerLoop() {
    for (;;) {
      Item item;
      std::string folder;
      int scaledWidth;
      {
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait(lock, [this] { return !queue.empty(); });
        item = queue.front();
        folder = dir;
        scaledWidth = width;
      }
      cv::Mat bgr;
      cv::cvtColorTwoPlane(item.gray, item.uv, bgr, cv::COLOR_YUV2BGR_NV12);
      if (scaledWidth >= 160 && scaledWidth < bgr.cols) cv::resize(bgr, bgr, cv::Size(scaledWidth, bgr.rows * scaledWidth / bgr.cols), 0, 0, cv::INTER_AREA);
      char name[40];
      std::snprintf(name, sizeof(name), "frame-%06ld.jpg", item.index);
      cv::imwrite(folder + "/" + name, bgr, {cv::IMWRITE_JPEG_QUALITY, 92});
      if (FILE* states = std::fopen((folder + "/states.jsonl").c_str(), "a")) {
        std::fprintf(states, "{\"file\":\"%s\",\"state\":%s}\n", name, item.state.substr(0, item.state.find('\n')).c_str());
        std::fclose(states);
      }
      std::lock_guard<std::mutex> lock(mutex);
      queue.pop_front();
      saved++;
      wake.notify_all();
    }
  }
};
static Recorder recorder;

// Everything the tracker prints also goes to web clients: /events sends each line as a "log" event, and a
// client that connects later first receives the recent lines.

static void say(const char* format, ...) {
  char text[900];
  va_list args;
  va_start(args, format);
  std::vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  std::string line(text);
  while (!line.empty() && line.back() == '\n') line.pop_back();
  std::printf("%s\n", line.c_str());
  std::fflush(stdout);
  std::lock_guard<std::mutex> lock(shared.mutex);
  shared.recentLog.push_back(line);
  if (shared.recentLog.size() > 40) shared.recentLog.erase(shared.recentLog.begin());
  shared.unsentLog.push_back(line);
}

static std::string logEvent(const std::string& line) { return "event: log\ndata: " + line + "\n\n"; }

static int listenOn(int port) {
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  int yes = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port);
  if (bind(fd, (sockaddr*)&addr, sizeof(addr)) != 0 || listen(fd, 4) != 0) return -1;
  fcntl(fd, F_SETFL, O_NONBLOCK);
  return fd;
}

static void sendAll(int fd, const char* data, size_t size) {
  while (size > 0) {
    ssize_t n = send(fd, data, size, 0);
    if (n <= 0) return;
    data += n;
    size -= n;
  }
}

// Answers one HTTP request on its own thread.
static void handleHttp(int fd) {
  fcntl(fd, F_SETFL, 0);  // the accepted socket may inherit non-blocking mode
  timeval timeout{2, 0};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  char request[2048] = {0};
  if (recv(fd, request, sizeof(request) - 1, 0) <= 0) {
    close(fd);
    return;
  }
  std::string path(request);
  path = path.substr(0, path.find("\r\n"));
  const std::string common = "Access-Control-Allow-Origin: *\r\nCache-Control: no-cache\r\n";

  if (path.find("GET /events") == 0) {
    std::string header = "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n" + common + "Connection: keep-alive\r\n\r\n";
    sendAll(fd, header.data(), header.size());
    std::lock_guard<std::mutex> lock(shared.mutex);  // held while joining, so no log line is missed or sent twice
    for (const std::string& line : shared.recentLog) {
      std::string event = logEvent(line);
      sendAll(fd, event.data(), event.size());
    }
    fcntl(fd, F_SETFL, O_NONBLOCK);  // a slow browser must not stall the tracker
    shared.eventClients.push_back(fd);
    return;
  }

  cv::Mat gray, uv;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  std::string state;
  {
    std::lock_guard<std::mutex> lock(shared.mutex);
    gray = shared.gray;  // shares the pixels. The tracking loop never changes a frame after publishing it
    uv = shared.uv;
    corners = shared.corners;
    ids = shared.ids;
    state = shared.state;
  }
  auto query = [&](const char* key, int fallback) {
    size_t at = path.find(std::string(key) + "=");
    return at == std::string::npos ? fallback : std::atoi(path.c_str() + at + std::strlen(key) + 1);
  };
  std::string body, type = "application/json";
  if (path.find("GET /state.json") == 0) {
    body = state;
  } else if (path.find("GET /record/start") == 0) {
    body = recorder.start(query("every", 2), query("w", 0));
    say("recording started: %s", body.c_str());
  } else if (path.find("GET /record/stop") == 0) {
    body = recorder.stop();
    say("recording stopped: %s", body.c_str());
  } else if (path.find("GET /record/status") == 0) {
    std::lock_guard<std::mutex> lock(recorder.mutex);
    body = recorder.statusJson();
  } else if ((path.find("GET /frame.jpg") == 0 || path.find("GET /annotated.jpg") == 0) && !gray.empty()) {
    cv::Mat bgr;
    cv::cvtColorTwoPlane(gray, uv, bgr, cv::COLOR_YUV2BGR_NV12);
    if (path.find("GET /annotated.jpg") == 0) cv::aruco::drawDetectedMarkers(bgr, corners, ids);
    int width = query("w", 0);
    if (width >= 160 && width < bgr.cols) cv::resize(bgr, bgr, cv::Size(width, bgr.rows * width / bgr.cols), 0, 0, cv::INTER_AREA);
    std::vector<uchar> jpeg;
    cv::imencode(".jpg", bgr, jpeg, {cv::IMWRITE_JPEG_QUALITY, 80});
    body.assign(jpeg.begin(), jpeg.end());
    type = "image/jpeg";
  }
  std::string header = body.empty() ? "HTTP/1.1 404 Not Found\r\n" : "HTTP/1.1 200 OK\r\nContent-Type: " + type + "\r\n";
  header += common + "Content-Length: " + std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n";
  sendAll(fd, header.data(), header.size());
  sendAll(fd, body.data(), body.size());
  close(fd);
}

// Accepts HTTP connections for the whole run and gives each one its own thread.
static void serveHttp(int listener) {
  fcntl(listener, F_SETFL, 0);  // blocking accept
  for (;;) {
    int fd = accept(listener, nullptr, nullptr);
    if (fd >= 0) std::thread(handleHttp, fd).detach();
  }
}

// Sends one message to every client. A full send buffer skips the message. Any other error drops the client.
static void broadcast(std::vector<int>& clients, const std::string& message) {
  for (size_t i = 0; i < clients.size();) {
    if (send(clients[i], message.data(), message.size(), 0) < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      close(clients[i]);
      clients.erase(clients.begin() + i);
    } else {
      i++;
    }
  }
}

static cv::Point2f centerOf(const std::vector<cv::Point2f>& c) { return (c[0] + c[1] + c[2] + c[3]) / 4; }

// Camera pose over the floor for the current frame.
struct FloorCamera {
  bool valid = false;
  bool hasPrevious = false;
  cv::Vec3d rvec, tvec;  // last solved pose, used as the starting guess for the next frame
  cv::Matx33d K, Rt;   // intrinsics, and rotation from camera axes to floor axes
  cv::Vec3d t, C;      // floor-to-camera translation, and camera position in floor coordinates

  cv::Vec3d toFloor(const cv::Vec3d& inCamera) const { return Rt * (inCamera - t); }

  // Point where the ray through pixel p crosses the horizontal plane at floor-frame height z.
  cv::Point2f rayAtHeight(cv::Point2f p, double z) const {
    cv::Vec3d dir = Rt * (K.inv() * cv::Vec3d(p.x, p.y, 1));
    double k = (z - C[2]) / dir[2];
    return cv::Point2f(C[0] + k * dir[0], C[1] + k * dir[1]);
  }
};

int main(int argc, char** argv) {
  if (argc < 4) {
    std::printf("usage: tracker <unit> <floor width cm> <floor height cm> [lens code]\n");
    return 1;
  }
  int unit = std::atoi(argv[1]);
  float floorW = std::atof(argv[2]), floorH = std::atof(argv[3]);
  int lensCode = lensLoadCode(unit);  // saved by the focus tool
  double robotHeight = argc > 5 ? std::atof(argv[5]) : -1;
  if (argc > 4 && std::strcmp(argv[4], "-") != 0) {
    char* end = nullptr;
    long value = std::strtol(argv[4], &end, 10);
    if (*end != '\0' || value < 0 || value > 1023) {
      say("lens code must be a number from 0 to 1023, got \"%s\"\n", argv[4]);
      return 1;
    }
    lensCode = (int)value;
  }
  int port = 9000 + unit;
  std::signal(SIGPIPE, SIG_IGN);  // a client that disconnects must not kill the tracker

  int server = listenOn(port), httpServer = listenOn(8000 + unit);
  if (server < 0 || httpServer < 0) {
    say("cannot listen on ports %d and %d\n", port, 8000 + unit);
    return 2;
  }
  std::vector<int> clients;
  std::thread(serveHttp, httpServer).detach();
  std::thread([] { recorder.writerLoop(); }).detach();

  camera_handle_t handle = CAMERA_HANDLE_INVALID;
  camera_error_t err = camera_open((camera_unit_t)unit, CAMERA_MODE_RO, &handle);
  if (err != CAMERA_EOK) {
    say("unit %d: camera_open failed, error %d\n", unit, (int)err);
    return 3;
  }
  camera_set_vf_property(handle, CAMERA_IMGPROP_CREATEWINDOW, 0);
  err = camera_start_viewfinder(handle, onFrame, onStatus, nullptr);
  if (err != CAMERA_EOK) {
    say("unit %d: camera_start_viewfinder failed, error %d\n", unit, (int)err);
    return 4;
  }
  say("unit %d: poses on TCP port %d, frames on http port %d, floor %.0f x %.0f cm\n", unit, port, 8000 + unit,
              floorW, floorH);

  if (lensCode >= 0) {
    int error = lensApproach(unit, lensCode);
    say("lens code %d on %s: %s\n", lensCode, lensBusForUnit(unit), error ? std::strerror(error) : "set");
  }

  cv::aruco::ArucoDetector detector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
  const std::vector<cv::Point3f> floorCorners{{0, 0, 0}, {floorW, 0, 0}, {floorW, floorH, 0}, {0, floorH, 0}};
  FloorCamera cam;
  std::vector<cv::Point3f> floorMarkerCorners[5];  // floor position of each corner of markers 1 to 4, once learned
  cv::Point2d cornerSum[5][4] = {};        // running sums while a marker's corners are being learned
  int cornerSamples[5] = {};
  int handedness = 0;                      // +1: markers 1, 2, 3, 4 run counter-clockwise seen from above. -1: clockwise. 0: unknown
  cv::Point2f previousCentre[5];
  bool hadPrevious[5] = {};
  const int SAMPLES_NEEDED = 8;
  const float STEADY_MAX_MOVE_PX = 6;      // learn only from frames where the camera barely moved, to avoid motion blur
  const float fh = FLOOR_MARKER_CM / 2;
  const std::vector<cv::Point3f> floorMarkerShape{{-fh, fh, 0}, {fh, fh, 0}, {fh, -fh, 0}, {-fh, -fh, 0}};
  const cv::Mat noDistortion;
  const float half = TRACKED_MARKER_CM / 2;
  // Corner order matches ArUco's: top-left, top-right, bottom-right, bottom-left.
  const std::vector<cv::Point3f> markerCorners{{-half, half, 0}, {half, half, 0}, {half, -half, 0}, {-half, -half, 0}};
  auto start = std::chrono::steady_clock::now();
  auto lastReport = start, lastSnapshot = start - std::chrono::seconds(10);
  int framesSinceReport = 0;
  float fps = 0;

  for (;;) {
    cv::Mat gray, uv;
    {
      std::unique_lock<std::mutex> lock(frameMutex);
      if (!frameReady.wait_for(lock, std::chrono::seconds(2), [] { return !latestGray.empty(); })) {
        say("no frames for 2 s\n");
        continue;
      }
      gray = latestGray;
      uv = latestUv;
      // Let go of both buffers. copyTo() into a Mat that already has the right size reuses its memory, so a
      // buffer still held here would be overwritten by the next camera frame while this frame is in use.
      // Only the brightness plane was released before, and saved images got the colour of a later frame.
      latestGray = cv::Mat();
      latestUv = cv::Mat();
    }
    auto now = std::chrono::steady_clock::now();

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Mat small;
    cv::resize(gray, small, cv::Size(gray.cols / DETECT_SHRINK, gray.rows / DETECT_SHRINK), 0, 0, cv::INTER_AREA);
    detector.detectMarkers(small, corners, ids);
    for (auto& quad : corners) {
      for (auto& corner : quad) corner = corner * DETECT_SHRINK + cv::Point2f(0.5f, 0.5f) * (DETECT_SHRINK - 1);
      cv::cornerSubPix(gray, quad, cv::Size(5, 5), cv::Size(-1, -1),
                       cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.05));
    }

    auto setPose = [&](const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
      cv::Matx33d R;
      cv::Rodrigues(rvec, R);
      cam.Rt = R.t();
      cam.t = tvec;
      cam.C = cam.Rt * (-tvec);
      cam.rvec = rvec;
      cam.tvec = tvec;
      cam.valid = cam.hasPrevious = true;
    };
    cam.valid = false;
    cam.K = cv::Matx33d(FOCAL_PX, 0, gray.cols / 2.0, 0, FOCAL_PX, gray.rows / 2.0, 0, 0, 1);

    // Pose of each visible floor marker relative to the camera, from its known size.
    struct SeenFloorMarker { int id; cv::Matx33d R; cv::Vec3d t; };
    std::vector<SeenFloorMarker> seen;
    bool steady = true;
    bool visibleNow[5] = {};
    for (size_t i = 0; i < ids.size(); i++) {
      int id = ids[i];
      if (id < 1 || id > 4) continue;
      cv::Point2f centre = centerOf(corners[i]);
      if (!hadPrevious[id] || cv::norm(centre - previousCentre[id]) > STEADY_MAX_MOVE_PX) steady = false;
      previousCentre[id] = centre;
      visibleNow[id] = true;
      cv::Vec3d rvec, tvec;
      if (!cv::solvePnP(floorMarkerShape, corners[i], cam.K, noDistortion, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE)) continue;
      cv::Matx33d R;
      cv::Rodrigues(rvec, R);
      seen.push_back({id, R, tvec});
    }
    for (int id = 1; id <= 4; id++) hadPrevious[id] = visibleNow[id];

    // Learn marker corners from steady frames that show at least two floor markers.
    bool allLearned = cornerSamples[1] >= SAMPLES_NEEDED && cornerSamples[2] >= SAMPLES_NEEDED &&
                      cornerSamples[3] >= SAMPLES_NEEDED && cornerSamples[4] >= SAMPLES_NEEDED;
    if (!allLearned && steady && seen.size() >= 2) {
      cv::Vec3d normal(0, 0, 0);
      for (const auto& m : seen) normal += cv::Vec3d(m.R(0, 2), m.R(1, 2), m.R(2, 2));
      normal /= cv::norm(normal);
      // Floor x direction in camera coordinates, from every pair of markers, for one assumed handedness.
      auto floorX = [&](int sign, double* agreement) {
        cv::Vec3d sum(0, 0, 0);
        int pairs = 0;
        for (size_t a = 0; a < seen.size(); a++) {
          for (size_t b = a + 1; b < seen.size(); b++) {
            cv::Vec3d d = seen[b].t - seen[a].t;
            d -= normal * normal.dot(d);
            d /= cv::norm(d);
            cv::Point3f fa = floorCorners[seen[a].id - 1], fb = floorCorners[seen[b].id - 1];
            double fx = fb.x - fa.x, fy = fb.y - fa.y, length = std::hypot(fx, fy);
            sum += (fx / length) * d - (fy / length) * sign * normal.cross(d);
            pairs++;
          }
        }
        if (agreement) *agreement = cv::norm(sum) / pairs;  // 1 when every pair gives the same direction
        return sum / cv::norm(sum);
      };
      if (handedness == 0 && seen.size() >= 3) {
        double counterClockwise = 0, clockwise = 0;
        floorX(1, &counterClockwise);
        floorX(-1, &clockwise);
        if (std::abs(counterClockwise - clockwise) > 0.2) {
          handedness = counterClockwise > clockwise ? 1 : -1;
          say("floor markers 1, 2, 3, 4 run %s seen from above\n", handedness > 0 ? "counter-clockwise" : "clockwise");
        }
      }
      if (handedness != 0) {
        cv::Vec3d ex = floorX(handedness, nullptr), ey = handedness * normal.cross(ex);
        for (const auto& m : seen) {
          if (cornerSamples[m.id] >= SAMPLES_NEEDED) continue;
          cv::Point3f centre = floorCorners[m.id - 1];
          for (int k = 0; k < 4; k++) {
            cv::Vec3d offset = m.R * cv::Vec3d(floorMarkerShape[k].x, floorMarkerShape[k].y, 0);
            cornerSum[m.id][k] += cv::Point2d(centre.x + offset.dot(ex), centre.y + offset.dot(ey));
          }
          if (++cornerSamples[m.id] == SAMPLES_NEEDED) {
            for (int k = 0; k < 4; k++) {
              floorMarkerCorners[m.id].push_back({(float)(cornerSum[m.id][k].x / SAMPLES_NEEDED), (float)(cornerSum[m.id][k].y / SAMPLES_NEEDED), 0});
            }
            say("floor marker %d learned\n", m.id);
          }
        }
      }
    }

    // Solve this frame's camera pose from every visible corner of the floor markers learned so far.
    // The first solve needs two markers, because one small marker alone can give a mirrored tilt.
    int floorMarkersSeen = 0;
    {
      std::vector<cv::Point3f> onFloor;
      std::vector<cv::Point2f> inImage;
      for (size_t i = 0; i < ids.size(); i++) {
        if (ids[i] < 1 || ids[i] > 4 || floorMarkerCorners[ids[i]].empty()) continue;
        floorMarkersSeen++;
        onFloor.insert(onFloor.end(), floorMarkerCorners[ids[i]].begin(), floorMarkerCorners[ids[i]].end());
        inImage.insert(inImage.end(), corners[i].begin(), corners[i].end());
      }
      if (floorMarkersSeen >= (cam.hasPrevious ? 1 : 2)) {
        // Starting from the previous pose keeps the solution from flipping when few markers are visible.
        cv::Vec3d rvec = cam.rvec, tvec = cam.tvec;
        bool ok = cam.hasPrevious
                      ? cv::solvePnP(onFloor, inImage, cam.K, noDistortion, rvec, tvec, true, cv::SOLVEPNP_ITERATIVE)
                      : cv::solvePnP(onFloor, inImage, cam.K, noDistortion, rvec, tvec, false, cv::SOLVEPNP_IPPE);
        if (ok) setPose(rvec, tvec);
      }
    }

    // Pose of one tracked marker as JSON: floor centimetres once calibrated, pixels before that.
    // Heading is the direction of the marker's top edge, in degrees. z is height above the floor.
    auto poseJson = [&](const std::vector<cv::Point2f>& quad, double knownHeight) {
      cv::Point2f c = centerOf(quad);
      cv::Point2f front = (quad[0] + quad[1]) / 2;
      char buf[240];
      cv::Vec3d rvec, tvec;
      if (cam.valid && cv::solvePnP(markerCorners, quad, cam.K, noDistortion, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE)) {
        double z = cam.toFloor(tvec)[2];  // from the marker's apparent size
        if (knownHeight >= 0) z = cam.C[2] > 0 ? knownHeight : -knownHeight;
        cv::Point2f fc = cam.rayAtHeight(c, z), ff = cam.rayAtHeight(front, z);
        float heading = std::atan2(ff.y - fc.y, ff.x - fc.x) * 180 / CV_PI;
        // The floor z axis points away from the camera when markers 1, 2, 3, 4 run clockwise seen from above.
        double height = cam.C[2] > 0 ? z : -z;
        std::snprintf(buf, sizeof(buf), "{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f,\"heading\":%.1f,\"px\":[%.1f,%.1f]}",
                      fc.x, fc.y, height, heading, c.x, c.y);
      } else {
        float heading = std::atan2(front.y - c.y, front.x - c.x) * 180 / CV_PI;
        std::snprintf(buf, sizeof(buf), "{\"px\":[%.1f,%.1f],\"headingPx\":%.1f}", c.x, c.y, heading);
      }
      return std::string(buf);
    };

    std::string idList, robot = "null", arm = "null";
    for (size_t i = 0; i < ids.size(); i++) {
      idList += (i ? "," : "") + std::to_string(ids[i]);
      if (ids[i] == ROBOT_ID) robot = poseJson(corners[i], robotHeight);
      if (ids[i] == ARM_BASE_ID) arm = poseJson(corners[i], -1);
    }

    int learnedCount = 0;
    for (int id = 1; id <= 4; id++) learnedCount += !floorMarkerCorners[id].empty();
    framesSinceReport++;
    if (now - lastReport >= std::chrono::seconds(1)) {
      fps = framesSinceReport / std::chrono::duration<float>(now - lastReport).count();
      framesSinceReport = 0;
      lastReport = now;
      say("%.1f fps, camera height %.0f cm, floor markers: %d learned, %d used this frame, markers [%s], robot %s, arm %s\n", fps,
                  cam.valid ? std::abs(cam.C[2]) : -1.0, learnedCount, floorMarkersSeen, idList.c_str(), robot.c_str(), arm.c_str());
    }

    // Pinhole camera for this frame: pixel = K * (R * floorPoint + tvec), with R = Rodrigues(rvec).
    std::string cameraJson = "null";
    if (cam.valid) {
      char buf[300];
      std::snprintf(buf, sizeof(buf), "{\"f\":%.1f,\"cx\":%.1f,\"cy\":%.1f,\"rvec\":[%.6f,%.6f,%.6f],\"tvec\":[%.3f,%.3f,%.3f]}",
                    FOCAL_PX, gray.cols / 2.0, gray.rows / 2.0, cam.rvec[0], cam.rvec[1], cam.rvec[2], cam.tvec[0],
                    cam.tvec[1], cam.tvec[2]);
      cameraJson = buf;
    }

    long ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
    std::string line = "{\"t\":" + std::to_string(ms) + ",\"calibrated\":" + (cam.valid ? "true" : "false") +
                       ",\"frame\":[" + std::to_string(gray.cols) + "," + std::to_string(gray.rows) + "],\"floor\":[" + std::to_string((int)floorW) + "," + std::to_string((int)floorH) + "],\"zUp\":" + (cam.valid && cam.C[2] < 0 ? "false" : "true") + ",\"learned\":" + std::to_string(learnedCount) + ",\"cameraHeight\":" + std::to_string(cam.valid ? (int)std::lround(std::abs(cam.C[2])) : -1) + ",\"floorMarkers\":" + std::to_string(floorMarkersSeen) + ",\"fps\":" + std::to_string((int)std::lround(fps)) + ",\"markers\":[" + idList +
                       "],\"robot\":" + robot + ",\"arm\":" + arm + ",\"camera\":" + cameraJson + "}\n";

    recorder.offer(gray, uv, line);
    int fd;
    while ((fd = accept(server, nullptr, nullptr)) >= 0) clients.push_back(fd);
    broadcast(clients, line);
    {
      std::lock_guard<std::mutex> lock(shared.mutex);
      shared.gray = gray;
      shared.uv = uv;
      shared.corners = corners;
      shared.ids = ids;
      shared.state = line;
      broadcast(shared.eventClients, "data: " + line + "\n");  // line already ends with a newline, which completes the event
      for (const std::string& text : shared.unsentLog) broadcast(shared.eventClients, logEvent(text));
      shared.unsentLog.clear();
    }

    // Colour snapshot with detections drawn, for checking the camera aim and the markers.
    if (now - lastSnapshot >= std::chrono::seconds(5)) {
      lastSnapshot = now;
      cv::Mat bgr;
      cv::cvtColorTwoPlane(gray, uv, bgr, cv::COLOR_YUV2BGR_NV12);
      cv::aruco::drawDetectedMarkers(bgr, corners, ids);
      cv::imwrite("snapshot-unit" + std::to_string(unit) + ".jpg", bgr);
    }
  }
}
