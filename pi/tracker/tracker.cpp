// Top-down robot tracker for QNX. Reads one Camera Module 3, detects ArUco markers (DICT_4X4_50),
// and streams the robot's pose as one JSON object per line over TCP.
//   ./tracker <unit> <floor width cm> <floor height cm> [lens code]
// QNX's driver for this camera has no autofocus (tried on the Pi: error 22, no manual focus steps).
// A lens code (0 to 1023, see lens.h) sets the focus motor directly. pi/run-focus.sh finds the best code.
// Without one the lens stays where it is.
// Marker 0 is on the robot. Marker 5 is taped flat on the floor against the base of the SO-101 arm.
// Markers 1 to 4 are taped flat on the floor at the corners of a
// rectangle: 1 = (0, 0), 2 = (W, 0), 3 = (W, H), 4 = (0, H).
// The camera may move. The layout of the floor markers is recorded once, from the first moment all four
// are visible and the camera has been held still for a few frames. That records where each of
// their corners lies on the floor. After that, every frame solves the camera's position from
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
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include "lens.h"

static const int ROBOT_ID = 0;
static const int ARM_BASE_ID = 5;
static const float TRACKED_MARKER_CM = 3.6f;  // printed side length of markers 0 and 5

// Camera Module 3, standard lens, 2304x1296 mode (2x2 binned): 4.74 mm / (2 * 1.4 um) = 1693 px.
// The wide-angle module is 2.75 mm, which gives 982 px.
static const double FOCAL_PX = 1693.0;

static std::mutex frameMutex;
static std::condition_variable frameReady;
static cv::Mat latestGray;
static cv::Mat latestUv;  // NV12 chroma plane, kept only for the colour snapshot

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

// Answers one HTTP request. Returns the socket if it became an event stream that must stay open, else -1.
static int handleHttp(int fd, const cv::Mat& gray, const cv::Mat& uv, const std::vector<std::vector<cv::Point2f>>& corners,
                      const std::vector<int>& ids, const std::string& stateLine) {
  fcntl(fd, F_SETFL, 0);  // the accepted socket may inherit non-blocking mode
  timeval timeout{2, 0};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  char request[1024] = {0};
  recv(fd, request, sizeof(request) - 1, 0);
  std::string path(request);
  path = path.substr(0, path.find("\r\n"));
  const std::string common = "Access-Control-Allow-Origin: *\r\nCache-Control: no-cache\r\n";

  if (path.find("GET /events") == 0) {
    std::string header = "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n" + common + "Connection: keep-alive\r\n\r\n";
    sendAll(fd, header.data(), header.size());
    fcntl(fd, F_SETFL, O_NONBLOCK);  // a slow browser must not stall the tracker
    return fd;
  }

  std::string body, type;
  if (path.find("GET /state.json") == 0) {
    body = stateLine;
    type = "application/json";
  } else if (path.find("GET /frame.jpg") == 0 || path.find("GET /annotated.jpg") == 0) {
    cv::Mat bgr;
    cv::cvtColorTwoPlane(gray, uv, bgr, cv::COLOR_YUV2BGR_NV12);
    if (path.find("GET /annotated.jpg") == 0) cv::aruco::drawDetectedMarkers(bgr, corners, ids);
    size_t w = path.find("w=");
    int width = w == std::string::npos ? 0 : std::atoi(path.c_str() + w + 2);
    if (width >= 160 && width < bgr.cols) cv::resize(bgr, bgr, cv::Size(width, bgr.rows * width / bgr.cols), 0, 0, cv::INTER_AREA);
    std::vector<uchar> jpeg;
    cv::imencode(".jpg", bgr, jpeg, {cv::IMWRITE_JPEG_QUALITY, 85});
    body.assign(jpeg.begin(), jpeg.end());
    type = "image/jpeg";
  }
  std::string header = body.empty() ? "HTTP/1.1 404 Not Found\r\n" : "HTTP/1.1 200 OK\r\nContent-Type: " + type + "\r\n";
  header += common + "Content-Length: " + std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n";
  sendAll(fd, header.data(), header.size());
  sendAll(fd, body.data(), body.size());
  close(fd);
  return -1;
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
  int lensCode = argc > 4 ? std::atoi(argv[4]) : -1;
  int port = 9000 + unit;
  std::signal(SIGPIPE, SIG_IGN);  // a client that disconnects must not kill the tracker

  int server = listenOn(port), httpServer = listenOn(8000 + unit);
  if (server < 0 || httpServer < 0) {
    std::printf("cannot listen on ports %d and %d\n", port, 8000 + unit);
    return 2;
  }
  std::vector<int> clients, eventClients;

  camera_handle_t handle = CAMERA_HANDLE_INVALID;
  camera_error_t err = camera_open((camera_unit_t)unit, CAMERA_MODE_RO, &handle);
  if (err != CAMERA_EOK) {
    std::printf("unit %d: camera_open failed, error %d\n", unit, (int)err);
    return 3;
  }
  camera_set_vf_property(handle, CAMERA_IMGPROP_CREATEWINDOW, 0);
  err = camera_start_viewfinder(handle, onFrame, onStatus, nullptr);
  if (err != CAMERA_EOK) {
    std::printf("unit %d: camera_start_viewfinder failed, error %d\n", unit, (int)err);
    return 4;
  }
  std::printf("unit %d: poses on TCP port %d, frames on http port %d, floor %.0f x %.0f cm\n", unit, port, 8000 + unit,
              floorW, floorH);

  if (lensCode >= 0) {
    int error = lensApproach(unit, lensCode);
    std::printf("lens code %d on %s: %s\n", lensCode, lensBusForUnit(unit), error ? std::strerror(error) : "set");
  }

  cv::aruco::ArucoDetector detector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
  const std::vector<cv::Point3f> floorCorners{{0, 0, 0}, {floorW, 0, 0}, {floorW, floorH, 0}, {0, floorH, 0}};
  FloorCamera cam;
  std::vector<cv::Point3f> floorMarkerCorners[5];  // floor position of each corner of markers 1 to 4, once learned
  bool layoutLearned = false;
  std::vector<cv::Point2f> previousSeen;  // floor marker centres in the last frame that showed all four
  int steadyFrames = 0;
  const int STEADY_FRAMES_NEEDED = 5;     // the camera may be handheld: learn only while it is held still
  const float STEADY_MAX_MOVE_PX = 3;
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
        std::printf("no frames for 2 s\n");
        continue;
      }
      gray = latestGray;
      uv = latestUv;
      latestGray = cv::Mat();
    }
    auto now = std::chrono::steady_clock::now();

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector.detectMarkers(gray, corners, ids);

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

    // Learn the floor layout once, from a frame that shows all four floor markers.
    if (!layoutLearned) {
      std::vector<cv::Point2f> seen(4);
      int found = 0;
      for (size_t i = 0; i < ids.size(); i++) {
        if (ids[i] >= 1 && ids[i] <= 4) {
          seen[ids[i] - 1] = centerOf(corners[i]);
          found++;
        }
      }
      // Count consecutive frames in which all four markers are visible and have barely moved.
      float moved = STEADY_MAX_MOVE_PX + 1;
      if (found == 4 && previousSeen.size() == 4) {
        moved = 0;
        for (int i = 0; i < 4; i++) moved = std::max(moved, (float)cv::norm(seen[i] - previousSeen[i]));
      }
      steadyFrames = found == 4 && moved <= STEADY_MAX_MOVE_PX ? steadyFrames + 1 : 0;
      previousSeen = found == 4 ? seen : std::vector<cv::Point2f>();
      cv::Vec3d rvec, tvec;
      if (steadyFrames >= STEADY_FRAMES_NEEDED &&
          cv::solvePnP(floorCorners, seen, cam.K, noDistortion, rvec, tvec, false, cv::SOLVEPNP_IPPE)) {
        setPose(rvec, tvec);
        for (size_t i = 0; i < ids.size(); i++) {
          if (ids[i] < 1 || ids[i] > 4) continue;
          for (const cv::Point2f& corner : corners[i]) {
            cv::Point2f onFloor = cam.rayAtHeight(corner, 0);
            floorMarkerCorners[ids[i]].push_back({onFloor.x, onFloor.y, 0});
          }
        }
        layoutLearned = true;
        std::printf("floor layout learned: camera %.0f cm above the floor, over floor point (%.0f, %.0f)\n",
                    std::abs(cam.C[2]), cam.C[0], cam.C[1]);
      }
    }

    // Solve this frame's camera pose from every visible floor marker corner.
    int floorMarkersSeen = 0;
    if (layoutLearned) {
      std::vector<cv::Point3f> onFloor;
      std::vector<cv::Point2f> inImage;
      for (size_t i = 0; i < ids.size(); i++) {
        if (ids[i] < 1 || ids[i] > 4) continue;
        floorMarkersSeen++;
        onFloor.insert(onFloor.end(), floorMarkerCorners[ids[i]].begin(), floorMarkerCorners[ids[i]].end());
        inImage.insert(inImage.end(), corners[i].begin(), corners[i].end());
      }
      if (floorMarkersSeen > 0) {
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
    auto poseJson = [&](const std::vector<cv::Point2f>& quad) {
      cv::Point2f c = centerOf(quad);
      cv::Point2f front = (quad[0] + quad[1]) / 2;
      char buf[240];
      cv::Vec3d rvec, tvec;
      if (cam.valid && cv::solvePnP(markerCorners, quad, cam.K, noDistortion, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE)) {
        double z = cam.toFloor(tvec)[2];  // from the marker's apparent size
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
      if (ids[i] == ROBOT_ID) robot = poseJson(corners[i]);
      if (ids[i] == ARM_BASE_ID) arm = poseJson(corners[i]);
    }

    framesSinceReport++;
    if (now - lastReport >= std::chrono::seconds(1)) {
      fps = framesSinceReport / std::chrono::duration<float>(now - lastReport).count();
      framesSinceReport = 0;
      lastReport = now;
      int floorVisible = 0;
      for (int id : ids) floorVisible += id >= 1 && id <= 4;
      std::printf("%.1f fps, camera height %.0f cm, floor markers %d of 4%s, markers [%s], robot %s, arm %s\n", fps,
                  cam.valid ? std::abs(cam.C[2]) : -1.0, floorVisible, layoutLearned ? "" : " (need all 4 held still to learn the layout)",
                  idList.c_str(), robot.c_str(), arm.c_str());
      std::fflush(stdout);
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
                       ",\"frame\":[" + std::to_string(gray.cols) + "," + std::to_string(gray.rows) + "],\"floor\":[" + std::to_string((int)floorW) + "," + std::to_string((int)floorH) + "],\"zUp\":" + (cam.valid && cam.C[2] < 0 ? "false" : "true") + ",\"floorMarkers\":" + std::to_string(floorMarkersSeen) + ",\"fps\":" + std::to_string((int)std::lround(fps)) + ",\"markers\":[" + idList +
                       "],\"robot\":" + robot + ",\"arm\":" + arm + ",\"camera\":" + cameraJson + "}\n";

    int fd;
    while ((fd = accept(httpServer, nullptr, nullptr)) >= 0) {
      int kept = handleHttp(fd, gray, uv, corners, ids, line);
      if (kept >= 0) eventClients.push_back(kept);
    }
    while ((fd = accept(server, nullptr, nullptr)) >= 0) clients.push_back(fd);
    broadcast(clients, line);
    broadcast(eventClients, "data: " + line + "\n");  // line already ends with a newline, which completes the event

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
