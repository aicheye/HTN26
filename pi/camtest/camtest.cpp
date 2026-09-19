// Captures frames from one QNX camera unit for a few seconds, runs ArUco detection on each,
// and saves the first frame as a JPEG.
//   ./camtest <unit> [seconds]
// Units on this image: 3 = Camera Module 3 on port 0, 4 = Camera Module 3 on port 1.
#include <camera/camera_api.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>

static std::mutex frameMutex;
static std::condition_variable frameReady;
static cv::Mat latestGray;   // copy of the newest frame's luma plane
static long framesReceived = 0;
static int lastFrameType = -1;

// Runs on a camera library thread. The buffer is only valid during the call, so copy it out.
static void onFrame(camera_handle_t, camera_buffer_t* buf, void*) {
  std::lock_guard<std::mutex> lock(frameMutex);
  framesReceived++;
  lastFrameType = (int)buf->frametype;
  if (buf->frametype == CAMERA_FRAMETYPE_NV12) {
    // NV12 starts with a full-resolution 8-bit luma plane, which is the grayscale image ArUco needs.
    const auto& d = buf->framedesc.nv12;
    cv::Mat(d.height, d.width, CV_8UC1, buf->framebuf, d.stride).copyTo(latestGray);
  } else if (buf->frametype == CAMERA_FRAMETYPE_YCBYCR) {
    const auto& d = buf->framedesc.ycbycr;
    cv::Mat packed(d.height, d.width, CV_8UC2, buf->framebuf, d.stride);
    cv::extractChannel(packed, latestGray, 0);
  } else {
    return;
  }
  frameReady.notify_one();
}

static void onStatus(camera_handle_t, camera_devstatus_t status, uint16_t extra, void*) {
  std::printf("camera status %d (extra %u)\n", (int)status, (unsigned)extra);
}

int main(int argc, char** argv) {
  int unit = argc > 1 ? std::atoi(argv[1]) : 3;
  int seconds = argc > 2 ? std::atoi(argv[2]) : 5;

  camera_handle_t handle = CAMERA_HANDLE_INVALID;
  camera_error_t err = camera_open((camera_unit_t)unit, CAMERA_MODE_RO, &handle);
  if (err != CAMERA_EOK) {
    std::printf("unit %d: camera_open failed, error %d\n", unit, (int)err);
    return 2;
  }
  err = camera_set_vf_property(handle, CAMERA_IMGPROP_CREATEWINDOW, 0);
  if (err != CAMERA_EOK) std::printf("unit %d: could not disable the viewfinder window, error %d\n", unit, (int)err);

  err = camera_start_viewfinder(handle, onFrame, onStatus, nullptr);
  if (err != CAMERA_EOK) {
    std::printf("unit %d: camera_start_viewfinder failed, error %d\n", unit, (int)err);
    camera_close(handle);
    return 3;
  }

  cv::aruco::ArucoDetector detector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
  auto start = std::chrono::steady_clock::now();
  long processed = 0;
  bool saved = false;
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(seconds)) {
    cv::Mat gray;
    {
      std::unique_lock<std::mutex> lock(frameMutex);
      if (!frameReady.wait_for(lock, std::chrono::seconds(1), [] { return !latestGray.empty(); })) continue;
      gray = latestGray;
      latestGray = cv::Mat();
    }
    if (!saved) {
      std::string path = "frame-unit" + std::to_string(unit) + ".jpg";
      cv::imwrite(path, gray);
      std::printf("unit %d: %dx%d, saved %s\n", unit, gray.cols, gray.rows, path.c_str());
      saved = true;
    }
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector.detectMarkers(gray, corners, ids);
    processed++;
    for (size_t i = 0; i < ids.size(); i++) {
      cv::Point2f c = (corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]) / 4;
      std::printf("unit %d: marker %d at (%.0f, %.0f)\n", unit, ids[i], c.x, c.y);
    }
  }

  camera_stop_viewfinder(handle);
  camera_close(handle);
  std::printf("unit %d: received %ld frames, ran detection on %ld, in %d s (frame type %d)\n",
              unit, framesReceived, processed, seconds, lastFrameType);
  return framesReceived > 0 ? 0 : 4;
}
