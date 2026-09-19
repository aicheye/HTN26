// Finds and sets the sharpest lens position for one Camera Module 3, then leaves the lens there.
//   ./focus <unit>          sweep lens codes, print sharpness for each, keep the sharpest
//   ./focus <unit> <code>   set one lens code (0 to 1023) and report the sharpness
// Aim the camera at the markers from its working distance before running. Saves focus-unit<N>.jpg.
#include <camera/camera_api.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>

#include "../tracker/lens.h"

static std::mutex frameMutex;
static std::condition_variable frameReady;
static cv::Mat latestGray;
static long frameCount = 0;

static void onFrame(camera_handle_t, camera_buffer_t* buf, void*) {
  if (buf->frametype != CAMERA_FRAMETYPE_NV12) return;
  const auto& d = buf->framedesc.nv12;
  std::lock_guard<std::mutex> lock(frameMutex);
  cv::Mat(d.height, d.width, CV_8UC1, buf->framebuf, d.stride).copyTo(latestGray);
  frameCount++;
  frameReady.notify_one();
}
static void onStatus(camera_handle_t, camera_devstatus_t, uint16_t, void*) {}

// Waits for `skip` new frames so the lens has settled, then returns the next one.
static cv::Mat nextFrame(int skip) {
  std::unique_lock<std::mutex> lock(frameMutex);
  long target = frameCount + skip + 1;
  frameReady.wait_for(lock, std::chrono::seconds(3), [&] { return frameCount >= target; });
  return latestGray.clone();
}

// Variance of the Laplacian over the middle half of the frame. Higher means sharper.
static double sharpness(const cv::Mat& gray) {
  if (gray.empty()) return 0;
  cv::Mat middle = gray(cv::Rect(gray.cols / 4, gray.rows / 4, gray.cols / 2, gray.rows / 2)), edges;
  cv::Laplacian(middle, edges, CV_64F);
  cv::Scalar mean, deviation;
  cv::meanStdDev(edges, mean, deviation);
  return deviation[0] * deviation[0];
}

int main(int argc, char** argv) {
  int unit = argc > 1 ? std::atoi(argv[1]) : 3;
  camera_handle_t handle = CAMERA_HANDLE_INVALID;
  camera_error_t err = camera_open((camera_unit_t)unit, CAMERA_MODE_RO, &handle);
  if (err != CAMERA_EOK) { std::printf("camera_open failed, error %d\n", (int)err); return 2; }
  camera_set_vf_property(handle, CAMERA_IMGPROP_CREATEWINDOW, 0);
  err = camera_start_viewfinder(handle, onFrame, onStatus, nullptr);
  if (err != CAMERA_EOK) { std::printf("camera_start_viewfinder failed, error %d\n", (int)err); return 3; }

  std::printf("unit %d, lens on %s. sharpness before any change: %.1f\n", unit, lensBusForUnit(unit), sharpness(nextFrame(5)));
  int best = -1;
  if (argc > 2) {
    best = std::atoi(argv[2]);
  } else {
    double bestSharpness = -1;
    for (int code = 300; code <= 800; code += 20) {
      int error = lensSetCode(unit, code);
      if (error) { std::printf("lens write failed at code %d in %s(): error %d (%s)\n", code, lensFailedAt(), error, std::strerror(error)); return 4; }
      double s = (sharpness(nextFrame(6)) + sharpness(nextFrame(0)) + sharpness(nextFrame(0))) / 3;
      std::printf("code %4d  sharpness %8.1f\n", code, s);
      if (s > bestSharpness) { bestSharpness = s; best = code; }
    }
  }
  int error = lensSetCode(unit, best);
  if (error) { std::printf("lens write failed in %s(): error %d (%s)\n", lensFailedAt(), error, std::strerror(error)); return 4; }
  cv::Mat frame = nextFrame(8);
  std::printf("lens set to code %d (focus distance about %.2f m). sharpness now: %.1f\n", best,
              best > 445 ? 32.0 / (best - 445) : 99.0, sharpness(frame));
  cv::imwrite("focus-unit" + std::to_string(unit) + ".jpg", frame);
  camera_stop_viewfinder(handle);
  camera_close(handle);
  return 0;
}
