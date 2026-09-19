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
  auto fail = [&](int code, int error) {
    std::printf("lens write failed at code %d in %s(): error %d (%s)\n", code, lensFailedAt(), error, std::strerror(error));
    return 4;
  };
  // Scores codes from `from` to `to`, always moving upward. Returns the sharpest code.
  double bestSharpness = -1;
  auto sweep = [&](int from, int to, int step, int& best) {
    int error = lensApproach(unit, from);
    if (error) return error;
    for (int code = from; code <= to; code += step) {
      if ((error = lensSetCode(unit, code))) return error;
      double s = (sharpness(nextFrame(4)) + sharpness(nextFrame(0)) + sharpness(nextFrame(0))) / 3;
      std::printf("code %4d  sharpness %8.1f\n", code, s);
      if (s > bestSharpness) { bestSharpness = s; best = code; }
    }
    return 0;
  };

  int best = -1, error = 0;
  if (argc > 2) {
    best = std::atoi(argv[2]);
  } else {
    if ((error = sweep(300, 800, 20, best))) return fail(best, error);
    std::printf("fine sweep around %d\n", best);
    int coarse = best;
    bestSharpness = -1;
    if ((error = sweep(coarse - 30, coarse + 30, 5, best))) return fail(best, error);
  }
  if ((error = lensApproach(unit, best))) return fail(best, error);
  cv::Mat frame = nextFrame(6);
  double now = sharpness(frame);
  std::printf("lens set to code %d. sharpness now: %.1f", best, now);
  if (bestSharpness > 0) std::printf(" (best seen in the sweep: %.1f)", bestSharpness);
  std::printf("\n");
  cv::imwrite("focus-unit" + std::to_string(unit) + ".jpg", frame);
  camera_stop_viewfinder(handle);
  camera_close(handle);
  return 0;
}
