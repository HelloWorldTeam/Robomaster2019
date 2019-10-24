//
// Created by wx on 18-4-28.
// Modified by flyingtiger 19.03.05.
//

#include "RoboMasterProcess.h"
#include <sys/time.h>
#include <thread>
#include <unistd.h>
#include "Params.h"
#include "TargetCarDetector.h"
#include "Timer.h"
#include "communicator.h"

using namespace std;

void video_saver(shared_ptr<cv::VideoWriter>& pWriter, cv::Mat& srcImg) {
  static int cnt = 0;
  if (cnt == 0) {
    if (pWriter != NULL) pWriter->release();
    char videoName[50] = "";
    timeval videoTime;
    gettimeofday(&videoTime, 0);
    long long t_v = videoTime.tv_sec;
    sprintf(videoName, "../videos/raw_video_%04lld.avi", t_v);
    pWriter.reset(new cv::VideoWriter(videoName, CV_FOURCC('M', 'J', 'P', 'G'),
                                      100, cv::Size(752, 480)));
  }
  // cv::resize(srcImg, s_img, cv::Size(640, 360));
  pWriter->write(srcImg);
  if (++cnt == 38000) cnt = 0;//about 6 mins
}

void RoboMasterProcess::Process() {
  Mat srcImg;
  float x = 1, y = 1, z = 1;
  int type = 0;
  bool isSuccess = false;
  bool stopped = false;

  printf("Load Calibration...\n");
  Params::LoadCalibration("../TargetCarDetector/calibration/calibration.yml");
  printf("Load Config...\n\n");
  Params::LoadConfig("../TargetCarDetector/config/marker_config.yml");
  
  TargetCarDetector target_car_detector;
  target_car_detector.SetTarget(setting->color);

#ifdef __SAVE_VIDEO__
  shared_ptr<cv::VideoWriter> pWriter;
#endif

#ifdef __SHOW_DEBUGINFO__
  timeval last_time;
  timeval now_time;
  gettimeofday(&last_time, 0);
  double fps = 1;
#endif

  while (1) {
    /// check stopped
    if (stopped) break;

    (*getframe)(srcImg);  // After test, This is not a time-consuming procedure

    if (srcImg.empty()) {
#ifdef __SHOW_DEBUGINFO__
      printf("Image empty !\n");
#endif
#if __READ_VIDEO__
      break;
#endif
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    switch (setting->mode) {
      case 0: break;
      case 1: //auto-shoot
        target_car_detector.SetTarget(setting->color);
        TIME_BEGIN();
        isSuccess =
            (target_car_detector.Process(srcImg, x, y, z, type) ==
             TargetCarDetector::STATUS_SUCCESS);
        TIME_END("ProcessFrameXY");
        if (isSuccess) {
          communicator.send(x, y, z, type);
        }
        // else {
        //   communicator.send(0, 0, 0, type);
        // }
        break;
      default:
        break;
    }
#ifdef __SHOWING_SRC_IMG__
    cv::imshow("srcImg", srcImg);
    char k = cv::waitKey(1);
    if (k == 27) break;
#endif

#ifdef __SAVE_VIDEO__
    video_saver(pWriter, srcImg);
#endif

#ifdef __SHOW_DEBUGINFO__
    gettimeofday(&now_time, 0);
    double time_duaration = now_time.tv_sec - last_time.tv_sec +
                            (now_time.tv_usec - last_time.tv_usec) * 1e-6;
    fps = 5 / (time_duaration + 4 / fps);
    printf("FPS: %06.3f\n", fps);
    gettimeofday(&last_time, 0);
#endif
  }
}
