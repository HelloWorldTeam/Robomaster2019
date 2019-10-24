//
// Created by flyingtiger on 19-03-04.
//

#include <thread>

#include "MVCamera.h"
#include "RemoteControl.h"
#include "RoboMasterProcess.h"
#include "setting.h"
#include "uart.h"

/*
  definitions:
    __SHOW_DEBUGINFO__
    __SHOWING_SRC_IMG__
    __SHOWING_ALL_IMG__
    __SAVE_VIDEO__
    __READ_VIDEO__
*/
const char *VIDEOPATH = "../../../../test_video.avi";

using namespace std;
using namespace cv;
bool RUNNING = true;

inline void start_camera() {
#ifdef __SHOW_DEBUGINFO__
  printf("MVCamera Init\n");
#endif
  MVCamera::Init();
  MVCamera::Play();
  MVCamera::SetExposureTime(false, 2000);
  MVCamera::SetLargeResolution(true);
}

#ifdef __READ_VIDEO__
int getframe_video(Mat &img) {
  static VideoCapture cap(VIDEOPATH);
  cap >> img;
  return 0;
}
#endif

inline void stop_camera() {
  MVCamera::Stop();
  MVCamera::Uninit();
}

int main(int argc, char *argv[]) {
  Setting setting;
  setting.mode  = 0;  // 0：not working； 1：auto-shooooot
  setting.color = 1;  // 0: all 1: red 2: blue 

#ifndef __READ_VIDEO__
  start_camera();
#endif
  // CAN  commun("can0");
  // UART commun("/dev/ttyUSB0", "/dev/ttyUSB1");
  COMMUNICATOR commun;

  // receive car command
  RemoteControl remoteProcess(&setting, commun);
  std::thread t0(&RemoteControl::Receiver, &remoteProcess);

// process frame and send angle to car
#ifndef __READ_VIDEO__
  RoboMasterProcess roboProcess(&setting, commun, &MVCamera::GetFrame);
#else
  RoboMasterProcess roboProcess(&setting, commun, &getframe_video);
#endif
  std::thread t1(&RoboMasterProcess::Process, &roboProcess);

  t1.join();
  RUNNING = false;
  t0.join();
#ifndef __READ_VIDEO__
  stop_camera();
#endif
  std::cout << "Hello, World!" << std::endl;
  return 0;
}
