//
// Created by wx on 18-4-28.
//

#ifndef ROBOMASTERRUNEDETECTOR_ROBOMASTERPROCESS_H
#define ROBOMASTERRUNEDETECTOR_ROBOMASTERPROCESS_H

#include <opencv2/opencv.hpp>
#include "communicator.h"
#include "setting.h"

class RoboMasterProcess {
 public:
  RoboMasterProcess(Setting* _setting, COMMUNICATOR& tmp,
                    int (*getframe)(cv::Mat& srcImg))
      : setting(_setting), communicator(tmp), getframe(getframe) {}
  void Process();
  Setting* setting;

 private:
  COMMUNICATOR& communicator;
  int (*getframe)(cv::Mat& srcImg);
};

#endif  // ROBOMASTERRUNEDETECTOR_ROBOMASTERPROCESS_H
