//
// Created by liming on 5/31/18.
//

#ifndef MVCAMERA_MVCAMERA_H
#define MVCAMERA_MVCAMERA_H

#include "CameraApi.h"  //相机SDK头文件

#include <condition_variable>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>

using namespace std;
using namespace cv;

#define BAYER_BUFSIZE 4
#define RGB_BUFSIZE 4

class MVCamera {
 public:
  static int Init(int id = 0);
  static void Play();
  static void Read();
  static void Process();
  static int GetFrame(Mat& frame);
  static int Stop();
  static void Uninit();
  static int SetExposureTime(bool auto_exp, double exp_time = 10000);
  static double GetExposureTime();
  static int SetLargeResolution(bool if_large_resolution);
  static Size GetResolution();
  static void SetGain(double gain);
  static double GetGain();
  static void SetWBMode(bool auto_wb = true);
  static void GetWBMode(bool& auto_wb);
  static void SetOnceWB();

  static int iCameraCounts;
  static int iStatus;
  static int hCamera;
  static int channel;

  // static tSdkCameraDevInfo       tCameraEnumList[4];
  static tSdkCameraCapbility tCapability;  //设备描述信息
  static tSdkFrameHead sFrameInfo;

  static unsigned char* g_pBayerBuffer[BAYER_BUFSIZE];
  static mutex g_pBayerBuffer_mtx[BAYER_BUFSIZE];
  static int bayer_buf_head, bayer_buf_tail;
  static mutex bayer_mtx;

  static unsigned char* g_pRgbBuffer[RGB_BUFSIZE];  //处理后数据缓存区
  static mutex g_pRgbBuffer_mtx[RGB_BUFSIZE];
  static int rgb_buf_head, rgb_buf_tail;
  static mutex rgb_mtx;

  static bool stopped;

  static mutex frame_bayer_mutex;
  static condition_variable frame_bayer_cond;
  static mutex frame_rgb_mutex;
  static condition_variable frame_rgb_cond;

  static IplImage* iplImage;

 private:
  MVCamera();
};

#endif  // MVCAMERA_MVCAMERA_H
