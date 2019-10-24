//
// reCreated by mfh on 20/2/19.
// add multi-core support and soft trigger feature
//

#include "MVCamera.h"
#include <CameraDefine.h>
#include <zconf.h>
#include "Timer.h"

int MVCamera::iCameraCounts = 4;
int MVCamera::iStatus = 0;
int MVCamera::hCamera = 0;
int MVCamera::channel = 3;

// tSdkCameraDevInfo       MVCamera::tCameraEnumList[4];
tSdkCameraCapbility MVCamera::tCapability;  //设备描述信息
tSdkFrameHead MVCamera::sFrameInfo;
unsigned char *MVCamera::g_pBayerBuffer[BAYER_BUFSIZE] = {NULL, NULL, NULL,
                                                          NULL};
unsigned char *MVCamera::g_pRgbBuffer[RGB_BUFSIZE] = {NULL, NULL, NULL,
                                                      NULL};  //处理后数据缓存区

mutex MVCamera::g_pBayerBuffer_mtx[BAYER_BUFSIZE];
mutex MVCamera::bayer_mtx;
mutex MVCamera::g_pRgbBuffer_mtx[RGB_BUFSIZE];
mutex MVCamera::rgb_mtx;

int MVCamera::bayer_buf_head = 0;
int MVCamera::bayer_buf_tail = 0;
int MVCamera::rgb_buf_head = 0;
int MVCamera::rgb_buf_tail = 0;

bool MVCamera::stopped = false;

mutex MVCamera::frame_bayer_mutex;
condition_variable MVCamera::frame_bayer_cond;
mutex MVCamera::frame_rgb_mutex;
condition_variable MVCamera::frame_rgb_cond;

IplImage *MVCamera::iplImage = NULL;

MVCamera::MVCamera() {}

int MVCamera::Init(int id) {
  printf("CAMERA SDK INIT...\n");
  CameraSdkInit(1);
  printf("DONE\n");

  printf("ENUM CAMERA DEVICES...\n");
  tSdkCameraDevInfo tCameraEnumList[4];

  while (1) {
    CameraEnumerateDevice(tCameraEnumList, &iCameraCounts);
    //没有连接设备
    if (iCameraCounts == 0) {
      printf("ERROR: NO CAMERA CONNECTED.\n");
    } else if (iCameraCounts <= id) {
      printf("CONNECTED CAMERA NUMBERS: %d TOO SMALL, ID: %d.\n", iCameraCounts, id);
    } else {
      printf("CONNECTED CAMERA NUMBERS: %d.\n", iCameraCounts);
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    CameraSdkInit(1);
  }
  printf("DONE\n");

  //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
  iStatus = CameraInit(&(tCameraEnumList[id]), -1, -1, &hCamera);
  //初始化失败
  if (iStatus != CAMERA_STATUS_SUCCESS) {
    printf("ERROR: CAMERA INIT FAILED.\n");
    return -1;
  } else {
    printf("CAMERA INIT SUCCESS.\n");
  }

  //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
  CameraGetCapability(hCamera, &tCapability);

  //设置输出为彩色
  channel = 3;
  CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);

  //初始化缓冲区
  for (int i = 0; i < RGB_BUFSIZE; ++i) {
    g_pRgbBuffer[i] =
        (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax *
                                tCapability.sResolutionRange.iWidthMax * 3);
  }
  return 0;
}

void MVCamera::Uninit() {
  printf("Save Parameter...\n");
  CameraSaveParameter(hCamera, 0);

  printf("Uninit...\n");
  int status = CameraUnInit(hCamera);

  printf("status: %d\n", status);

  if (status == CAMERA_STATUS_SUCCESS) {
    printf("CAMERA UNINIT SUCCESS!\n");
  } else {
    printf("CAMERA UNINIT FAILED! ERROR CODE: %d\n", status);
  }
  for (int i = 0; i < RGB_BUFSIZE; ++i) {
    if (g_pRgbBuffer[i] != NULL) {
      free(g_pRgbBuffer[i]);
      g_pRgbBuffer[i] = NULL;
    }
  }
}

int MVCamera::SetExposureTime(bool auto_exp, double exp_time) {
  if (auto_exp) {
    CameraSdkStatus status = CameraSetAeState(hCamera, true);
    if (status == CAMERA_STATUS_SUCCESS) {
      printf("ENABLE AUTO EXP SUCCESS.\n");
    } else {
      printf("ENABLE AUTO EXP FAILED.\n");
      return status;
    }
  } else {
    CameraSdkStatus status = CameraSetAeState(hCamera, false);
    if (status == CAMERA_STATUS_SUCCESS) {
      printf("DISABLE AUTO EXP SUCCESS.\n");
    } else {
      printf("DISABLE AUTO EXP FAILED.\n");
      return status;
    }
    CameraSdkStatus status1 = CameraSetExposureTime(hCamera, exp_time);
    if (status1 == CAMERA_STATUS_SUCCESS) {
      printf("SET EXP TIME SUCCESS.\n");
    } else {
      printf("SET EXP TIME FAILED.\n");
      return status;
    }
  }

  return 0;
}

double MVCamera::GetExposureTime() {
  int auto_exp;
  if (CameraGetAeState(hCamera, &auto_exp) == CAMERA_STATUS_SUCCESS) {
    if (auto_exp) {
      return 0;
    } else {
      double exp_time;
      if (CameraGetExposureTime(hCamera, &exp_time) == CAMERA_STATUS_SUCCESS) {
        return exp_time;
      } else {
        printf("GET CAMERA EXP TIME ERROR.\n");
        return -1;
      }
    }
  } else {
    printf("GET CAMERA AE STATE ERROR.\n");
    return -1;
  }
}

int MVCamera::SetLargeResolution(bool if_large_resolution) {
  tSdkImageResolution resolution;
  if (if_large_resolution) {
    resolution.iIndex = 0;
    if (CameraSetImageResolution(hCamera, &resolution) ==
        CAMERA_STATUS_SUCCESS) {
      printf("CAMERA SET LARGE RESOLUTION SUCCESS.\n");
    } else {
      printf("CAMERA SET LARGE RESOLUTION FAILED.\n");
      return -1;
    }
  } else {
    resolution.iIndex = 1;
    CameraSetImageResolution(hCamera, &resolution);
    if (CameraSetImageResolution(hCamera, &resolution) ==
        CAMERA_STATUS_SUCCESS) {
      printf("CAMERA SET SMALL RESOLUTION SUCCESS.\n");
    } else {
      printf("CAMERA SET SMALL RESOLUTION FAILED.\n");
      return -1;
    }
  }

  return 0;
}

void MVCamera::SetWBMode(bool auto_wb) {
  int status = CameraSetWbMode(hCamera, auto_wb);
  if (CAMERA_STATUS_SUCCESS == status) {
    printf("CAMERA SETWBMODE %d SUCCESS!\n", auto_wb);
  } else {
    printf("CAMERA SETWBMODE %d FAILED! ERROR CODE: %d\n", auto_wb, status);
  }
}

void MVCamera::GetWBMode(bool &auto_wb) {
  int res = 0;
  if (CAMERA_STATUS_SUCCESS == CameraGetWbMode(hCamera, &res)) {
    printf("CAMERA GETWBMODE %d SUCCESS!\n", res);
  } else {
    printf("CAMERA GETWBMODE FAILED!\n");
  }
  auto_wb = res;
}

void MVCamera::SetOnceWB() {
  int status = CameraSetOnceWB(hCamera);
  if (CAMERA_STATUS_SUCCESS == status) {
    printf("CAMERA SETONCEWB SUCCESS!\n");
  } else {
    printf("CAMERA SETONCEWB FAILED, ERROR CODE: %d!\n", status);
  }
}

void MVCamera::SetGain(double gain) {
  int set_gain = int(gain * 100);
  int status = CameraSetGain(hCamera, set_gain, set_gain, set_gain);
  if (CAMERA_STATUS_SUCCESS == status) {
    printf("CAMERA SETGAIN SUCCESS!\n");
  } else {
    printf("CAMERA SETGAIN FAILED! ERROR CODE: %d\n", status);
  }
}

double MVCamera::GetGain() {
  int r_gain, g_gain, b_gain;
  int status = CameraGetGain(hCamera, &r_gain, &g_gain, &b_gain);
  if (CAMERA_STATUS_SUCCESS == status) {
    printf("CAMERA GETGAIN SUCCESS!\n");
  } else {
    printf("CAMERA GETGAIN FAILED! ERROR CODE: %d\n", status);
  }

  return (r_gain + g_gain + b_gain) / 300.;
}

void MVCamera::Play() {
  CameraPlay(hCamera);
#ifndef __NO_SOFT_TRIG__
  CameraSetTriggerMode(hCamera, 1);
  CameraSetTriggerCount(hCamera, 0x7fffffff);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  CameraSoftTrigger(hCamera);
#else
  CameraSetTriggerMode(hCamera, 0);
#endif
  std::thread thread1(MVCamera::Read);
  thread1.detach();
  std::thread thread2(MVCamera::Process);
  thread2.detach();
}

void MVCamera::Read() {
  while (!stopped) {
    int bayer_id = (bayer_buf_head + 1) % BAYER_BUFSIZE;
    g_pBayerBuffer_mtx[bayer_id].lock();
    int status = CameraGetImageBuffer(hCamera, &sFrameInfo,
                                      &g_pBayerBuffer[bayer_id], 100);
    g_pBayerBuffer_mtx[bayer_id].unlock();

    if (CAMERA_STATUS_SUCCESS == status) {
      bayer_mtx.lock();
      bayer_buf_head = bayer_id;
      if (bayer_buf_head == bayer_buf_tail) {
        bayer_buf_tail = (bayer_buf_tail + 1) % BAYER_BUFSIZE;
        g_pBayerBuffer_mtx[bayer_buf_tail].lock();
        CameraReleaseImageBuffer(hCamera, g_pBayerBuffer[bayer_buf_tail]);
        g_pBayerBuffer_mtx[bayer_buf_tail].unlock();
      }
      bayer_mtx.unlock();

      frame_bayer_cond.notify_one();
    } else {
#ifndef __NO_SOFT_TRIG__
      CameraSetTriggerMode(hCamera, 1);
      CameraSetTriggerCount(hCamera, 0x3f3f3f3f);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      CameraSoftTrigger(hCamera);
#else
      CameraSetTriggerMode(hCamera, 0);
#endif
    }
  }
}

void MVCamera::Process() {
  while (!stopped) {
    unique_lock<mutex> lk(frame_bayer_mutex);
    frame_bayer_cond.wait(lk,
                          [] { return (bayer_buf_head != bayer_buf_tail); });

    bayer_mtx.lock();
    int bayer_id = (bayer_buf_tail + 1) % BAYER_BUFSIZE;
    bayer_mtx.unlock();
    int rgb_id = (rgb_buf_head + 1) % RGB_BUFSIZE;

    g_pBayerBuffer_mtx[bayer_id].lock();
    g_pRgbBuffer_mtx[rgb_id].lock();
    CameraImageProcess(hCamera, g_pBayerBuffer[bayer_id], g_pRgbBuffer[rgb_id],
                       &sFrameInfo);
    g_pRgbBuffer_mtx[rgb_id].unlock();
    g_pBayerBuffer_mtx[bayer_id].unlock();

    bayer_mtx.lock();
    if ((bayer_buf_tail + 1) % BAYER_BUFSIZE == bayer_id) {
      g_pBayerBuffer_mtx[bayer_id].lock();
      CameraReleaseImageBuffer(hCamera, g_pBayerBuffer[bayer_id]);
      g_pBayerBuffer_mtx[bayer_id].unlock();
      bayer_buf_tail = bayer_id;
    }
    bayer_mtx.unlock();

    rgb_mtx.lock();
    rgb_buf_head = rgb_id;
    if (rgb_buf_head == rgb_buf_tail) {
      rgb_buf_tail = (rgb_buf_tail + 1) % RGB_BUFSIZE;
    }
    rgb_mtx.unlock();
    frame_rgb_cond.notify_one();
  }
}

int MVCamera::GetFrame(Mat &frame) {
  if (frame.cols != sFrameInfo.iWidth || frame.rows != sFrameInfo.iHeight) {
    printf("GetFrame: resize frame !\n");
    frame.create(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC3);
  }

  unique_lock<mutex> lk(frame_rgb_mutex);
  frame_rgb_cond.wait(lk, [] { return (rgb_buf_head != rgb_buf_tail); });

  rgb_mtx.lock();
  rgb_buf_tail = (rgb_buf_tail + 1) % RGB_BUFSIZE;
  g_pRgbBuffer_mtx[rgb_buf_tail].lock();
  memcpy(frame.data, g_pRgbBuffer[rgb_buf_tail],
         frame.cols * frame.rows * channel);
  // frame.data = g_pRgbBuffer[rgb_buf_tail];
  g_pRgbBuffer_mtx[rgb_buf_tail].unlock();
  rgb_mtx.unlock();

  return 0;
}

int MVCamera::Stop() {
  stopped = true;
  usleep(30000);
  return 0;
}
