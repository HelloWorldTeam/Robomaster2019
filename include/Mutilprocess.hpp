#ifndef _MUTILPROCESS_HPP_
#define _MUTILPROCESS_HPP_
#include <atomic>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include "MVCamera.h"
#include "MarkerSensor.h"
#include "setting.h"
#include "uart.h"
class ImgProcess {
 public:
  using size_t = ::size_t;
  struct DataSend {
    float x, y, z;
    int type;
  };
  ImgProcess(Setting* _setting, int _fd2car) {
    setting = _setting;
    fd2car = _fd2car;
    threadNum = 0;
    lastFrameID = 0;
  }
  void launch() {
    std::thread loadImg_thread(&ImgProcess::loadImg, this);
    std::thread sendResult_thread(&ImgProcess::sendResult, this);
    while (!isExit()) {
      while (!imgBuffer.empty()) {
        while (threadNum >= MAXTHREAD) {
          std::this_thread::yield();
        }
        //取图
        mutexs["imgBuffer"].lock();
        auto frame = imgBuffer.front();
        imgBuffer.pop();
        mutexs["imgBuffer"].unlock();
        //创建子线程处理
        ++threadNum;
        std::thread tmp(&ImgProcess::processImg, this, std::ref(frame.first),
                        frame.second);
        tmp.detach();
      }
    }
    loadImg_thread.join();
    sendResult_thread.join();
  }

 private:
  Setting* setting;
  int fd2car;

  static const int MAXTHREAD = 10;           //最大线程数
  std::map<std::string, std::mutex> mutexs;  //互斥锁
  std::atomic_int threadNum;                 //当前线程数
  // std::atomic_size_t lastFrameID;
  size_t lastFrameID;  //记录输入缓冲中最新的图片id
  // std::pair<std::thread, bool> processThreads[MAXTHREAD];
  std::queue<std::pair<cv::Mat, size_t>>
      imgBuffer;  //输入缓冲，第一个参数为待处理图片，第二个为图片id
  std::pair<DataSend, size_t>
      output;  //输出缓冲，第一个参数为结果，第二个参数为结果对应的图片id
  bool noInput = false;  //记录loadImg线程是否结束
  bool isExit() {
    //最新的图片已处理 且没有新的输入
    return lastFrameID == output.second && noInput;
  }
  void loadImg() {
    printf("MVCamera Init\n");
    MVCamera::Init();
    printf("MVCamera Play\n");
    MVCamera::Play();
    MVCamera::SetExposureTime(false, 2000);
    MVCamera::SetLargeResolution(true);
    Mat srcImg;
    size_t i = 0;
    //获取图片，图片按顺序编号
    while (1) {
      MVCamera::GetFrame(srcImg);
      if (srcImg.empty()) {
        printf("Image empty !\n");
        // usleep(10000);
        continue;
      }
      //将图片编号并放入队列，
      mutexs["imgBuffer"].lock();
      imgBuffer.push(make_pair(srcImg, i));
      mutexs["imgBuffer"].unlock();
      lastFrameID = i;
      ++i;
    }
    //当前线程结束
    noInput = true;
    MVCamera::Stop();
    MVCamera::Uninit();
  }
  void sendResult() {
    static size_t id = 0;
    while (!isExit()) {
      /*if (output.second > id)
      {
              id = output.second;
              printf("send %d\n", output.second);
      }
      continue;*/
      //判断输出缓冲是否更新
      while (output.second <= id && !isExit()) {
        std::this_thread::yield();
      }
      id = output.second;

      //发送结果
      printf("send %llu\n", output.second);
      uart_send(fd2car, x, y, z, type);
    }
  }
  void processImg(const cv::Mat& mat, size_t id) {
    //模拟处理图片所需时间

    //输出结果
    /*while (id > output.second &&!mutexs["output"].try_lock())
    {
            std::this_thread::yield();
    }*/
    if (id > output.second) {
      mutexs["output"].lock();
      //写入到输出缓冲
      output.second = id;
      mutexs["output"].unlock();
    }
    --threadNum;

    //输出调试信息
    std::ostringstream s;
    // s << "finish process img: " << id << " time " << temp % 1000;
    s << " latest frame " << output.second << std::endl;
    std::cout << s.str();
  }
};
#endif