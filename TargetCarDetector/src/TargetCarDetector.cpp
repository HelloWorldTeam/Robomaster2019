//
// NEED TO RECONSTRUCT
//

#include "TargetCarDetector.h"
#include <string>
#include "Timer.h"

std::atomic<TargetCarDetector::SensorStatus> TargetCarDetector::status(
    TargetCarDetector::STATUS_DETECTING);

TargetCarDetector::TargetCarDetector() {
  // status = STATUS_DETECTING;
  index = 0;
}

int TargetCarDetector::ResizeImg(const cv::Mat &img) {
  // TIME_BEGIN();
  if (img.cols != Params::camera_width || img.rows != Params::camera_height) {
    cv::resize(img, img_bgr,
               cv::Size(Params::camera_width, Params::camera_height));
  } else {
    img.copyTo(img_bgr);
  }
  // TIME_END("ResizeOrCopy");

  return 0;
}

int TargetCarDetector::PCALEDStrip(vector<cv::Point> &contour, RotRect &LED) {
  int sz = static_cast<int>(contour.size());
  cv::Mat data_pts(sz, 2, CV_64FC1);
  double *_data_pts = (double *)data_pts.data;

  for (int i = 0; i < data_pts.rows; ++i, _data_pts += 2) {
    _data_pts[0] = contour[i].x;
    _data_pts[1] = contour[i].y;
  }

  cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
  LED.center.x = static_cast<float>(pca_analysis.mean.at<double>(0, 0));
  LED.center.y = static_cast<float>(pca_analysis.mean.at<double>(0, 1));

  cv::Point2f dir1, dir2;
  dir1.x = static_cast<float>(pca_analysis.eigenvectors.at<double>(0, 0));
  dir1.y = static_cast<float>(pca_analysis.eigenvectors.at<double>(0, 1));
  dir2.x = static_cast<float>(pca_analysis.eigenvectors.at<double>(1, 0));
  dir2.y = static_cast<float>(pca_analysis.eigenvectors.at<double>(1, 1));

  dir1 = dir1 * (1 / cv::norm(dir1));
  dir2 = dir2 * (1 / cv::norm(dir2));

  LED.dir = dir1;
  LED.width = ComputeLengthAlongDir(contour, dir1);
  LED.height = ComputeLengthAlongDir(contour, dir2);

  return 0;
}

float TargetCarDetector::ComputeLengthAlongDir(vector<cv::Point> &contour,
                                               cv::Point2f &dir) {
  float max_range = -999999;
  float min_range = 999999;
  for (auto &pt : contour) {
    float x = pt.x * dir.x + pt.y * dir.y;
    if (x < min_range) min_range = x;
    if (x > max_range) max_range = x;
  }

  return (max_range - min_range);
}

int TargetCarDetector::GetLEDMarker(cv::Mat &roi_mask, Marker &res_marker) {
  // FindContours and check length
  vector<vector<cv::Point> > tmp_contours;
  vector<vector<cv::Point> *> pContours;
  cv::findContours(roi_mask, tmp_contours, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_NONE);
  for (auto &contour : tmp_contours) {
    int contour_sz = static_cast<int>(contour.size());
    if (contour_sz >= Params::contours_length_min &&
        contour_sz <= Params::contours_length_max) {
      pContours.push_back(&contour);
    }
  }

  // PCA - Get LED Strip
  vector<RotRect> LEDs;
  LEDs.reserve(10);
  for (auto &pContour : pContours) {
    RotRect LED;
    if (PCALEDStrip(*pContour, LED) == STATUS_SUCCESS) {
      // check width
      if (LED.width < Params::LED_width_min ||
          LED.width > Params::LED_width_max)
        continue;
      float ratio = LED.width / LED.height;

      // check width/height(ratio)
      if (ratio < Params::LED_ratio_min || ratio > Params::LED_ratio_max)
        continue;
      LEDs.push_back(LED);
    }
  }

#ifdef __SHOWING_ALL_IMG__
  Mat res_1 = img_bgr.clone();
  Mat res_2 = img_bgr.clone();
  drawContours(res_1, tmp_contours, -1, Scalar(255, 0, 0), 1);
  // imshow("Coutours", res_1);

  for (int i = 0; i < LEDs.size(); i++) {
    string s = to_string(i);
    Point2f dir_h;
    dir_h.x = LEDs[i].dir.y;
    dir_h.y = (-1.0) * LEDs[i].dir.x;

    Point2f kp[6];
    kp[0] = LEDs[i].center + LEDs[i].dir * LEDs[i].width * 0.5f +
            dir_h * LEDs[i].height * 0.5f;
    kp[1] = LEDs[i].center + LEDs[i].dir * LEDs[i].width * 0.5f -
            dir_h * LEDs[i].height * 0.5f;
    kp[2] = LEDs[i].center - LEDs[i].dir * LEDs[i].width * 0.5f -
            dir_h * LEDs[i].height * 0.5f;
    kp[3] = LEDs[i].center - LEDs[i].dir * LEDs[i].width * 0.5f +
            dir_h * LEDs[i].height * 0.5f;
    kp[4] = LEDs[i].center + LEDs[i].dir * 40;
    kp[5] = LEDs[i].center + dir_h * 40;

    line(res_2, kp[0], kp[1], cv::Scalar(255, 0, 0), 1);
    line(res_2, kp[1], kp[2], cv::Scalar(255, 0, 0), 1);
    line(res_2, kp[2], kp[3], cv::Scalar(255, 0, 0), 1);
    line(res_2, kp[3], kp[0], cv::Scalar(255, 0, 0), 1);
    line(res_2, LEDs[i].center, kp[4], cv::Scalar(0, 255, 0), 1);
    line(res_2, LEDs[i].center, kp[5], cv::Scalar(0, 255, 0), 1);
    putText(res_2, s, LEDs[i].center, FONT_HERSHEY_SIMPLEX, 1,
            cv::Scalar(255, 255, 0));
  }
#endif

  if (LEDs.size() < 2) {
    printf("LED num < 2 ! \n");
    return -1;
  }

  vector<Marker> markers;
  float cos_marker_parallel_radian =
      cos(Params::marker_parallel_angle / 180.f * 3.1415926f);
  float cos_marker_vertical_radian =
      cos((90 - Params::marker_vertical_angle) / 180.f * 3.1415926f);
  float cos_marker_direction_radian =
      cos(Params::marker_direction_angle / 180.f * 3.1415926f);
  size_t LED_sz = LEDs.size();

  vector<bool> matched(LED_sz, false);
  for (size_t i = 0; i < LED_sz; ++i) {
    if (matched[i]) continue;
    for (size_t j = i + 1; j < LED_sz; ++j) {
      if (matched[j]) continue;

      // printf("Check pair: %lu %lu\n", i, j);
      cv::Point2f c2c = LEDs[i].center - LEDs[j].center;

      // check width difference
      float max_width = max(LEDs[i].width, LEDs[j].width);
      if (fabs(LEDs[i].width - LEDs[j].width) / max_width > 0.3) {
        // printf("LED difference not satisfied !\n");
        continue;
      }

      // check parallel
      if (fabs(LEDs[i].dir.dot(LEDs[j].dir)) < cos_marker_parallel_radian) {
        // printf("LED parallel not satisfied !\n");
        continue;
      }

      // check vertical
      Point2f c2c_1 = c2c * (1.0 / cv::norm(c2c));
      if (fabs(LEDs[i].dir.dot(c2c_1)) > cos_marker_vertical_radian) {
        // printf("LED vertical not satisfied !\n");
        continue;
      }

      // check direction
      if (fabs(c2c_1.dot(cv::Point2f(1, 0))) < cos_marker_direction_radian) {
        // printf("Marker direction not satisfied !\n");
        continue;
      }

      // check distance
      float distance = cv::norm(c2c);
      if (distance > Params::marker_size_max ||
          distance < Params::marker_size_min) {
        // printf("LED distance not satisfied !\n");
        continue;
      }

      Marker marker;
      if (c2c.x > 0) {
        marker.LEDs[0] = LEDs[j];
        marker.LEDs[1] = LEDs[i];
      } else {
        marker.LEDs[0] = LEDs[i];
        marker.LEDs[1] = LEDs[j];
      }

      // check marker width/height ratio
      float marker_width = distance;
      float marker_height = (LEDs[i].width + LEDs[j].width) * 0.5f;
      float marker_size_ratio = marker_width / marker_height;
      if (marker_size_ratio > Params::marker_ratio_max ||
          marker_size_ratio < Params::marker_ratio_min) {
        // printf("Marker size ratio not satisfied !\n");
        continue;
      }

      matched[i] = matched[j] = true;
      markers.push_back(marker);
    }
  }

  if (markers.size() == 0) {
    printf("No marker found !\n");
    return -1;
  } else {
    printf("Find markers !\n");
  }

  int marker_num = int(markers.size());

  if (marker_num == 1) {
    res_marker = markers[0];
    printf(">>>>>>>>>>>>>>>>==================<<<<<<<<<<<<<<<< %d\n",
           marker_num);
    return 0;
  }

  ///////////////////choose best begin///////////////////////
  Point2f center(Params::camera_cx, Params::camera_cy);
  float marker_dist[marker_num], marker_area_size[marker_num],
      marker_direction[marker_num];
  int rank[3][marker_num], marker_value[marker_num];

  for (int i = 0; i < marker_num; i++) {
    rank[0][i] = i;
    rank[1][i] = i;
    rank[2][i] = i;
    marker_value[i] = 0;

    Point2f marker_center =
        0.5f * (markers[i].LEDs[0].center + markers[i].LEDs[1].center);
    Point2f LED_c2c = markers[i].LEDs[1].center - markers[i].LEDs[0].center;
    float LED_WIDTH =
        0.5f * (markers[i].LEDs[0].width + markers[i].LEDs[1].width);
    float LED_LENGTH = cv::norm(LED_c2c);

    // evaluate the distance from center
    marker_dist[i] = cv::norm(marker_center - center);

    // evaluate the size of marker
    marker_area_size[i] = LED_LENGTH * LED_WIDTH;

    // evaluate the gradient of marker
    marker_direction[i] = fabs(LED_c2c.dot(cv::Point2f(1, 0)));
  }

  float temp1, temp2;
  int t1, t2, t3;
  for (int i = 0; i < marker_num - 1; i++) {
    t1 = t2 = t3 = i;
    for (int j = i + 1; j < marker_num; j++) {
      if (marker_dist[j] < marker_dist[t1]) t1 = j;
      if (marker_area_size[j] > marker_area_size[t2]) t2 = j;
      if (marker_direction[j] > marker_direction[t3]) t3 = j;
    }

    temp1 = marker_dist[i];
    marker_dist[i] = marker_dist[t1];
    marker_dist[t1] = temp1;
    temp2 = rank[0][i];
    rank[0][i] = rank[0][t1];
    rank[0][t1] = temp2;

    temp1 = marker_area_size[i];
    marker_area_size[i] = marker_area_size[t2];
    marker_area_size[t2] = temp1;
    temp2 = rank[1][i];
    rank[1][i] = rank[1][t2];
    rank[1][t2] = temp2;

    temp1 = marker_direction[i];
    marker_direction[i] = marker_direction[t3];
    marker_direction[t3] = temp1;
    temp2 = rank[2][i];
    rank[2][i] = rank[2][t3];
    rank[2][t3] = temp2;
  }

  // assign different points to different situations
  for (int i = 0; i < marker_num; ++i) {
    if (i == 0) {
      marker_value[rank[0][i]] += 10 * 0.5;
      marker_value[rank[1][i]] += 10 * 0.3;
      marker_value[rank[2][i]] += 10 * 0.2;
      continue;
    }
    if (i == 1) {
      marker_value[rank[0][i]] += 6 * 0.5;
      marker_value[rank[1][i]] += 6 * 0.3;
      marker_value[rank[2][i]] += 6 * 0.2;
      continue;
    }
    marker_value[rank[0][i]] += 4 * 0.5;
    marker_value[rank[1][i]] += 4 * 0.3;
    marker_value[rank[2][i]] += 4 * 0.2;
  }

#ifdef __SHOWING_ALL_IMG__
  for (int i = 0; i < marker_num; i++) {
    string s = to_string(marker_value[i]);
    markers[i].Draw(res_2);
    putText(res_2, s,
            0.5f * (markers[i].LEDs[0].center + markers[i].LEDs[1].center),
            FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
  }
  imshow("Markers_all", res_2);
#endif

  // choose the one with the hightest probability
  int best_id = -1;
  float best_value = 0;
  for (int i = 0; i < marker_num; ++i) {
    if (marker_value[i] > best_value) {
      best_value = marker_value[i];
      best_id = i;
    }
  }
  res_marker = markers[best_id];

  //////////////////////choose best end/////////////////////////

  printf(">>>>>>>>>>>>>>>>==================<<<<<<<<<<<<<<<< %d\n", marker_num);
  return 0;
}

int TargetCarDetector::DetectLEDMarker(Marker &res_marker) {
  cv::Mat img_hsv, led_mask;
  // TIME_BEGIN();
  cv::cvtColor(img_bgr, img_hsv, CV_BGR2HSV);
  // TIME_END("cvtColorHsv");

  if (Params::target_color == 1) {
    cv::inRange(
        img_hsv, cv::Scalar(Params::red_hmin, Params::smin, Params::vmin),
        cv::Scalar(Params::red_hmax, Params::smax, Params::vmax), led_mask);
  } else if (Params::target_color == 2) {
    cv::inRange(
        img_hsv, cv::Scalar(Params::blue_hmin, Params::smin, Params::vmin),
        cv::Scalar(Params::blue_hmax, Params::smax, Params::vmax), led_mask);
  } else {
    cv::inRange(img_hsv, cv::Scalar(Params::hmin, Params::smin, Params::vmin),
                cv::Scalar(Params::hmax, Params::smax, Params::vmax), led_mask);
  }

#ifdef __SHOWING_ALL_IMG__
  cv::imshow("led_mask", led_mask);
#endif

  return GetLEDMarker(led_mask, res_marker);
}

int TargetCarDetector::TrackLEDMarker(Marker &res_marker) {
  res_marker.ComputeKeyPoints();
  res_marker.ComputeBBox();
  Rect &box = res_marker.bbox;

  float left = target.x - 1.5*box.width;
  float right = target.x + 1.5*box.width;
  float top = target.y - 1.25*box.height;
  float bot = target.y + 1.25*box.height;

  left = left < 0 ? 0 : left;
  right = right >= Params::camera_width ? Params::camera_width : right;
  top = top < 0 ? 0 : top;
  bot = bot >= Params::camera_height ? Params::camera_height : bot;
  Rect ROI(left, top, (right - left), (bot - top));

  // Get Mask
  cv::Mat ROI_bgr = img_bgr(ROI).clone();
  cv::Mat ROI_img_hsv;
  cv::cvtColor(ROI_bgr, ROI_img_hsv, CV_BGR2HSV);
  cv::Mat ROI_led_mask;

  if (Params::target_color == 1) {
    cv::inRange(
        ROI_img_hsv, cv::Scalar(Params::red_hmin, Params::smin, Params::vmin),
        cv::Scalar(Params::red_hmax, Params::smax, Params::vmax), ROI_led_mask);
  } else if (Params::target_color == 2) {
    cv::inRange(ROI_img_hsv,
                cv::Scalar(Params::blue_hmin, Params::smin, Params::vmin),
                cv::Scalar(Params::blue_hmax, Params::smax, Params::vmax),
                ROI_led_mask);
  } else {
    cv::inRange(
        ROI_img_hsv, cv::Scalar(Params::hmin, Params::smin, Params::vmin),
        cv::Scalar(Params::hmax, Params::smax, Params::vmax), ROI_led_mask);
  }

#ifdef __SHOWING_ALL_IMG__
  cv::imshow("ROI_led_mask", ROI_led_mask);
#endif

  if (GetLEDMarker(ROI_led_mask, res_marker) != STATUS_SUCCESS) {
    return -1;
  }

  res_marker.LEDs[0].center.x += ROI.x;
  res_marker.LEDs[0].center.y += ROI.y;
  res_marker.LEDs[1].center.x += ROI.x;
  res_marker.LEDs[1].center.y += ROI.y;

  return 0;
}

int TargetCarDetector::Process(const Mat &img, float &X, float &Y, float &Z,
                               int &type) {
  ResizeImg(img);

  if (status == STATUS_DETECTING) {
    if (DetectLEDMarker(marker) == STATUS_SUCCESS) {
      status = STATUS_TRACKING;
      printf("Detect Success!\n");
    } else {
      printf("Detect No target!\n");
      return -1;
    }
  } else {
    if (TrackLEDMarker(marker) == STATUS_SUCCESS) {
      printf("Track Success!\n");
    } else {
      status = STATUS_DETECTING;
      printf("Track No target!\n");
      return -1;
    }
  }  

  // Update 3D position
  float marker_width =
      (float)cv::norm(marker.LEDs[0].center - marker.LEDs[1].center);
  float marker_height = (marker.LEDs[0].width + marker.LEDs[1].width) * 0.5f;
  float focal_length = (Params::camera_fx + Params::camera_fy) * 0.5f;
  float real_L = Params::transformer_small_marker_size;
  type = 0;  // judge car type
  if ((marker_width / marker_height) > 4) {
    real_L = Params::transformer_big_marker_size;
    type = 1;
  }
  depth = (real_L * focal_length) / (marker_width);
  target = (marker.LEDs[0].center + marker.LEDs[1].center) * 0.5f;
  Z = depth;
  X = (target.x - Params::camera_cx) / Params::camera_fx * Z;
  Y = (target.y - Params::camera_cy) / Params::camera_fy * Z;

#ifdef __SHOWING_ALL_IMG__
  Mat res = img_bgr.clone();
  marker.Draw(res);
#ifdef __SAVE_VIDEO__
  char videoName[50] = "";
  timeval videoTime;
  gettimeofday(&videoTime, 0);
  long long t_v = videoTime.tv_sec;
  sprintf(videoName, "../images/record_%04lld.jpg", t_v);
  imwrite(videoName, res);
#endif

  if (status = STATUS_TRACKING) {
    circle(res, target, 5, CV_RGB(255, 255, 0), 2);
  }

  imshow("Marker", res);

  if (index < 500) {
    Mat figure_x(640, 500, CV_8UC3, Scalar(255, 255, 255));
    Mat figure_y(640, 500, CV_8UC3, Scalar(255, 255, 255));
    Mat figure_z(640, 500, CV_8UC3, Scalar(255, 255, 255));
    index_x[0][index] = target.x;
    index_y[0][index] = target.y;
    index_z[0][index] = depth * 100;
    for (int i = 0; i <= index; i++) {
      Point p1(i, int(index_x[0][i]));
      circle(figure_x, p1, 1, Scalar(255, 0, 0));
      Point p2(i, int(index_y[0][i]));
      circle(figure_y, p2, 1, Scalar(255, 0, 0));
      Point p3(i, int(index_z[0][i]));
      circle(figure_z, p3, 1, Scalar(255, 0, 0));
    }

    imshow("x", figure_x);
    imshow("y", figure_y);
    imshow("z", figure_z);
    index++;
  }
  if (index >= 500) {
    Mat figure_x(640, 500, CV_8UC3, Scalar(255, 255, 255));
    Mat figure_y(640, 500, CV_8UC3, Scalar(255, 255, 255));
    Mat figure_z(640, 500, CV_8UC3, Scalar(255, 255, 255));

    for (int i = 1; i <= 499; i++) {
      index_x[0][i - 1] = index_x[0][i];
      index_y[0][i - 1] = index_y[0][i];
      index_z[0][i - 1] = index_z[0][i];
    }
    index_x[0][499] = target.x;
    index_y[0][499] = target.y;
    index_z[0][499] = depth * 100;

    for (int i = 0; i < 500; i++) {
      Point p1(i, int(index_x[0][i]));
      circle(figure_x, p1, 1, Scalar(255, 0, 0));
      Point p2(i, int(index_y[0][i]));
      circle(figure_y, p2, 1, Scalar(255, 0, 0));
      Point p3(i, int(index_z[0][i]));
      circle(figure_z, p3, 1, Scalar(255, 0, 0));
    }

    imshow("x", figure_x);
    imshow("y", figure_y);
    imshow("z", figure_z);
  }
#endif

  // printf("Get target: %f %f %f type: %d\n", X, Y, Z, type);
  return 0;
}

int TargetCarDetector::SetTarget(int i) {
  Params::target_color = i;
  return 0;
}

int Params::LoadCalibration(const string &calibration) {
  // read calibration parameters
  cv::FileStorage fCalibration(calibration, cv::FileStorage::READ);
  Params::camera_width = fCalibration["Camera.width"];
  Params::camera_height = fCalibration["Camera.height"];
  Params::camera_fps = fCalibration["Camera.fps"];
  Params::camera_fx = fCalibration["Camera.fx"];
  Params::camera_fy = fCalibration["Camera.fy"];
  Params::camera_cx = fCalibration["Camera.cx"];
  Params::camera_cy = fCalibration["Camera.cy"];
  Params::camera_k1 = fCalibration["Camera.k1"];
  Params::camera_k2 = fCalibration["Camera.k2"];
  Params::camera_k3 = fCalibration["Camera.k3"];
  cout << "===Camera Calibration===" << endl;
  cout << "width: " << Params::camera_width << endl;
  cout << "height: " << Params::camera_height << endl;
  cout << "fps: " << Params::camera_fps << endl;
  cout << "fx: " << Params::camera_fx << endl;
  cout << "fy: " << Params::camera_fy << endl;
  cout << "cx: " << Params::camera_cx << endl;
  cout << "cy: " << Params::camera_cy << endl;
  cout << "k1: " << Params::camera_k1 << endl;
  cout << "k2: " << Params::camera_k2 << endl;
  cout << "k3: " << Params::camera_k3 << endl;
  return 0;
}
int Params::LoadConfig(const string &config) {
  // read config parameters
  cv::FileStorage fConfig(config, cv::FileStorage::READ);
  // ImageFrame
  Params::blur_sz = fConfig["ImageFrame.blur_size"];
  Params::blur_sigma = fConfig["ImageFrame.blur_sigma"];
  // Detector
  Params::hmin = fConfig["Detector.hmin"];
  Params::hmax = fConfig["Detector.hmax"];
  Params::blue_hmin = fConfig["Detector.blue_hmin"];
  Params::blue_hmax = fConfig["Detector.blue_hmax"];
  Params::red_hmin = fConfig["Detector.red_hmin"];
  Params::red_hmax = fConfig["Detector.red_hmax"];
  Params::smin = fConfig["Detector.smin"];
  Params::smax = fConfig["Detector.smax"];
  Params::vmin = fConfig["Detector.vmin"];
  Params::vmax = fConfig["Detector.vmax"];
  Params::contours_length_min = fConfig["Detector.contours_length_min"];
  Params::contours_length_max = fConfig["Detector.contours_length_max"];
  Params::LED_ratio_min = fConfig["Detector.LED_ratio_min"];
  Params::LED_ratio_max = fConfig["Detector.LED_ratio_max"];
  Params::LED_width_min = fConfig["Detector.LED_width_min"];
  Params::LED_width_max = fConfig["Detector.LED_width_max"];
  Params::marker_parallel_angle = fConfig["Detector.marker_parallel_angle"];
  Params::marker_vertical_angle = fConfig["Detector.marker_vertical_angle"];
  Params::marker_direction_angle = fConfig["Detector.marker_direction_angle"];
  Params::marker_ratio_min = fConfig["Detector.marker_ratio_min"];
  Params::marker_ratio_max = fConfig["Detector.marker_ratio_max"];
  Params::marker_size_min = fConfig["Detector.marker_size_min"];
  Params::marker_size_max = fConfig["Detector.marker_size_max"];
  // Transformer
  Params::transformer_gray_min = fConfig["Transformer.gray_min"];
  Params::transformer_gray_max = fConfig["Transformer.gray_max"];
  Params::transformer_area_min = fConfig["Transformer.area_min"];
  Params::transformer_area_max = fConfig["Transformer.area_max"];
  Params::transformer_c2_s_ratio_min = fConfig["Transformer.c2_s_ratio_min"];
  Params::transformer_c2_s_ratio_max = fConfig["Transformer.c2_s_ratio_max"];
  Params::transformer_ellipse_epsi = fConfig["Transformer.ellipse_epsi"];
  Params::transformer_ellipse_inlier_ratio =
      fConfig["Transformer.ellipse_inlier_ratio"];
  Params::transformer_ellipse_radius = fConfig["Transformer.ellipse_radius"];
  // target
  Params::target_color = fConfig["Target.color"];
  Params::target_size = fConfig["Target.size"];
  Params::target_bubing_shift_x = fConfig["Target.bubing.shift_x"];
  Params::target_bubing_shift_y = fConfig["Target.bubing.shift_y"];
  Params::target_bubing_shift_z = fConfig["Target.bubing.shift_z"];
  Params::target_bubing_L = fConfig["Target.bubing.L"];
  Params::target_shaobing_shift_x = fConfig["Target.shaobing.shift_x"];
  Params::target_shaobing_shift_y = fConfig["Target.shaobing.shift_y"];
  Params::target_shaobing_shift_z = fConfig["Target.shaobing.shift_z"];
  Params::target_shaobing_L = fConfig["Target.shaobing.L"];
  Params::target_yingxiong_shift_x = fConfig["Target.yingxiong.shift_x"];
  Params::target_yingxiong_shift_y = fConfig["Target.yingxiong.shift_y"];
  Params::target_yingxiong_shift_z = fConfig["Target.yingxiong.shift_z"];
  Params::target_yingxiong_L = fConfig["Target.yingxiong.L"];
  return 0;
}
