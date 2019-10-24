//
// Created by liming on 12/26/17.
//

#include "Params.h"

/// Camera
float Params::camera_width = 640;
float Params::camera_height = 480;
float Params::camera_fps = 30;
float Params::camera_fx = 500;
float Params::camera_fy = 500;
float Params::camera_cx = 320;
float Params::camera_cy = 240;
float Params::camera_k1 = 0;
float Params::camera_k2 = 0;
float Params::camera_k3 = 0;

/// ImageFrame
int Params::blur_sz = 5;
float Params::blur_sigma = 0.4;

/// Detect
int Params::hmin = 90;
int Params::hmax = 110;
int Params::blue_hmin = 90;
int Params::blue_hmax = 110;
int Params::red_hmin = 90;
int Params::red_hmax = 110;
int Params::smin = 120;
int Params::smax = 255;
int Params::vmin = 140;
int Params::vmax = 255;
int Params::contours_length_min = 10;
int Params::contours_length_max = 300;
float Params::LED_ratio_min = 2;
float Params::LED_ratio_max = 20;
float Params::LED_width_min = 7;
float Params::LED_width_max = 100;
float Params::marker_parallel_angle = 6;
float Params::marker_vertical_angle = 10;
float Params::marker_direction_angle = 45;
float Params::marker_ratio_min = 0.8;
float Params::marker_ratio_max = 5;
float Params::marker_size_min = 10;
float Params::marker_size_max = 200;

/// Transform
int Params::transformer_template_width = 60;
int Params::transformer_template_height = 30;
float Params::transformer_template_score_thres = 2000;
int Params::transformer_hmin = 80;
int Params::transformer_hmax = 120;
int Params::transformer_gray_min = 100;
int Params::transformer_gray_max = 255;
int Params::transformer_area_min = 64;
int Params::transformer_area_max = 40000;
float Params::transformer_c2_s_ratio_min = 8;
float Params::transformer_c2_s_ratio_max = 20;
float Params::transformer_ellipse_epsi = 0.2;
float Params::transformer_ellipse_inlier_ratio = 0.9;
float Params::transformer_ellipse_radius = 0.035;
float Params::transformer_big_marker_size = 0.225;
float Params::transformer_small_marker_size = 0.12;

/// target
int Params::target_color = 1;
int Params::target_size = 1;

float Params::target_bubing_shift_x = 0.125;
float Params::target_bubing_shift_y = 0.198;
float Params::target_bubing_shift_z = 0.16;
float Params::target_bubing_L = 0.16;

float Params::target_shaobing_shift_x = 0.125;
float Params::target_shaobing_shift_y = 0.198;
float Params::target_shaobing_shift_z = 0.16;
float Params::target_shaobing_L = 0.16;

float Params::target_yingxiong_shift_x = 0.125;
float Params::target_yingxiong_shift_y = 0.198;
float Params::target_yingxiong_shift_z = 0.16;
float Params::target_yingxiong_L = 0.16;
