#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cmath>
#include <opencv2/opencv.hpp>

namespace constants {

  // center-to-center in m
  double wheel_base = 0.37;

  // axel-to-axel in m
  double chassis_length = 0.43;

  double steering_degress_per_pwm = 1.5;

  double steering_radians_per_pwm = steering_degress_per_pwm * M_PI / 180.;

  double pixels_per_meter = 100;

  double world_width_meters = 8;
  double world_height_meters = 16;
  int world_width = pixels_per_meter * world_width_meters;
  int world_height = pixels_per_meter * world_height_meters;

  double origin_x = world_height / 2;
  double origin_y = world_width / 2;

  int car_length_pixels = chassis_length * pixels_per_meter;

  double max_speed = 5; //meters per second
  double laser_scan_time = 1/15.0; //seconds
  double camera_scan_time = 1/60.0; //seconds
  double reaction_time = .3; //seconds, edit me please
  double laser_step_size = .0174532925; // 1 degree in radians

  //positive x is forward, y is left, z is up

  double camera_laser_offset = .33; //meters
  int image_pixel_delta = camera_laser_offset * pixels_per_meter;  //pixels
  int image_size = 800; //pixels
  double radial_steps = 60;

  cv::Mat H = (cv::Mat_<float>(3, 3) << -2.083084279737351, -15.21667662881241, 2140.192767483677,
                                         6.439293542825908e-15, -24.26180514046955, 3708.527196653065,
                                         1.517883041479706e-17, -0.01016138673042565, 1);

cv::Mat H = (cv::Mat_<float>(3,3) << -0.6159268185076195, -8.608702653062393, 662.9430651719022, 0.5362046116462343, -18.00046627466312, 1363.408535122169, 0.0006176242040685544, -0.02125830990529155, 1);
}

#endif // CONSTANTS_HPP
