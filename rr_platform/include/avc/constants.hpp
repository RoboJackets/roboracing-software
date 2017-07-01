#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cmath>
#include <opencv2/opencv.hpp>

namespace constants {

  // center-to-center in m
  double wheel_base = 0.37;

  double pixels_per_meter = 100;

  double camera_distance_max = 5.0; //distance to look ahead of the car
  double camera_distance_min = 0.5; //avoid the front bumper

}

#endif // CONSTANTS_HPP
