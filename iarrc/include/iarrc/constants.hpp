#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cmath>

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

double max_speed = 5; //meters per second
double laser_scan_time = 1/15.0; //seconds
double camera_scan_time = 1/60.0; //seconds
double reaction_time = .3; //seconds, edit me please
double laser_step_size = .0174532925; // 1 degree in radians

//positive x is forward, y is left, z is up

double camera_laser_offset = .33; //meters
int image_pixel_delta = camera_laser_offset * pixels_per_meter;  //pixels
int image_size = 800; //pixels
double radial_steps = 15;
}

#endif // CONSTANTS_HPP
