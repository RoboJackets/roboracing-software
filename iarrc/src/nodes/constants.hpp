#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cmath>

namespace constants {

// center-to-center in m
double wheel_base = 0.26;

// axel-to-axel in m
double chassis_length = 0.325;

double steering_degress_per_pwm = 1.5;

double steering_radians_per_pwm = steering_degress_per_pwm * M_PI / 180.;

}

#endif // CONSTANTS_HPP
