#pragma once

#include <tuple>

#include <ros/time.h>

#include "planner_types.hpp"


namespace rr {

class BicycleModel {
 public:
  BicycleModel(double wheel_base, double max_lateral_accel, double distance_increment, double max_speed,
               double max_steer_rate, const std::vector<int>& segment_sections);

  BicycleModel() = default;

  /*
   * RollOutPath: roll out a trajectory through the world.
   * Params:
   * control - vector with a steering value for each path segment
   */
  void RollOutPath(std::vector<PathPoint>& path_points, const std::vector<double>& control);

  /*
   * UpdateSteeringAngle: move the tracked steering angle of the car towards the value that
   * the car has been commanded to steer. This tracks the ROS time it is called, so it
   * should be used just before RollOutPath
   */
  void UpdateSteeringAngle(double commanded_angle);

  double GetCurrentSteeringAngle();

 private:
  /*
   * SteeringToSpeed: Calculate a desired speed from a steering angle based on 
   * a maximum acceptable lateral acceleration. See 
   * https://www.desmos.com/calculator/7nh83g8afj
   * Params:
   * steer_angle - proposed steering value to publish
   */
  double SteeringToSpeed(double steer_angle);

  /*
   * StepKinematics: calculate one distance-step of "simulated"
   * vehicle motion. Bicycle-model steering trig happens here.
   * Params:
   * steer_angle - steering angle in radians centered at 0
   * pose - initial (x,y,theta)
   */
  void StepKinematics(Pose& pose, const Pose& last_pose, double steer_angle);

  double IncrementSteering(double current, double target, double dt);


  const double wheel_base;
  const double max_lateral_accel;
  const std::vector<int> segment_sections;
  const double distance_increment;
  const double max_speed;
  const double max_steer_rate;

  double current_steering_;
  ros::Time last_steering_update_;
};

}  // namespace rr
