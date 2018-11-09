#ifndef RR_COMMON_KINEMATIC_BICYCLE_MODEL_H
#define RR_COMMON_KINEMATIC_BICYCLE_MODEL_H

#include <tuple>

#include "planner_types.h"

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


  double wheel_base_;
  double max_lateral_accel_;
  std::vector<int> segment_sections_;
  double distance_increment_;
  double max_speed_;
  double max_steer_rate_;
};

}  // namespace rr

#endif  // RR_COMMON_KINEMATIC_BICYCLE_MODEL_H
