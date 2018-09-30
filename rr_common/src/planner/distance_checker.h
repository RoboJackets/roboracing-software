#ifndef RR_COMMON_DISTANCE_CHECKER_H
#define RR_COMMON_DISTANCE_CHECKER_H

#include <tuple>

#include "planner_types.h"

namespace rr {

class DistanceChecker {
 public:
  DistanceChecker(double length_front, double length_back, double width_left,
                  double width_right, double obstacle_search_radius);

  /*
   * Calculate the shortest distance from any part of the robot to any point 
   * in the map. Pose is future relative to current/origin (0,0,0)
   * Returns:
   * bool - collision detected
   * double - clearing distance to nearest obstacle
   */
  std::tuple<bool, double> GetCollisionDistance(const Pose& pose, 
                                                const KdTreeMap& kd_tree_map);

  std::tuple<bool, double> GetCollisionDistance(const Pose& pose, 
                                                const pcl::PointXYZ& point);

 private:
  const double length_front_;  // distance from origin to front edge
  const double length_back_;  // ditto for back edge, etc.
  const double width_left_;
  const double width_right_;
  const double obstacle_search_radius_;
};

}  // namespace rr

#endif  // RR_COMMON_DISTANCE_CHECKER_H
