#include "distance_checker.h"

namespace rr {

DistanceChecker::DistanceChecker(double length_front, double length_back,
                                 double width_left, double width_right,
                                 double obstacle_search_radius)
  : length_front_(length_front),
    length_back_(length_back),
    width_left_(width_left),
    width_right_(width_right),
    obstacle_search_radius_(obstacle_search_radius) {}

std::tuple<bool, double> DistanceChecker::GetCollisionDistance(const Pose& pose,
    const KdTreeMap& kd_tree_map) {
  double cosTheta = std::cos(pose.theta);
  double sinTheta = std::sin(pose.theta);

  double robotCenterX = (length_front_ - length_back_) / 2.;
  double robotCenterY = (width_left_ - width_right_) / 2.;
  double searchX = pose.x + robotCenterX * cosTheta - robotCenterY * sinTheta;
  double searchY = pose.y + robotCenterX * sinTheta + robotCenterY * cosTheta;

  double halfLength = (length_front_ + length_back_) / 2.;
  double halfWidth = (width_left_ + width_right_) / 2.;
  double cornerDist = std::sqrt(halfLength*halfLength + halfWidth*halfWidth);

  pcl::PointXYZ searchPoint(searchX, searchY, 0);
  std::vector<int> pointIdxs;
  std::vector<float> squaredDistances;
  int nResults = kd_tree_map.radiusSearch(searchPoint, obstacle_search_radius_,
                                          pointIdxs, squaredDistances);

  bool collision = false;
  double min_dist = obstacle_search_radius_;
  if (nResults > 0) {
    // convert each point to robot reference frame and distances
    const auto& cloud = *(kd_tree_map.getInputCloud());
    for (int i : pointIdxs) {
      const auto& point = cloud[i];

      // note that this is inverse kinematics here. We have origin -> robot but
      // want robot -> origin
      double offsetX = point.x - searchX;
      double offsetY = point.y - searchY;
      double x =  cosTheta * offsetX + sinTheta * offsetY;
      double y = -sinTheta * offsetX + cosTheta * offsetY;

      // collisions
      if (std::abs(x) <= halfLength && std::abs(y) <= halfWidth) {
        collision = true;
        break;
      }

      // find distance, in several cases
      double dist;
      if (std::abs(x) > halfLength) {
        // not alongside the robot
        if (std::abs(y) > halfWidth) {
          // closest to a corner
          double cornerX = halfLength * ((x < 0) ? -1 : 1);
          double cornerY = halfWidth * ((y < 0) ? -1 : 1);
          double dx = x - cornerX;
          double dy = y - cornerY;
          dist = std::sqrt(dx*dx + dy*dy);
        } else {
          // directly in front of or behind robot
          dist = std::abs(x) - halfLength;
        }
      } else {
        // directly to the side of the robot
        dist = std::abs(y) - halfWidth;
      }
      min_dist = std::min(min_dist, dist);
    }
  }

  return std::make_tuple(collision, min_dist - cornerDist);
}

std::tuple<bool, double> DistanceChecker::GetCollisionDistance(const Pose& pose,
    const pcl::PointXYZ& point) {

}

}  // namespace rr

#endif  // RR_COMMON_DISTANCE_CHECKER_H
