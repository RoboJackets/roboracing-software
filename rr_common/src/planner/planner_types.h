#ifndef RR_COMMON_PLANNER_TYPES_H
#define RR_COMMON_PLANNER_TYPES_H

#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

namespace rr {

using KdTreeMap = pcl::KdTreeFLANN<pcl::PointXYZ>;

struct Pose {
  double x;
  double y;
  double theta;

  Pose(double x, double y, double theta) : x(x), y(x), theta(theta) {}
  Pose() : x(0), y(0), theta(0) {}
};

struct PathPoint {
  Pose pose;
  double steer;
  double speed;

  PathPoint(const Pose& pose, double steer, double speed) 
    : pose(pose), steer(steer), speed(speed) {}
  PathPoint() : pose(), steer(0), speed(0) {}
};

struct PlannedPath {
  std::vector<double> control;  // input to a path
  std::vector<PathPoint> path;  // results of path rollout
  double cost;  // result of applying cost function
};

}  // namespace rr

#endif  // RR_COMMON_PLANNER_TYPES_H
