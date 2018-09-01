#ifndef RR_COMMON_PLANNER_H
#define RR_COMMON_PLANNER_H

#include <string>
#include <random>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

namespace rr {
namespace planning {

using KdTreeMap = pcl::KdTreeFLANN<pcl::PointXYZ>;

struct Pose {
  double x;
  double y;
  double theta;
};

struct PathPoint {
  Pose pose;
  double steer;
  double speed;
};

struct PlannedPath {
  std::vector<double> control;  // input to a path
  std::vector<PathPoint> path;  // results of path rollout
  double cost;  // result of applying cost function
};

struct CollisionBox {
  double lengthFront;  // distance from origin to front edge
  double lengthBack;  // ditto for back edge, etc.
  double widthLeft;
  double widthRight;
};

class Planner
{
public:

  struct Params {
    int n_path_segments;
    int n_control_samples;
    int smoothing_array_size;
    double distance_increment;
    double max_speed;
    double wheel_base;
    double path_similarity_cutoff;
    double max_relative_cost;
    double obstacle_search_radius;
    std::vector<double> segment_distances;
    std::vector<double> steer_limits;
    std::vector<double> steer_stddevs;
    CollisionBox collision_box;
  };


  Planner(const Params &params);

  ~Planner() = default;

  PlannedPath Plan(const KdTreeMap &kd_tree_map);

private:

  double SampleSteering(int stage);

  double SteeringToSpeed(double steer_angle);

  /*
   * StepKinematics: calculate one distance-step of "simulated"
   * vehicle motion. Bicycle-model steering trig happens here.
   * Params:
   * steer_angle - steering angle in radians centered at 0
   * pose - (x,y,theta)
   */
  Pose StepKinematics(const Pose &pose, double steer_angle);

  std::vector<PathPoint> RollOutPath(const std::vector<double> &control);

  /*
   * GetCollisionDistance: calculate the shortest distance from any part of the robot to any
   * point in the map.
   * Params:
   * - pose - Future pose relative to current pose. The robot's initial state is (0, 0, 0)
   * - kdtree - nearest-neighbors-searchable map
   * Returns:
   * - bool, collision detected
   * - double, clearing distance to nearest obstacle
   */
  std::tuple<bool, double> GetCollisionDistance(const Pose &pose, const KdTreeMap &kd_tree_map);

  std::tuple<bool, double> GetCost(const std::vector<double> &control, const KdTreeMap &kd_tree_map);

  std::vector<int> GetLocalMinima(const std::vector<PlannedPath> &paths);


  // algorithm parameters
  const Params params;

  // mutable state
  std::vector<double> prev_steering_angles_;
  int prev_steering_angles_index_;

  std::vector<std::normal_distribution<double>> steering_gaussians_;
  std::mt19937 rand_gen_;
};


}  // namespace planning
}  // namespace rr

#endif //RR_COMMON_PLANNER_H
