#ifndef RR_COMMON_PLANNER_H
#define RR_COMMON_PLANNER_H

#include <string>
#include <random>
#include <tuple>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

namespace rr {
namespace planning {

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

  PathPoint(Pose pose, double steer, double speed) : pose(pose), steer(steer), speed(speed) {}
  PathPoint() : pose(), steer(0), speed(0) {}
};

struct PlannedPath {
  std::vector<double> control;  // input to a path
  std::vector<PathPoint> path;  // results of path rollout
  double cost;  // result of applying cost function
};

struct CollisionBox {
  double length_front;  // distance from origin to front edge
  double length_back;  // ditto for back edge, etc.
  double width_left;
  double width_right;
};

class Planner
{
public:

  struct Params {
    // robot
    double wheel_base;
    double lateral_accel;
    CollisionBox collision_box;

    // path generation
    int n_path_segments;
    std::vector<double> segment_distances;
    std::vector<double> steer_limits;
    std::vector<double> steer_stddevs;

    // path filtering
    double obstacle_search_radius;
    double path_similarity_cutoff;
    double max_relative_cost;
    double k_dist;
    double k_speed;

    // CPU usage tradeoff
    int n_control_samples;
    double distance_increment;

    // control
    int smoothing_array_size;
    double max_speed;

    // comp experimental things
    double left_dist_weight;
    double right_dist_weight;
    double obs_dist_slow_thresh;
    double obs_dist_slow_ratio;
  };


  Planner(const Params& params);

  ~Planner() = default;

  PlannedPath Plan(const KdTreeMap& kd_tree_map);

private:

  /*
   * steeringSample: get a random steering value from a normal
   * distribution. Each path stage has a potentially different bell
   * curve. If the value is out of bounds, try again
   * Params:
   * stage - the path stage or control vector dimension
   */
  double SampleSteering(int stage);

  /*
   * SteeringToSpeed: Calculate a desired speed from a steering angle based on a maximum
   * acceptable lateral acceleration. See https://www.desmos.com/calculator/7nh83g8afj
   * Params:
   * steer_angle - proposed steering value to publish
   */
  double SteeringToSpeed(double steer_angle);

  /*
   * StepKinematics: calculate one distance-step of "simulated"
   * vehicle motion. Bicycle-model steering trig happens here.
   * Params:
   * steer_angle - steering angle in radians centered at 0
   * pose - (x,y,theta)
   */
  Pose StepKinematics(const Pose& pose, double steer_angle);

  /*
   * RollOutPath: roll out a trajectory through the world using a given control vector.
   * Params:
   * - control - vector of dimension N_PATH_SEGMENTS with a steering value for 
                 each stage/dimension
   */
  std::vector<PathPoint> RollOutPath(const std::vector<double>& control);

  /*
   * GetCollisionDistance: calculate the shortest distance from any part of the robot to any
   * point in the map.
   * Params:
   * pose - Future pose relative to current pose. The robot's initial state is (0, 0, 0)
   * kdtree - nearest-neighbors-searchable map
   * Returns:
   * bool - collision detected
   * double - clearing distance to nearest obstacle
   */
  std::tuple<bool, double> GetCollisionDistance(const Pose& pose, const KdTreeMap& kd_tree_map);

  /*
   * GetCost: calculate the total cost of a path.
   * cost = path integral 1 / (k_dist * obstacle_distance + k_speed * speed)
   * Params:
   * path - list of (pose, steering, speed) tuples
   * kdtree - nearest-neighbors-searchable map
   * Returns:
   * bool - collision detected
   * double - cost
   */
  std::tuple<bool, double> GetCost(const std::vector<PathPoint>& path, const KdTreeMap& kd_tree_map);

  /*
   * GetLocalMinima: A "clustering" function. Given a set of paths, find local cost
   * minima in the space of control vectors and return these indices from the input matrix.
   * Params:
   * paths - list of PlannedPaths
   * Return:
   * list of indices of selected paths
   */
  std::vector<int> GetLocalMinima(const std::vector<std::reference_wrapper<PlannedPath>>& paths);

  /* 
   * FilterOutput: Apply a filter to the outputs of this planner over time
   */
  double FilterOutput(double this_steer);


  // algorithm parameters
  const Params params;

  // mutable state
  std::vector<double> prev_steering_angles_;
  int prev_steering_angles_index_;

  std::vector<std::normal_distribution<float> > steering_gaussians_;
  std::mt19937 rand_gen_;
};


}  // namespace planning
}  // namespace rr

#endif  // RR_COMMON_PLANNER_H
