#ifndef RR_COMMON_ANNEALING_PLANNER_H
#define RR_COMMON_ANNEALING_PLANNER_H

#include <random>
#include <tuple>
#include <vector>

#include "planner.h"
#include "planner_types.h"
#include "distance_checker.h"
#include "bicycle_model.h"

namespace rr {

class AnnealingPlanner : public Planner
{
public:

  struct Params {
    unsigned int n_path_segments;
    unsigned int annealing_steps;
    double temperature_start;
    double temperature_end;
    double k_dist;
    double k_speed;
    double k_similarity;
    double k_final_pose;
    double collision_penalty;
    double max_steering;
    double acceptance_scale;
  };

  AnnealingPlanner(const DistanceChecker&, const BicycleModel&, const Params&);

  ~AnnealingPlanner() = default;

  /*
   * Plan: given a map of the world, find the best path through it
   */
  PlannedPath Plan(const KdTreeMap& kd_tree_map);

private:

  std::vector<double> SampleControls(const std::vector<double>& last, unsigned int t);

  double GetTemperature(unsigned int t);

  /*
   * GetCost: calculate the total cost of a path.
   * TODO document cost function here
   * Params:
   * path - list of (pose, steering, speed) tuples
   * kdtree - nearest-neighbors-searchable map
   * Returns:
   * bool - collision detected
   * double - cost
   */
  std::tuple<bool, double> GetCost(const std::vector<PathPoint>& path, const KdTreeMap& kd_tree_map);

  const Params params;

  DistanceChecker distance_checker_;
  BicycleModel model_;

  std::normal_distribution<double> steering_gaussian_;
  std::uniform_real_distribution<double> uniform_01_;
  std::mt19937 rand_gen_;

  unsigned int last_path_idx_;
  std::vector<PlannedPath> path_pool_;
};

}  // namespace rr

#endif  // RR_COMMON_ANNEALING_PLANNER_H
