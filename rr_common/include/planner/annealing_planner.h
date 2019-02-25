#pragma once

#include <random>
#include <vector>

#include "planner.h"
#include "planner_types.hpp"
#include "distance_checker.h"
#include "bicycle_model.h"

namespace rr {

class AnnealingPlanner : public Planner
{
public:

  struct Params {
    unsigned int n_path_segments;  // number of segments in a path
    unsigned int annealing_steps;  // number of timesteps to run simulated annealing
    double temperature_start;  // temperature parameter starts high and decreases
    double temperature_end;
    double k_dist;  // importance of distance in cost function
    double k_speed;  // importance of speed/straightness in cost fn
    double k_final_pose;  // importance of final position in cost function
    double backwards_penalty;
    double collision_penalty;  // cost fn penalty for a collision
    double acceptance_scale;  // strictness for accepting bad paths
    double max_steering;  // max steering angle output
  };

  AnnealingPlanner(const DistanceChecker&, const BicycleModel&, const Params&);

  ~AnnealingPlanner() = default;

  /*
   * Plan: given a map of the world, find the best path through it
   */
  PlannedPath Plan(const PCLMap& map);

private:

  void SampleControls(std::vector<double>& ctrl, const std::vector<double>& source,
                      unsigned int t);

  double GetTemperature(unsigned int t);

  /*
   * GetCost: calculate the total cost of a path
   * Params:
   *  path - list of (pose, steering, speed) tuples
   *  kd_tree_map - nearest-neighbors-searchable map
   * Returns:
   *  cost of the path
   */
  double GetCost(const PlannedPath& path, const PCLMap& map);

  const Params params;

  DistanceChecker distance_checker_;
  BicycleModel model_;

  std::normal_distribution<double> steering_gaussian_;
  std::uniform_real_distribution<double> uniform_01_;
  std::mt19937 rand_gen_;

  unsigned int last_path_idx_;
  std::vector<PlannedPath> path_pool_;

  double max_path_length_;
};

}  // namespace rr
