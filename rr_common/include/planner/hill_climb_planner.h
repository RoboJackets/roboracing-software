#pragma once

#include <optional>
#include <random>

#include <ros/node_handle.h>

#include "bicycle_model.h"
#include "nearest_point_cache.h"
#include "planner_types.hpp"
#include "planning_optimizer.h"

namespace rr {

class HillClimbPlanner : public PlanningOptimizer {
  public:
    HillClimbPlanner(const ros::NodeHandle& nh, const NearestPointCache&, const BicycleModel&);

    OptimizedTrajectory Optimize(const PCLMap& map) override;

  private:
    void FillObstacleCosts(OptimizedTrajectory& planned_path) const;
    void JitterControls(std::vector<double>& ctrl, double stddev);

    int state_dim_;             // number of control inputs in state
    double k_dist_;             // importance of distance in cost function
    double k_speed_;            // importance of speed/straightness in cost fn
    double k_angle_;            // cost penalty for heading angle
    double k_steering_;         // cost penalty for steering angle
    double collision_penalty_;  // cost penalty for a collision
    double max_steering_;       // max steering angle output
    int num_workers_;           // number of threads to run in parallel
    int num_restarts_;          // total number of hill descents to do
    double neighbor_stddev_;    // standard deviation of noise added in neighbor function
    int local_optimum_tries_;   // we are at a local optimum if we try this many times with no improvement

    NearestPointCache distance_checker_;
    BicycleModel model_;

    std::normal_distribution<double> normal_distribution_;
    std::mt19937 rand_gen_;

    OptimizedTrajectory previous_best_plan_;
};

}  // namespace rr
