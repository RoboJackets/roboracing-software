#pragma once

#include <random>
#include <vector>

#include <ros/node_handle.h>

#include "bicycle_model.h"
#include "nearest_point_cache.h"
#include "planning_optimizer.h"

namespace rr {

class AnnealingPlanner : public PlanningOptimizer {
  public:
    struct Params {
        int n_path_segments;       // number of segments in a path
        int annealing_steps;       // number of timesteps to run simulated annealing
        double temperature_start;  // temperature parameter starts high and decreases
        double temperature_end;    // temperature at last iteration
        double k_dist;             // importance of distance in cost function
        double k_speed;            // importance of speed/straightness in cost fn
        double k_final_pose;       // importance of final position in cost function
        double k_angle;            // cost penalty for not going straight
        double collision_penalty;  // cost penalty for a collision
        double acceptance_scale;   // strictness for accepting bad paths
        double max_steering;       // max steering angle output
    };

    AnnealingPlanner(const ros::NodeHandle& nh, const NearestPointCache&, const BicycleModel&);

    ~AnnealingPlanner() = default;

    std::vector<double> Optimize(const CostFunction& cost_fn, const std::vector<double>& init_controls) override;

  private:
    /**
     * Find the current annealing temperature. This is used as the standard
     * deviation for SampleControls in addition to the normal usage in the SA
     * algorithm. Uses an exponential decay schedule.
     * @param t Iteration number in annealing procedure
     * @return temperature
     */
    double GetTemperature(unsigned int t);

    Params params_;
    std::uniform_real_distribution<double> uniform_01_;
    std::mt19937 rand_gen_;
};

}  // namespace rr
