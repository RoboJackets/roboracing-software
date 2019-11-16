#pragma once

#include <random>
#include <tuple>
#include <vector>

#include <ros/node_handle.h>

#include "bicycle_model.h"
#include "nearest_point_cache.h"
#include "planner_types.hpp"
#include "planning_optimizer.h"

namespace rr {

class RandomSamplePlanner : public PlanningOptimizer {
  public:
    struct Params {
        // path generation
        int n_path_segments;
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

        // comp experimental things
        double obs_dist_slow_thresh;
        double obs_dist_slow_ratio;
    };

    RandomSamplePlanner(const ros::NodeHandle& nh, const NearestPointCache&, const BicycleModel&);

    ~RandomSamplePlanner() = default;

    /*
     * Optimize: given a map of the world, find the best path through it
     */
    OptimizedTrajectory Optimize(const PCLMap& map) override;

  private:
    /*
     * SampleSteering: get a random steering value from a normal
     * distribution. Each path stage has a potentially different bell
     * curve. If the value is out of bounds, try again
     * Params:
     * stage - the path stage or control vector dimension
     */
    double SampleSteering(int stage);

    /*
     * GetCost: calculate the total cost of a path.
     * cost = path integral 1 / (k_dist * obstacle_distance + k_speed * speed)
     * Params:
     * path - list of (pose, steering, speed) tuples
     * Returns:
     * bool - collision detected
     * double - cost
     */
    std::tuple<bool, double> GetCost(const std::vector<PathPoint>& path);

    /*
     * GetLocalMinima: A "clustering" function. Given a set of paths, find local
     * cost minima in the space of control vectors and return these indices from
     * the input list.
     * Params:
     * paths - list of all PlannedPaths
     * mask - true if OptimizedTrajectory should be considered (false if collision)
     * Return:
     * list of indices of selected paths
     */
    std::vector<int> GetLocalMinima(const std::vector<std::reference_wrapper<OptimizedTrajectory>>& plans);

    /*
     * FilterOutput: Apply a filter to the outputs of this planner over time
     */
    double FilterOutput(double this_steer);

    // algorithm parameters
    Params params;

    NearestPointCache distance_checker_;
    BicycleModel model_;

    // mutable state
    std::vector<double> prev_steering_angles_;
    int prev_steering_angles_index_;

    std::vector<std::normal_distribution<float>> steering_gaussians_;
    std::mt19937 rand_gen_;
};

}  // namespace rr
