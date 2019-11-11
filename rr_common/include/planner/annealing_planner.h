#pragma once

#include <random>
#include <vector>

#include <ros/node_handle.h>

#include "bicycle_model.h"
#include "distance_checker.h"
#include "planner.h"
#include "planner_types.hpp"

namespace rr {

class AnnealingPlanner : public Planner {
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

    AnnealingPlanner(const ros::NodeHandle& nh, const DistanceChecker&, const BicycleModel&);

    ~AnnealingPlanner() = default;

    /**
     * Given a map of the world, find the best path through it
     * @param map A point cloud representation of the world around the car
     * @return PlannedPath object representing the plan for movement
     */
    PlannedPath Plan(const PCLMap& map) override;

  private:
    /**
     * Probabilistically jiggle a vector of control inputs (steering angles) to an
     * optimization neighbor
     * @param ctrl Out param, list of steering angles which we will test
     * @param source In param, previously-tested control vector
     * @param t Iteration number of simulated annealing algorithm
     */
    void SampleControls(std::vector<double>& ctrl, const std::vector<double>& source, unsigned int t);

    /**
     * Find the current annealing temperature. This is used as the standard
     * deviation for SampleControls in addition to the normal usage in the SA
     * algorithm. Uses an exponential decay schedule.
     * @param t Iteration number in annealing procedure
     * @return temperature
     */
    double GetTemperature(unsigned int t);

    /**
     * Calculate the total cost of a path (see Params for more detail on cost
     * coefficients)
     * @param path list of (pose, steering, speed) tuples
     * @param map point cloud map
     * @return cost of the path
     */
    double GetCost(const PlannedPath& path, const PCLMap& map);

    Params params;
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
