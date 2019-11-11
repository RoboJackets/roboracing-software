#include "planner/random_sample_planner.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <parameter_assertions/assertions.h>
#include <flann/flann.hpp>

double last_speed = 0;

namespace rr {

RandomSamplePlanner::RandomSamplePlanner(const ros::NodeHandle& nh, const DistanceChecker& distance_checker,
                                         const BicycleModel& model)
      : distance_checker_(distance_checker), model_(model) {
    using assertions::getParam;

    getParam(nh, "n_path_segments", params.n_path_segments);
    getParam(nh, "steer_limits", params.steer_limits);
    getParam(nh, "steer_stddevs", params.steer_stddevs);
    getParam(nh, "path_similarity_cutoff", params.path_similarity_cutoff);
    getParam(nh, "max_relative_cost", params.max_relative_cost);
    getParam(nh, "k_dist", params.k_dist);
    getParam(nh, "k_speed", params.k_speed);
    getParam(nh, "n_control_samples", params.n_control_samples);
    getParam(nh, "smoothing_array_size", params.smoothing_array_size);
    getParam(nh, "obs_dist_slow_thresh", params.obs_dist_slow_thresh);
    getParam(nh, "obs_dist_slow_ratio", params.obs_dist_slow_ratio);

    prev_steering_angles_.resize(params.smoothing_array_size, 0);
    prev_steering_angles_index_ = 0;

    for (double d : params.steer_stddevs) {
        steering_gaussians_.emplace_back(0, d);
    }
    rand_gen_ = std::mt19937(std::random_device{}());
}

double RandomSamplePlanner::SampleSteering(int stage) {
    double angle;
    do {
        auto& gaussian = steering_gaussians_[stage];
        angle = gaussian(rand_gen_);
    } while (std::abs(angle) > params.steer_limits[stage]);

    return angle;
}

std::tuple<bool, double> RandomSamplePlanner::GetCost(const std::vector<PathPoint>& path) {
    double denominator = 0;
    bool is_collision = false;

    for (const auto& path_point : path) {
        auto opt_dist = distance_checker_.GetCollisionDistance(path_point.pose);

        if (!opt_dist) {
            is_collision = true;
            break;
        }
        denominator += params.k_dist * opt_dist.value() + params.k_speed * path_point.speed;
    }

    if (denominator == 0.0) {
        denominator = 0.001;
    }

    return std::make_tuple(is_collision, 1.0 / denominator);
}

std::vector<int> RandomSamplePlanner::GetLocalMinima(const std::vector<std::reference_wrapper<PlannedPath>>& plans) {
    std::vector<int> local_minima_indices;
    auto n_samples = plans.size();
    auto n_path_segments = plans[0].get().control.size();

    if (n_samples > 0) {
        // fill flann-format sample matrix
        double sampleArray[n_samples * n_path_segments];
        for (int i = 0; i < n_samples; i++) {
            for (int j = 0; j < n_path_segments; j++) {
                sampleArray[i * n_path_segments + j] = plans[i].get().control[j];
            }
        }
        flann::Matrix<double> samples(sampleArray, n_samples, n_path_segments);

        // construct FLANN index (nearest neighbors preprocessing)
        auto index_params = flann::KDTreeSingleIndexParams();
        // TODO determine if L1 (Manhattan) or L2 (Euclidean) distance is better
        flann::Index<flann::L1<double>> flannIndex(samples, index_params);
        flannIndex.buildIndex();

        // Initialize group membership list
        std::vector<bool> samples_known_status(n_samples, false);

        // initialize input/output parameters for radius searches
        flann::Matrix<int> indices(new int[n_samples], 1, n_samples);
        flann::Matrix<double> dists(new double[n_samples], 1, n_samples);
        flann::Matrix<double> ctrl(new double[n_path_segments], 1, n_path_segments);
        flann::SearchParams searchParams(1);
        const double& searchRadius = params.path_similarity_cutoff;

        for (int i = 0; i < n_samples; i++) {
            if (samples_known_status[i]) {
                continue;
            }

            // perform radius search
            for (int d = 0; d < n_path_segments; d++) {
                ctrl[0][d] = samples[i][d];
            }
            int n_neighbors = flannIndex.radiusSearch(ctrl, indices, dists, searchRadius, searchParams);

            // Iterate through neighbors, tracking if lower costs exist.
            // Any neighbors in radius that have higher costs are not local minima.
            // i is the query, j is a neighbor
            bool any_better_in_area = false;
            for (int j = 0; j < n_neighbors; j++) {
                int found_index = indices[0][j];

                if (found_index == i) {  // handle index of query later
                    continue;
                }

                if (plans[found_index].get().cost < plans[i].get().cost) {
                    any_better_in_area = true;
                } else {
                    samples_known_status[found_index] = true;  // known not minimum
                }
            }

            samples_known_status[i] = true;
            if (!any_better_in_area) {
                // includes regions with only one point
                local_minima_indices.push_back(i);
            }
        }
    }

    return local_minima_indices;
}

double RandomSamplePlanner::FilterOutput(double this_steer) {
    // update circular buffer
    prev_steering_angles_[prev_steering_angles_index_] = this_steer;
    prev_steering_angles_index_ = (prev_steering_angles_index_ + 1) % params.smoothing_array_size;

    // sort array of previous vectors in ascending order from start to midpoint to
    // find median
    auto angles = prev_steering_angles_;  // make a fresh copy
    int median_index = params.smoothing_array_size / 2;
    std::nth_element(angles.begin(), angles.begin() + median_index, angles.end());
    const double& median = angles[median_index];

    return median;
}

PlannedPath RandomSamplePlanner::Plan(const PCLMap& map) {
    // allocate planned paths
    std::vector<PlannedPath> plans;

    distance_checker_.SetMap(map);

    // fill planned paths and determine best cost
    double best_cost = 1e10;
    for (int i = 0; i < params.n_control_samples; i++) {
        plans.emplace_back();
        auto& plan = plans.back();

        for (int stage = 0; stage < params.n_path_segments; stage++) {
            plan.control.push_back(SampleSteering(stage));
        }

        model_.RollOutPath(plan.control, plan.path);

        bool collision;
        std::tie(collision, plan.cost) = GetCost(plan.path);

        if (collision) {
            plans.pop_back();
            continue;
        }

        best_cost = std::min(best_cost, plan.cost);
    }

    std::vector<std::reference_wrapper<PlannedPath>> good_plans;

    // filter by cost (compared to best cost)
    for (auto& plan : plans) {
        if (plan.cost <= best_cost * params.max_relative_cost) {
            good_plans.push_back(std::ref(plan));
        }
    }

    if (good_plans.size() < 2) {
        std::cout << "[Planner] Warning: found " << good_plans.size() << " good paths" << std::endl;
    }

    PlannedPath fallback_plan;
    fallback_plan.has_collision = true;

    if (good_plans.empty()) {
        std::cout << "[Planner] Using fallback plan" << std::endl;
        good_plans.push_back(std::ref(fallback_plan));
    }

    auto local_minima_indices = GetLocalMinima(good_plans);

    // TODO implement smarter path selection. For now, choose the straighest path
    int best_index = -1;
    double best_curviness = 0;
    for (int i : local_minima_indices) {
        double curviness = 0;  // square of euclidean distance from zero turning
        for (auto x : good_plans[i].get().control) {
            curviness += x * x;
        }

        if (best_index == -1 || curviness < best_curviness) {
            best_index = i;
            best_curviness = curviness;
        }
    }

    PlannedPath& best_plan = good_plans[best_index];

    bool is_collision;
    double min_dist = 10000;
    for (const auto& path_point : best_plan.path) {
        auto opt_dist = distance_checker_.GetCollisionDistance(path_point.pose);

        if (*opt_dist < min_dist) {
            min_dist = *opt_dist;
        }
    }

    if (min_dist < params.obs_dist_slow_thresh) {
        best_plan.path[0].speed *= params.obs_dist_slow_ratio;
    }

    double this_speed = best_plan.path[0].speed;

    if (last_speed > 0 && this_speed < 0) {
        best_plan.path[0].speed = last_speed;
    }

    last_speed = this_speed;

    std::cout << "Planner found " << local_minima_indices.size() << " local minima. "
              << "Best cost is " << best_plan.cost << std::endl;

    return best_plan;
}

}  // namespace rr
