#include "planner/annealing_planner.h"

#include <parameter_assertions/assertions.h>

namespace rr {

AnnealingPlanner::AnnealingPlanner(const ros::NodeHandle& nh, const NearestPointCache& c, const BicycleModel& m)
      : distance_checker_(c), model_(m) {
    using assertions::getParam;

    getParam(nh, "n_path_segments", params.n_path_segments, { assertions::greater(0) });
    getParam(nh, "annealing_steps", params.annealing_steps, { assertions::greater(0) });
    getParam(nh, "k_dist", params.k_dist, { assertions::greater_eq(0.0) });
    getParam(nh, "k_speed", params.k_speed, { assertions::greater_eq(0.0) });
    getParam(nh, "k_final_pose", params.k_final_pose, { assertions::greater_eq(0.0) });
    getParam(nh, "k_angle", params.k_angle, { assertions::greater_eq(0.0) });
    getParam(nh, "collision_penalty", params.collision_penalty, { assertions::greater_eq(0.0) });
    getParam(nh, "max_steering", params.max_steering, { assertions::greater(0.0) });
    getParam(nh, "acceptance_scale", params.acceptance_scale, { assertions::greater(0.0) });
    getParam(nh, "temperature_start", params.temperature_start, { assertions::greater(0.0) });
    getParam(nh, "temperature_end", params.temperature_end,
             { assertions::greater(0.0), assertions::less(params.temperature_start) });

    for (int i = 0; i < params.annealing_steps; i++) {
        rr::OptimizedTrajectory& path = path_pool_.emplace_back();
        path.control = std::vector<double>(params.n_path_segments, 0);
        model_.RollOutPath(path.control, path.path);
        path.dists = std::vector<double>(path.path.size(), 0.0);
        path.cost = std::numeric_limits<double>::max();
    }
    last_path_idx_ = 0;
    max_path_length_ = path_pool_[0].path.back().pose.x;
}

void AnnealingPlanner::SampleControls(std::vector<double>& ctrl, const std::vector<double>& source, unsigned int t) {
    for (size_t i = 0; i < ctrl.size(); ++i) {
        double stddev = GetTemperature(t);
        double new_control_i = source[i] + (steering_gaussian_(rand_gen_) * stddev);
        ctrl[i] = std::clamp(new_control_i, -params.max_steering, params.max_steering);
    }
}

double AnnealingPlanner::GetTemperature(unsigned int t) {
    static const double K = std::log(params.temperature_end / params.temperature_start) / params.annealing_steps;

    return params.temperature_start * std::exp(K * t);
}

double AnnealingPlanner::GetCost(const OptimizedTrajectory& planned_path, const PCLMap& map) {
    double cost = 0;

    const std::vector<PathPoint>& path = planned_path.path;
    const auto& last_path = path_pool_[last_path_idx_].path;

    for (size_t i = 0; i < path.size(); i++) {
        double dist = planned_path.dists[i];
        bool collision_here = (dist <= 0);

        if (collision_here) {
            cost += params.collision_penalty * (path.size() - i);
            break;
        } else {
            cost -= params.k_dist * std::log(std::max(0.01, dist));
            cost -= params.k_speed * path[i].speed;
            cost += std::abs(path[i].pose.theta) * params.k_angle;
        }
    }

    double dd = max_path_length_ - path.back().pose.x;
    cost += params.k_final_pose * dd * dd;

    return cost;
}

OptimizedTrajectory AnnealingPlanner::Optimize(const PCLMap& map) {
    path_pool_[0] = path_pool_[last_path_idx_];

    unsigned int best_idx = 0;
    unsigned int state_idx = 0;

    auto best_prev_control = path_pool_[best_idx].control;
    for (auto& path : path_pool_) {
        path.control = best_prev_control;
    }

    distance_checker_.SetMap(map);

    // update our steering angle estimate using the last output
    model_.UpdateSteeringAngle(path_pool_[best_idx].path[0].steer);

    for (unsigned int t = 0; t < params.annealing_steps; t++) {
        auto& path = path_pool_[t];

        if (t > 0) {
            SampleControls(path.control, path_pool_[state_idx].control, t);
        }

        model_.RollOutPath(path.control, path.path);

        // Check collisions
        path.has_collision = false;
        for (size_t i = 0; i < path.path.size(); ++i) {
            double dist = distance_checker_.GetCollisionDistance(path.path[i].pose);
            path.has_collision |= (dist <= 0);
            path.dists[i] = dist;
        }

        path.cost = GetCost(path, map);
        double dcost = path.cost - path_pool_[state_idx].cost;

        if (dcost < 0) {
            state_idx = t;
        } else {
            double p_accept = std::exp(-params.acceptance_scale * dcost / GetTemperature(t));
            if (uniform_01_(rand_gen_) < p_accept) {
                state_idx = t;
            }
        }

        if (path.cost < path_pool_[best_idx].cost) {
            best_idx = t;
        }
    }

    last_path_idx_ = best_idx;

    return path_pool_[best_idx];
}

}  // namespace rr
