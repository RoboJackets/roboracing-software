#include <rr_common/planning/cost_heuristic.h>

namespace rr {

CostHeuristic::CostHeuristic(ros::NodeHandle nh) {
    assertions::getParam(nh, "k_map_cost", k_map_cost_);
    assertions::getParam(nh, "k_speed", k_speed_);
    assertions::getParam(nh, "k_steering", k_steering_);
    assertions::getParam(nh, "k_angle", k_angle_);
    assertions::getParam(nh, "collision_penalty", collision_penalty_);
}

double CostHeuristic::getMapCost(const rr::TrajectoryRollout& rollout, std::vector<double> map_costs) {
    const auto& path = rollout.path;
    double map_cost = 0;
    for (size_t i = 0; i < path.size(); ++i) {
        if (map_costs[i] >= 0) {
            map_cost += k_map_cost_ * map_costs[i];
        } else {
            map_cost += collision_penalty_ * (path.size() - i);
            break;
        }
    }
    return map_cost;
}

double CostHeuristic::getSpeedCost(const rr::TrajectoryRollout& rollout, double max_speed) {
    const auto& path = rollout.path;
    double speed_cost = 0;
    for (size_t i = 0; i < path.size(); ++i) {
        speed_cost += k_speed_ * std::pow(max_speed - path[i].speed, 2);
    }
    return speed_cost;
}

double CostHeuristic::getSteeringCost(const rr::TrajectoryRollout& rollout) {
    const auto& path = rollout.path;
    double steering_cost = 0;
    for (size_t i = 0; i < path.size(); ++i) {
        steering_cost += k_steering_ * std::abs(path[i].steer);
    }
    return steering_cost;
}

double CostHeuristic::getAngleCost(const rr::TrajectoryRollout& rollout) {
    const auto& path = rollout.path;
    double angle_cost = 0;
    for (size_t i = 0; i < path.size(); ++i) {
        angle_cost += k_angle_ * std::abs(path[i].pose.theta);
    }
    return angle_cost;
}

}  // namespace rr