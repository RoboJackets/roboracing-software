#include "annealing_planner.h"

namespace rr {

AnnealingPlanner::AnnealingPlanner(const DistanceChecker& c, const BicycleModel& m, const Params& p)
  : distance_checker_(c), model_(m), params(p)
{
  for (int i = 0; i < params.annealing_steps; i++) {
    rr::PlannedPath& path = path_pool_.emplace_back();
    path.control = std::vector<double>(params.n_path_segments, 0);
    model_.RollOutPath(path.path, path.control);
    path.dists = std::vector<double>(path.path.size(), 0.0);
    path.cost = std::numeric_limits<double>::max();
  }
  last_path_idx_ = 0;
  max_path_length_ = path_pool_[0].path.back().pose.x;
}

void AnnealingPlanner::SampleControls(std::vector<double>& ctrl, const std::vector<double>& source,
                                      unsigned int t) {
  auto it_ctrl = ctrl.begin();
  auto it_source = source.begin();
  for (; it_ctrl != ctrl.end(); ++it_ctrl, ++it_source) {
    double stddev = GetTemperature(t);
    *it_ctrl = *it_source + steering_gaussian_(rand_gen_) * stddev;
    if (std::abs(*it_ctrl) > params.max_steering) {
      *it_ctrl = std::copysign(params.max_steering, *it_ctrl);
    }
  }
}

double AnnealingPlanner::GetTemperature(unsigned int t) {
//  double progress = static_cast<double>(t) / params.annealing_steps;
//  return params.temperature_end * progress + params.temperature_start * (1.0 - progress);
  static const double K = std::log(params.temperature_end / params.temperature_start)
                          / params.annealing_steps;

  return params.temperature_start * std::exp(K * t);
}

double AnnealingPlanner::GetCost(const PlannedPath& planned_path, const KdTreeMap& kd_tree_map) {
  double cost = 0;

  const std::vector<PathPoint>& path = planned_path.path;
  const auto& last_path = path_pool_[last_path_idx_].path;

  for (auto i = 0; i < path.size(); i++) {
    double dist = planned_path.dists[i];
    bool collision_here = (dist <= 0);

    if (collision_here) {
      cost += params.collision_penalty * (path.size() - i);
      break;
    } else {
//      double dx = path[i].pose.x - last_path[i].pose.x;
//      double dy = path[i].pose.y - last_path[i].pose.y;
//      double deviation = dx*dx + dy*dy;
//      cost += params.k_similarity * deviation;
      cost -= params.k_dist * std::log(std::max(0.01, dist));
      cost -= params.k_speed * path[i].speed;
      cost += std::abs(path[i].pose.theta) * params.backwards_penalty;
    }
  }

//  double final_d = std::sqrt(std::pow(path.back().pose.x, 2) + std::pow(path.back().pose.y, 2));
  double dd = max_path_length_ - path.back().pose.x;
  cost += params.k_final_pose * dd * dd;

  return cost;
}

PlannedPath AnnealingPlanner::Plan(const KdTreeMap& kd_tree_map) {
  path_pool_[0] = path_pool_[last_path_idx_];

  unsigned int best_idx = 0;
  unsigned int state_idx = 0;
  bool collision = false;
  double dist = 0;

  auto best_prev_control = path_pool_[best_idx].control;  // copy
  std::for_each(path_pool_.begin(), path_pool_.end(), [&best_prev_control](rr::PlannedPath& path) {
    path.control = best_prev_control;
  });

  distance_checker_.SetMap(*kd_tree_map.input_);

  bool all_colliding = true;
  for (unsigned int t = 0; t < params.annealing_steps; t++) {
    auto& path = path_pool_[t];

    if (t > 0) {
      SampleControls(path.control, path_pool_[state_idx].control, t);
    }

    model_.RollOutPath(path.path, path.control);

    // Check collisions and adjust path speeds based on distance to obstacles
    bool has_been_too_close = false;
    bool has_collided = false;
    for (int i = static_cast<int>(path.path.size()) - 1; i >= 0; i--) {
      std::tie(collision, dist) = distance_checker_.GetCollisionDistance(path.path[i].pose);
      has_collided |= collision;
      path.dists[i] = dist;

      has_been_too_close |= (collision || dist < 0.5);
      if (has_been_too_close) {
        path.path[i].speed *= 0.7;
      }
    }

    all_colliding &= has_collided;

    double cost = GetCost(path, kd_tree_map);
    path.cost = cost;
    double dcost = cost - path_pool_[state_idx].cost;
    double p_accept = std::exp(-params.acceptance_scale * dcost / GetTemperature(t));

    if (uniform_01_(rand_gen_) < p_accept) {
      state_idx = t;
//      std::cout << "new state cost " << path.cost << ", t = " << t
//      << ", p = " << p_accept << ", T = " << GetTemperature(t) << std::endl;
    }

    if (path.cost < path_pool_[best_idx].cost) {
      best_idx = t;
//      std::cout << "new best cost " << path.cost << std::endl;
    }
  }

  if (all_colliding) {
    std::cout << "[Planner] no valid paths found" << std::endl;
    rr::PlannedPath& best = path_pool_[best_idx];
    std::fill(best.control.begin(), best.control.end(), 0);
    model_.RollOutPath(best.path, best.control);
    std::for_each(best.path.begin(), best.path.end(), [](PathPoint& p) { p.speed = -0.5; });
    best.all_collide = 1;
  }

  last_path_idx_ = best_idx;

  return path_pool_[best_idx];
}

}  // namespace rr
