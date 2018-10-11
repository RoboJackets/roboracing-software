#include "annealing_planner.h"

namespace rr {

AnnealingPlanner::AnnealingPlanner(const DistanceChecker& c, const BicycleModel& m, const Params& p)
  : distance_checker_(c), model_(m), params(p)
{
  for (int i = 0; i < params.annealing_steps; i++) {
    rr::PlannedPath& path = path_pool_.emplace_back();
    path.control = std::vector<double>(params.n_path_segments, 0);
    path.path = model_.RollOutPath(path.control);
    path.cost = std::numeric_limits<double>::max();
  }
  last_path_idx_ = 0;
}

std::vector<double> AnnealingPlanner::SampleControls(const std::vector<double>& last, unsigned int t) {
  std::vector<double> out = last;  // copy

  for (double& x : out) {
    double stddev = GetTemperature(t);
    x += steering_gaussian_(rand_gen_) * stddev;
    if (std::abs(x) > params.max_steering) {
      x = std::copysign(params.max_steering, x);
    }
  }

  return out;
}

double AnnealingPlanner::GetTemperature(unsigned int t) {
  double progress = static_cast<double>(t) / params.annealing_steps;
  return params.temperature_end * progress + params.temperature_start * (1.0 - progress);
}

std::tuple<bool, double> AnnealingPlanner::GetCost(const std::vector<PathPoint>& path, const KdTreeMap& kd_tree_map) {
  double cost = 0;
  bool collision = false;

  const auto& last_path = path_pool_[last_path_idx_].path;
  for (auto it = path.begin(), last_it = last_path.begin(); it != path.end(); ++it, ++last_it) {
    auto [collision_here, dist] = distance_checker_.GetCollisionDistance(it->pose, kd_tree_map);

    if (collision_here) {
      collision = true;
      cost += params.collision_penalty * (path.end() - it);
      break;
    } else {
      double dx = it->pose.x - last_it->pose.x;
      double dy = it->pose.y - last_it->pose.y;
      double deviation = dx*dx + dy*dy;
      cost += params.k_similarity * deviation;
      cost -= (params.k_dist * std::log(std::max(0.01, dist)) + params.k_speed * it->speed);
    }

    double final_x = path.back().pose.x;
    cost -= params.k_final_pose * std::log(std::max(1.0, final_x));
  }

  return std::make_tuple(collision, cost);
}

PlannedPath AnnealingPlanner::Plan(const KdTreeMap& kd_tree_map) {
  unsigned int best_idx = last_path_idx_;
  bool collision;

  std::tie(collision, path_pool_[best_idx].cost) = GetCost(path_pool_[best_idx].path, kd_tree_map);
  if (collision) {
    path_pool_[best_idx].cost = std::numeric_limits<double>::max();
  }

  auto best_prev_control = path_pool_[best_idx].control;  // copy
  std::for_each(path_pool_.begin(), path_pool_.end(), [&best_prev_control](rr::PlannedPath& path) {
    path.control = best_prev_control;
  });

  for (unsigned int t = 0; t < params.annealing_steps; t++) {
    auto& path = path_pool_[t];
    path.control = SampleControls(path.control, t);
    path.path = model_.RollOutPath(path.control);
    std::tie(collision, path.cost) = GetCost(path.path, kd_tree_map);

    double dcost = path.cost - path_pool_[best_idx].cost;
    double p_accept = std::exp(-params.acceptance_scale * dcost / GetTemperature(t));
//    std::cout << "p_accept = " << p_accept << std::endl;

    if (!collision && uniform_01_(rand_gen_) < p_accept) {
      best_idx = t;
    }
  }

  last_path_idx_ = best_idx;

  return path_pool_[best_idx];
}

}  // namespace rr
