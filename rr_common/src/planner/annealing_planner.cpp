#include "annealing_planner.h"

namespace rr {

AnnealingPlanner::AnnealingPlanner(const DistanceChecker& c, const BicycleModel& m, const Params& p)
  : distance_checker_(c), model_(m), params(p) {
  last_path_ = std::make_shared<rr::PlannedPath>();
  last_path_->control = {0, 0};
  last_path_->path = model_.RollOutPath(last_path_->control);
}

std::vector<double> AnnealingPlanner::SampleControls(const std::vector<double>& last, unsigned int t) {
  std::vector<double> out = last;  // copy
  double progress = static_cast<double>(t) / params.annealing_steps;

  for (int i = 0; i < out.size(); i++) {
    double stddev = params.temperature_end[i] * progress + params.temperature_start[i] * (1.0 - progress);
    out[i] += steering_gaussian_(rand_gen_) * stddev;
    if (std::abs(out[i]) > params.max_steering) {
      out[i] = std::copysign(params.max_steering, out[i]);
    }
  }

  return out;
}

std::tuple<bool, double> AnnealingPlanner::GetCost(const std::vector<PathPoint>& path, const KdTreeMap& kd_tree_map) {
  double cost = 0;
  bool collision = false;

  auto last_it = last_path_->path.begin();
  for (auto it = path.begin(); it != path.end(); ++it, ++last_it) {
    auto [collision_here, dist] = distance_checker_.GetCollisionDistance(it->pose, kd_tree_map);

    if (collision_here) {
      collision = true;
      cost += params.collision_penalty * (path.end() - it);
      break;
    } else {
      double dx = it->pose.x - last_it->pose.x;
      double dy = it->pose.y - last_it->pose.y;
      double similarity = std::sqrt(dx*dx + dy*dy);
      cost += params.k_similarity * similarity;
      cost -= (params.k_dist * std::log(1.0 + dist) + params.k_speed * it->speed);
    }
  }

  return std::make_tuple(collision, cost);
}

PlannedPath AnnealingPlanner::Plan(const KdTreeMap& kd_tree_map) {
  auto best_path = std::make_shared<PlannedPath>();
  best_path->control = std::vector<double>(params.n_path_segments, 0);
  best_path->path = model_.RollOutPath(best_path->control);
  best_path->cost = std::numeric_limits<double>::max();

  bool collision;
  for (unsigned int t = 0; t < params.annealing_steps; t++) {
    auto path = std::make_shared<PlannedPath>();
    path->control = SampleControls(best_path->control, t);
    path->path = model_.RollOutPath(path->control);
    std::tie(collision, path->cost) = GetCost(path->path, kd_tree_map);

    if (!collision && path->cost < best_path->cost) {
      best_path = path;
    }
  }

  last_path_ = best_path;
  return *best_path;
}

}  // namespace rr
