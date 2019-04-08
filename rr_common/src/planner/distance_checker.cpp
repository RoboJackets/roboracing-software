#include "planner/distance_checker.h"

namespace rr {

DistanceChecker::DistanceChecker(const CenteredBox& box, const CenteredBox& map_size)
  : hitbox_(box), map_size_(map_size), cache_resolution_(10.0)
{
  cache_rows_front_ = static_cast<int>(map_size_.length_front * cache_resolution_);
  cache_rows_back_ = static_cast<int>(map_size_.length_back * cache_resolution_);
  cache_rows_ = cache_rows_front_ + cache_rows_back_;
  cache_cols_left_ = static_cast<int>(map_size_.width_left * cache_resolution_);
  cache_cols_right_ = static_cast<int>(map_size_.width_right * cache_resolution_);
  cache_cols_ = cache_cols_left_ + cache_cols_right_;

  auto cache_size = static_cast<size_t>(cache_rows_ * cache_cols_);
  cache_.resize(cache_size);
  cache_visited_.resize(cache_size);
  for (int i = 0; i < static_cast<int>(cache_size); i++) {
    cache_[i].location = GetPointFromIndex(i);
    cache_[i].might_hit_points.clear();
    cache_[i].nearest_point = nullptr;
    cache_visited_[i] = false;
  }

  double half_length = (hitbox_.length_front + hitbox_.length_back) / 2.;
  double half_width = (hitbox_.width_left + hitbox_.width_right) / 2.;
  hitbox_corner_dist_ = std::sqrt(half_length * half_length + half_width * half_width);
}


int DistanceChecker::GetCacheIndex(double x, double y) {
  int r = static_cast<int>(x * cache_resolution_) + cache_rows_back_;
  if (r < 0 || r >= cache_rows_) {
    return -1;
  }

  int c = static_cast<int>(y * cache_resolution_) + cache_cols_right_;
  if (c < 0 || c > cache_cols_) {
    return -1;
  }

  return r * cache_cols_ + c;
}


PCLPoint DistanceChecker::GetPointFromIndex(int i) {
  int r = i / cache_cols_;
  int c = i % cache_cols_;

  PCLPoint out;
  out.x = (r - cache_rows_back_ + 0.5) / cache_resolution_;
  out.y = (c - cache_cols_right_ + 0.5) / cache_resolution_;
  return out;
}


double DistanceChecker::Dist(const PCLPoint& p1, const PCLPoint& p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}


void DistanceChecker::SetMap(const pcl::PointCloud<PCLPoint>& pointcloud) {
  for (CacheEntry& v : cache_) {
    v.might_hit_points.clear();
    v.nearest_point = nullptr;
  }

  cache_updates_.clear();
  std::fill(cache_visited_.begin(), cache_visited_.end(), false);

  for (const PCLPoint& p : pointcloud.points) {
    int i = GetCacheIndex(p.x, p.y);
    if (i < 0) {
      continue;
    }

    cache_[i].might_hit_points.push_back(&p);
    cache_[i].nearest_point = &p;
    cache_visited_[i] = true;

    int r = i / cache_cols_;
    int c = i % cache_cols_;
    for (int dr = -1; dr <= 1; dr++) {
      for (int dc = -1; dc <= 1; dc++) {
        if (r + dr < 0 || r + dr >= cache_rows_  || c + dc < 0 || c + dc >= cache_cols_) {
          continue;
        }

        int j = i + dr * cache_cols_ + dc;
        if (!cache_visited_[j]) {
          cache_updates_.emplace_back(j);
          cache_[j].parent = cache_.data() + i;
          cache_visited_[j] = true;
        }
      }
    }
  }

  while (!cache_updates_.empty()) {
    int j = cache_updates_.front();
    cache_updates_.pop_front();

    CacheEntry& entry = cache_[j];
    const CacheEntry& parent = *entry.parent;

    size_t size_start = entry.might_hit_points.size();
    const PCLPoint* nearest_point_start = entry.nearest_point;

    if (entry.nearest_point == nullptr) {
      entry.nearest_point = parent.nearest_point;
    }

    for (const PCLPoint* p : parent.might_hit_points) {
      double d = Dist(*p, entry.location);
      if (d < hitbox_corner_dist_ * 2) {
        entry.might_hit_points.push_back(p);
      }
    }

    if (entry.might_hit_points.size() != size_start || entry.nearest_point != nearest_point_start) {
      int r = j / cache_cols_;
      int c = j % cache_cols_;
      for (int dr = -1; dr <= 1; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
          if (r + dr < 0 || r + dr >= cache_rows_  || c + dc < 0 || c + dc >= cache_cols_) {
            continue;
          }

          int k = j + dr * cache_cols_ + dc;
          if (!cache_visited_[k]) {
            cache_updates_.emplace_back(k);
            cache_[k].parent = &entry;
            cache_visited_[k] = true;
          }
        }
      }
    }
  }
}

std::tuple<bool, double> DistanceChecker::GetCollisionDistance(const Pose& pose) {
  double cos_th = std::cos(pose.theta);
  double sin_th = std::sin(pose.theta);

  double center_x = (hitbox_.length_front - hitbox_.length_back) / 2.;
  double center_y = (hitbox_.width_left - hitbox_.width_right) / 2.;
  double search_x = pose.x + center_x * cos_th - center_y * sin_th;
  double search_y = pose.y + center_x * sin_th + center_y * cos_th;

  double half_length = (hitbox_.length_front + hitbox_.length_back) / 2.;
  double half_width = (hitbox_.width_left + hitbox_.width_right) / 2.;

  int i = GetCacheIndex(pose.x, pose.y);

  bool collision = false;
  double min_dist = 1000;

  if (i >= 0) {
    CacheEntry& entry = cache_[i];

    if (entry.nearest_point != nullptr) {
      entry.might_hit_points.push_back(entry.nearest_point);
    }

    for (const PCLPoint *p_ptr : entry.might_hit_points) {
      const PCLPoint &point = *p_ptr;

      // note that this is inverse kinematics here. We have origin -> robot but
      // want robot -> origin
      double offsetX = point.x - search_x;
      double offsetY = point.y - search_y;
      double x = cos_th * offsetX + sin_th * offsetY;
      double y = -sin_th * offsetX + cos_th * offsetY;

      // collisions
      if (std::abs(x) <= half_length && std::abs(y) <= half_width) {
        collision = true;
        min_dist = 0;
        break;
      }

      // find distance, in several cases
      double dist;
      if (std::abs(x) > half_length) {
        // not alongside the robot
        if (std::abs(y) > half_width) {
          // closest to a corner
          double cornerX = half_length * ((x < 0) ? -1 : 1);
          double cornerY = half_width * ((y < 0) ? -1 : 1);
          double dx = x - cornerX;
          double dy = y - cornerY;
          dist = std::sqrt(dx * dx + dy * dy);
        } else {
          // directly in front of or behind robot
          dist = std::abs(x) - half_length;
        }
      } else {
        // directly to the side of the robot
        dist = std::abs(y) - half_width;
      }
      min_dist = std::min(min_dist, dist);
    }

    if (entry.nearest_point != nullptr) {
      entry.might_hit_points.pop_back();
    }
  }

  return std::make_tuple(collision, min_dist);
}

bool DistanceChecker::GetCollision(const PCLPoint& relative_point) {
  return (relative_point.x < hitbox_.length_front)
      && (relative_point.x > -hitbox_.length_back)
      && (relative_point.y < hitbox_.width_left)
      && (relative_point.y > -hitbox_.width_right);
}

}  // namespace rr
