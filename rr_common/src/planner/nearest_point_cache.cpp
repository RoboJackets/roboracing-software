#include <rr_common/nearest_point_cache.h>

namespace rr {

NearestPointCache::NearestPointCache(const CenteredBox& box, const CenteredBox& map_size)
      : map_size_(map_size), cache_resolution_(10.0), hitbox_(box) {
    cache_rows_front_ = static_cast<int>(map_size_.length_front * cache_resolution_);
    cache_rows_back_ = static_cast<int>(map_size_.length_back * cache_resolution_);
    cache_rows_ = cache_rows_front_ + cache_rows_back_;
    cache_cols_left_ = static_cast<int>(map_size_.width_left * cache_resolution_);
    cache_cols_right_ = static_cast<int>(map_size_.width_right * cache_resolution_);
    cache_cols_ = cache_cols_left_ + cache_cols_right_;

    size_t cache_size = cache_rows_ * cache_cols_;
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

int NearestPointCache::GetCacheIndex(double x, double y) const {
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

PCLPoint NearestPointCache::GetPointFromIndex(int i) const {
    int r = i / cache_cols_;
    int c = i % cache_cols_;

    PCLPoint out;
    out.x = (r - cache_rows_back_ + 0.5) / cache_resolution_;
    out.y = (c - cache_cols_right_ + 0.5) / cache_resolution_;
    return out;
}

double NearestPointCache::Dist(const PCLPoint& p1, const PCLPoint& p2) const {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

void NearestPointCache::SetMap(const pcl::PointCloud<PCLPoint>& pointcloud) {
    for (CacheEntry& v : cache_) {
        v.might_hit_points.clear();
        v.nearest_point = nullptr;
    }

    cache_updates_.clear();
    cache_visited_ = false;

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
                if (r + dr < 0 || r + dr >= cache_rows_ || c + dc < 0 || c + dc >= cache_cols_) {
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
            if (d <= hitbox_corner_dist_) {
                entry.might_hit_points.push_back(p);
            }
        }

        if (entry.might_hit_points.size() != size_start || entry.nearest_point != nearest_point_start) {
            int r = j / cache_cols_;
            int c = j % cache_cols_;
            for (int dr = -1; dr <= 1; dr++) {
                for (int dc = -1; dc <= 1; dc++) {
                    if (r + dr < 0 || r + dr >= cache_rows_ || c + dc < 0 || c + dc >= cache_cols_) {
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

double NearestPointCache::GetCollisionDistance(const Pose& pose) const {
    double cos_th = std::cos(pose.theta);
    double sin_th = std::sin(pose.theta);

    double center_x = (hitbox_.length_front - hitbox_.length_back) / 2.;
    double center_y = (hitbox_.width_left - hitbox_.width_right) / 2.;
    double search_x = pose.x + center_x * cos_th - center_y * sin_th;
    double search_y = pose.y + center_x * sin_th + center_y * cos_th;

    double half_length = (hitbox_.length_front + hitbox_.length_back) / 2.;
    double half_width = (hitbox_.width_left + hitbox_.width_right) / 2.;

    int i = GetCacheIndex(pose.x, pose.y);
    if (i < 0) {
        return -1.0;
    }

    const CacheEntry& entry = cache_[i];

    auto point_in_local_frame = [search_x, search_y, cos_th, sin_th](const PCLPoint& p) {
        double offsetX = p.x - search_x;
        double offsetY = p.y - search_y;
        double x = cos_th * offsetX + sin_th * offsetY;
        double y = -sin_th * offsetX + cos_th * offsetY;
        return std::make_tuple(x, y);
    };

    // collisions
    for (const PCLPoint* p_ptr : entry.might_hit_points) {
        auto [x, y] = point_in_local_frame(*p_ptr);
        if (std::abs(x) <= half_length && std::abs(y) <= half_width) {
            return -1.0;
        }
    }

    // find distance, in several cases
    double dist;
    if (nullptr == entry.nearest_point) {  // empty map (?)
        dist = std::pow(10.0, 10);
    } else {
        auto [x, y] = point_in_local_frame(*entry.nearest_point);
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
    }

    return dist;
}

bool NearestPointCache::GetCollision(const PCLPoint& relative_point) const {
    return (relative_point.x < hitbox_.length_front) && (relative_point.x > -hitbox_.length_back) &&
           (relative_point.y < hitbox_.width_left) && (relative_point.y > -hitbox_.width_right);
}

}  // namespace rr
