#include <rr_common/planning/nearest_point_cache.h>

#include <functional>

#include <parameter_assertions/assertions.h>
#include <pcl_conversions/pcl_conversions.h>

namespace rr {

NearestPointCache::NearestPointCache(ros::NodeHandle& nh)
      : points_storage_()
      , map_limits_(ros::NodeHandle(nh, "map_limits"))
      , hitbox_(ros::NodeHandle(nh, "hitbox"))
      , updated_(false)
      , activated_(true) {
    assertions::getParam(nh, "cache_resolution", cache_resolution_, { assertions::greater(0.0) });

    cache_size_x_ = static_cast<int>((map_limits_.max_x - map_limits_.min_x) / cache_resolution_);
    cache_size_y_ = static_cast<int>((map_limits_.max_y - map_limits_.min_y) / cache_resolution_);

    size_t cache_size = cache_size_x_ * cache_size_y_;
    cache_.resize(cache_size);
    cache_visited_.resize(cache_size);
    for (int i = 0; i < static_cast<int>(cache_size); i++) {
        cache_[i].location = GetPointFromIndex(i);
        cache_[i].might_hit_points.clear();
        cache_[i].nearest_point = nullptr;
        cache_visited_[i] = false;
    }

    double half_x = (hitbox_.max_x - hitbox_.min_x) / 2.;
    double half_y = (hitbox_.max_y - hitbox_.min_y) / 2.;
    hitbox_corner_dist_ = std::sqrt(half_x * half_x + half_y * half_y);

    std::string obstacle_cloud_topic;
    assertions::getParam(nh, "input_cloud_topic", obstacle_cloud_topic);
    map_sub_ = nh.subscribe(obstacle_cloud_topic, 1, &NearestPointCache::SetMapMessage, this);

    assertions::getParam(nh, "distance_decay_factor", dist_decay_, { assertions::greater(0.0) });
}

inline double dist(const NearestPointCache::point_t& p1, const NearestPointCache::point_t& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

void NearestPointCache::SetMapMessage(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    std::unique_lock lock(mutex_);
    if (!activated_) {
        return;
    }

    points_storage_.clear();
    pcl::fromROSMsg(*cloud_msg, points_storage_);

    // remove points in collision with robot
    points_storage_.erase(std::remove_if(points_storage_.begin(), points_storage_.end(),
                                         [this](const auto& point) { return hitbox_.PointInside(point.x, point.y); }));

    if (points_storage_.empty()) {
        ROS_WARN("environment map pointcloud is empty");
    }

    for (CacheEntry& v : cache_) {
        v.might_hit_points.clear();
        v.nearest_point = nullptr;
    }

    std::deque<int> cache_updates_;
    cache_visited_ = false;

    for (const point_t& p : points_storage_) {
        int i = GetCacheIndex(p.x, p.y);
        if (i < 0) {
            continue;
        }

        cache_[i].might_hit_points.push_back(&p);
        cache_[i].nearest_point = &p;
        cache_visited_[i] = true;

        int my = i / cache_size_x_;
        int mx = i % cache_size_x_;
        for (int dmy = -1; dmy <= 1; dmy++) {
            for (int dmx = -1; dmx <= 1; dmx++) {
                if (my + dmy < 0 || my + dmy >= cache_size_y_ || mx + dmx < 0 || mx + dmx >= cache_size_x_) {
                    continue;
                }

                int j = (my + dmy) * cache_size_x_ + mx + dmx;
                if (!cache_visited_[j]) {
                    cache_updates_.push_back(j);
                    cache_[j].parent = &(cache_[i]);
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
        const point_t* nearest_point_start = entry.nearest_point;

        if (entry.nearest_point == nullptr) {
            entry.nearest_point = parent.nearest_point;
        }

        for (const point_t* p : parent.might_hit_points) {
            double d = dist(*p, entry.location);
            if (d <= hitbox_corner_dist_ * 2) {
                entry.might_hit_points.push_back(p);
            }
        }

        if (entry.might_hit_points.size() != size_start || entry.nearest_point != nearest_point_start) {
            int my = j / cache_size_x_;
            int mx = j % cache_size_x_;
            for (int dmy = -1; dmy <= 1; dmy++) {
                for (int dmx = -1; dmx <= 1; dmx++) {
                    if (my + dmy < 0 || my + dmy >= cache_size_y_ || mx + dmx < 0 || mx + dmx >= cache_size_x_) {
                        continue;
                    }

                    int k = j + (dmy * cache_size_x_) + dmx;
                    if (!cache_visited_[k]) {
                        cache_updates_.emplace_back(k);
                        cache_[k].parent = cache_.data() + j;
                        cache_visited_[k] = true;
                    }
                }
            }
        }
    }

    updated_ = true;
}

double NearestPointCache::_SingleDistanceCost(const rr::Pose& pose) {
    double cos_th = std::cos(pose.theta);
    double sin_th = std::sin(pose.theta);

    double center_x = (hitbox_.min_x + hitbox_.max_x) / 2.;
    double center_y = (hitbox_.min_y + hitbox_.max_y) / 2.;
    double search_x = pose.x + center_x * cos_th - center_y * sin_th;
    double search_y = pose.y + center_x * sin_th + center_y * cos_th;

    double half_x = (hitbox_.max_x - hitbox_.min_x) / 2.;
    double half_y = (hitbox_.max_y - hitbox_.min_y) / 2.;

    int i = GetCacheIndex(pose.x, pose.y);
    if (i < 0) {
        return -1.0;
    }

    const CacheEntry& entry = cache_[i];

    auto point_in_local_frame = [search_x, search_y, cos_th, sin_th](const point_t& p, double& x, double& y) {
        double offsetX = p.x - search_x;
        double offsetY = p.y - search_y;
        x = cos_th * offsetX + sin_th * offsetY;
        y = -sin_th * offsetX + cos_th * offsetY;
    };

    // collisions
    for (const point_t* p_ptr : entry.might_hit_points) {
        double x, y;
        point_in_local_frame(*p_ptr, x, y);
        if (std::abs(x) <= half_x && std::abs(y) <= half_y) {
            return -1.0;
        }
    }

    // find distance, in several cases
    double dist;
    if (nullptr == entry.nearest_point) {  // empty map (?)
        dist = std::pow(10.0, 10);
    } else {
        double x, y;
        point_in_local_frame(*entry.nearest_point, x, y);
        if (std::abs(x) > half_x) {
            // not alongside the robot
            if (std::abs(y) > half_y) {
                // closest to a corner
                double cornerX = half_x * ((x < 0) ? -1 : 1);
                double cornerY = half_y * ((y < 0) ? -1 : 1);
                double dx = x - cornerX;
                double dy = y - cornerY;
                dist = std::sqrt(dx * dx + dy * dy);
            } else {
                // directly in front of or behind robot
                dist = std::abs(x) - half_x;
            }
        } else {
            // directly to the side of the robot
            dist = std::abs(y) - half_y;
        }

        if (std::isnan(dist) || dist < 0 || dist > 1000) {
            std::cout << "index " << i << " nearest " << *entry.nearest_point << " dist " << dist << std::endl;
            std::cout << "x " << x << " y " << y << " halves " << half_x << " " << half_y << std::endl;
        }
    }

    return std::exp(-dist_decay_ * dist);
}

double NearestPointCache::DistanceCost(const Pose& pose) {
    return _SingleDistanceCost(pose);
}

std::vector<double> NearestPointCache::DistanceCost(const std::vector<Pose>& poses) {
    std::vector<double> out(poses.size());
    std::transform(poses.cbegin(), poses.cend(), out.begin(), [this](const Pose& p) { return _SingleDistanceCost(p); });
    return out;
}

std::vector<double> NearestPointCache::DistanceCost(const std::vector<PathPoint>& path) {
    std::vector<double> out(path.size());
    std::transform(path.cbegin(), path.cend(), out.begin(),
                   [this](const PathPoint& p) { return _SingleDistanceCost(p.pose); });
    return out;
}

}  // namespace rr
