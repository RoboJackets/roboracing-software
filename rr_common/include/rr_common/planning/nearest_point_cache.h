/**
 * DistanceChecker: implementation of the MapCostInterface which does careful distance and collision checking.
 * This is suitable for paths which pass close to obstacles, such as those used in the IARRC Obstacle Avoidance
 * Challenge.
 */

#pragma once

#include <deque>
#include <mutex>
#include <tuple>
#include <valarray>

#include <sensor_msgs/PointCloud2.h>

#include "map_cost_interface.h"
#include "planner_types.hpp"
#include "rectangle.hpp"

namespace rr {

class NearestPointCache : public MapCostInterface {
  public:
    using point_t = pcl::PointXYZ;

    /**
     * Constructor
     * @param hitbox Hitbox of the robot
     * @param map_size Map size limits for cached distances. Set this so that it is not possible for paths to leave this
     * box.
     */
    explicit NearestPointCache(ros::NodeHandle nh);

    double DistanceCost(const Pose& pose) override;

    std::vector<double> DistanceCost(const std::vector<Pose>& poses) override;

    std::vector<double> DistanceCost(const std::vector<PathPoint>& path) override;

  private:
    /**
     * Given a map, cache the nearest neighbors. Fill the cache outwards from locations containing obstacle points.
     * @param map Point cloud map representation
     */
    void SetMapMessage(const sensor_msgs::PointCloud2ConstPtr& cloud);

    /*
     * Caching system: map from discretized x, y location to its nearest neighbor in the map/obstacle point cloud
     */
    struct CacheEntry {
        point_t location;                              // x, y location represented by this entry
        std::vector<const point_t*> might_hit_points;  // map points which are close enough to check
                                                       // collisions
        const point_t* nearest_point;                  // nearest neighbor in obstacle map
        const CacheEntry* parent;                      // BFS parent
    };

    /*
     * Get the index of a cache element from the x, y location and vice versa
     */
    [[nodiscard]] inline int GetCacheIndex(double x, double y) const {
        if (!map_limits_.PointInside(x, y)) {
            return -1;
        }

        double dx = x - map_limits_.min_x;
        double dy = y - map_limits_.min_y;

        auto mx = static_cast<int>(dx / cache_resolution_);
        auto my = static_cast<int>(dy / cache_resolution_);

        return my * cache_size_x_ + mx;
    }

    [[nodiscard]] inline point_t GetPointFromIndex(int i) const {
        int my = i / cache_size_x_;
        int mx = i % cache_size_x_;

        point_t out;
        out.x = map_limits_.min_x + ((mx + 0.5) * cache_resolution_);
        out.y = map_limits_.min_y + ((my + 0.5) * cache_resolution_);
        return out;
    }

    pcl::PointCloud<point_t> points_storage_;
    std::vector<CacheEntry> cache_;      // cache storage
    std::valarray<bool> cache_visited_;  // boolean mask for tracking which locations have been updated
    int cache_size_x_;
    int cache_size_y_;
    double cache_resolution_;
    rr::Rectangle map_limits_;
    double dist_decay_;  // map cost is exp(-dist_decay_ * dist). Smaller value is like a larger inflation radius

    rr::Rectangle hitbox_;
    double hitbox_corner_dist_;

    ros::Subscriber map_sub_;
    std::mutex mutex_;
};

}  // namespace rr
