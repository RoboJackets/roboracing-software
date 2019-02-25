#pragma once

#include <tuple>
#include <deque>

#include "planner_types.hpp"

namespace rr {

class DistanceChecker {
 public:
  DistanceChecker(const CenteredBox& box, const CenteredBox& map_size);

  /*
   * Calculate the shortest distance from any part of the robot to any point 
   * in the map. Pose is future relative to current/origin (0,0,0)
   * Returns:
   * bool - collision detected
   * double - clearing distance to nearest obstacle
   */
  std::tuple<bool, double> GetCollisionDistance(const Pose& pose);

  void SetMap(const pcl::PointCloud<PCLPoint>& pointcloud);

  bool GetCollision(const PCLPoint& relative_point);

  double Dist(const PCLPoint& p1, const PCLPoint& p2);

 private:
  struct CacheEntry {
    PCLPoint location;
    std::vector<const PCLPoint*> might_hit_points;
    const PCLPoint* nearest_point;
    const CacheEntry* parent;
  };

  const CenteredBox hitbox_;
  const CenteredBox map_size_;
  const double cache_resolution_;

  double hitbox_corner_dist_;

  std::vector<CacheEntry> cache_;
  std::deque<int> cache_updates_;
  std::vector<char> cache_visited_;
  int cache_rows_front_;
  int cache_rows_back_;
  int cache_rows_;
  int cache_cols_left_;
  int cache_cols_right_;
  int cache_cols_;

  int GetCacheIndex(double x, double y);
  PCLPoint GetPointFromIndex(int i);
};

}  // namespace rr
