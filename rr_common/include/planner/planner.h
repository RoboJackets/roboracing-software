/**
 * Planner.h: Planner interface for free-space planners on point cloud maps
 */

#pragma once

#include "planner_types.hpp"

namespace rr {

class Planner {
public:
  /**
   * Given a map of the world, find the best path through it
   * @param map Point cloud map
   * @return Object representing the planned trajectory
   */
  virtual PlannedPath Plan(const PCLMap& map) = 0;
};

}  // namespace rr
