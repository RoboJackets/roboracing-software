#pragma once

#include "planner_types.hpp"

namespace rr {

class Planner {
public:

  /*
   * Plan: given a map of the world, find the best path through it
   */
  virtual PlannedPath Plan(const PCLMap& map) = 0;

};

}  // namespace rr
