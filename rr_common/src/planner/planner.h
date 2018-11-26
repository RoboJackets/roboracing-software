#ifndef RR_COMMON_PLANNER_H
#define RR_COMMON_PLANNER_H

#include "planner_types.hpp"

namespace rr {

class Planner {
public:

  /*
   * Plan: given a map of the world, find the best path through it
   */
  virtual PlannedPath Plan(const KdTreeMap& kd_tree_map) = 0;

};

}  // namespace rr

#endif  // RR_COMMON_RANDOM_SAMPLE_PLANNER_H
