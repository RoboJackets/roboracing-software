#pragma once

#include "planner_types.hpp"

namespace rr {

class MapCostInterface {
  public:
    /**
     * Get the cost w.r.t. the map of a single pose
     * @param pose (x, y, theta) relative to the current pose of the robot
     * @return distance cost if not in collision, -1 if in collision
     */
    virtual double DistanceCost(const Pose& pose) = 0;

    /**
     * Get the cost w.r.t. the map of a sequence of poses
     * @param poses (x, y, theta) relative to the current pose of the robot
     * @return sum of distance costs if not in collision, -1 if in collision
     */
    virtual double DistanceCost(const std::vector<Pose>& poses) = 0;
    virtual double DistanceCost(const std::vector<PathPoint>& path_points) = 0;
};

};  // namespace rr
