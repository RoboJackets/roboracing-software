/*
 * MapCostInterface:
 * - "owns" subscription to map data
 * - Given a pose or sequence of poses, returns the cost(s) w.r.t the map
 * - Reports whether new map data is available
 */

#pragma once

#include "planner_types.hpp"

namespace rr {

class MapCostInterface {
  public:
    MapCostInterface() : updated_(false), accepting_updates_(true) {}

    /**
     * Get the cost w.r.t. the map of a single pose
     * @param pose (x, y, theta) relative to the current pose of the robot
     * @return distance cost if not in collision, negative value if in collision
     */
    virtual double DistanceCost(const Pose& pose) = 0;

    /**
     * Get the cost w.r.t. the map of a sequence of poses
     * @param poses (x, y, theta) relative to the current pose of the robot
     * @return for each entry, distance cost if not in collision, negative value if in collision
     */
    virtual std::vector<double> DistanceCost(const std::vector<Pose>& path) {
        std::vector<double> costs(path.size());
        std::transform(path.cbegin(), path.cend(), costs.begin(), [this](const Pose& p) { return DistanceCost(p); });
        return costs;
    }

    virtual std::vector<double> DistanceCost(const std::vector<PathPoint>& path) {
        std::vector<double> costs(path.size());
        std::transform(path.cbegin(), path.cend(), costs.begin(),
                       [this](const PathPoint& p) { return DistanceCost(p.pose); });
        return costs;
    }

    virtual bool IsMapUpdated() {
        return updated_;
    }
    virtual void SetMapStale() {
        updated_ = false;
    }

    virtual void StartUpdates() {
        accepting_updates_ = true;
    }
    virtual void StopUpdates() {
        accepting_updates_ = false;
    }

  protected:
    bool updated_;            // set true when stored map data is updated
    bool accepting_updates_;  // only modify stored map data when this is true
};

}  // namespace rr
