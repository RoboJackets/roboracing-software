#pragma once

#include "planner_types.hpp"

#include <rr_common/planning/map_cost_interface.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <parameter_assertions/assertions.h>
#include <tf/transform_listener.h>

namespace rr {

class InflationCost : public MapCostInterface<nav_msgs::OccupancyGrid> {
    public:
        InflationCost(const ros::NodeHandle& nh, const CenteredBox& box);
        double DistanceCost(const Pose& pose) override;

        std::vector<double> DistanceCost(const std::vector<Pose>& poses) override;
        std::vector<double> DistanceCost(const std::vector<PathPoint>& path_points) override;

        void SetMapMessage(const boost::shared_ptr<nav_msgs::OccupancyGrid const>& map_msg) override;


    private:
        boost::shared_ptr<nav_msgs::OccupancyGrid const> map;
        CenteredBox hit_box;
        std::unique_ptr<tf::TransformListener> listener;
        tf::StampedTransform transform;
};

}  // namespace rr
