#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <parameter_assertions/assertions.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "map_cost_interface.h"
#include "planner_types.hpp"
#include "rectangle.hpp"

namespace rr {

class InflationMap : public MapCostInterface {
  public:
    explicit InflationMap(ros::NodeHandle nh);

    double DistanceCost(const Pose& pose) override;

    bool IsMapUpdated() override {
        return updated_ || (using_static_map && map_updated_ever);
    }

  private:
    void SetMapMessage(const nav_msgs::OccupancyGridConstPtr& map_msg);

    ros::Subscriber map_sub;
    ros::Timer pos_update_timer;
    nav_msgs::OccupancyGridConstPtr map;
    Rectangle hit_box;
    std::unique_ptr<tf::TransformListener> listener;
    tf::StampedTransform transform;
    int lethal_threshold;
    bool using_static_map;
    bool map_updated_ever;
};

}  // namespace rr
