#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <parameter_assertions/assertions.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>
#include "map_cost_interface.h"
#include "planner_types.hpp"
#include "rectangle.hpp"

namespace rr {

class DistanceMap : public MapCostInterface {
  public:
    explicit DistanceMap(ros::NodeHandle nh);

    double DistanceCost(const Pose& pose) override;

  private:
    std::pair<unsigned int, unsigned int> PoseToGridPosition(const rr::Pose& pose);
    void SetMapMessage(const nav_msgs::OccupancyGridConstPtr& map_msg);

    ros::Subscriber map_sub;
    ros::Publisher distance_map_pub;
    ros::Publisher inscribed_circle_pub;
    cv::Mat distance_cost_map;
    Rectangle hit_box;
    double cost_scaling_factor;
    double wall_inflation;
    double inscribed_circle_radius;
    double inscribed_circle_origin;
    bool publish_distance_map;
    bool publish_inscribed_circle;
    nav_msgs::MapMetaData mapMetaData;
    std::unique_ptr<tf::TransformListener> listener;
    tf::StampedTransform transform;
};

}  // namespace rr
