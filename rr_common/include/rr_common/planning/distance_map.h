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
    std::vector<double> DistanceCost(const std::vector<Pose>& poses) override;
    std::vector<double> DistanceCost(const std::vector<PathPoint>& path_points) override;


  private:
    std::pair<unsigned int, unsigned int> PoseToGridPosition(const rr::Pose& pose);
    void SetMapMessage(const nav_msgs::OccupancyGridConstPtr& map_msg);

    ros::Subscriber map_sub;
    ros::Publisher distance_map_pub;
    cv::Mat distance_cost_map;
    Rectangle hit_box;
    double cost_scaling_factor;
    double wall_inflation;
    int wall_cost;
    bool publish_distance_map;
    nav_msgs::MapMetaData mapMetaData;
    std::unique_ptr<tf::TransformListener> listener;
    tf::StampedTransform transform;

};

}  // namespace rr
