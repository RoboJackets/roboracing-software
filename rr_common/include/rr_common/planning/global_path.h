#pragma once

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include "planner_types.hpp"

namespace rr {

class GlobalPath {
  public:
    explicit GlobalPath(ros::NodeHandle nh);

    std::vector<double> CalculateCost(const std::vector<PathPoint>& plan);
    double CalculateCost(const Pose& pose);
    void LookupPathTransform();
    void PreProcess();

  private:
    void SetPathMessage(const nav_msgs::Path& map_msg);
    unsigned int GetNextIndex(unsigned int i);
    static double GetPointDistance(tf::Pose pose1, tf::Pose pose2);

    bool accepting_updates_;
    bool updated_;

    unsigned int last_used_point_index_;
    double cost_scaling_factor_;

    ros::Subscriber global_path_sub_;
    std::string robot_base_frame_;
    double cost_scaling_factor;
    nav_msgs::Path global_path_msg_;
    std::unique_ptr<tf::TransformListener> listener_;
    tf::StampedTransform robot_to_path_transform_;
};

}  // namespace rr
