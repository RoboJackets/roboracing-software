#pragma once

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace rr {

class RelativePoseHistoryClient {
public:
  using Pose = geometry_msgs::Pose2D;

  RelativePoseHistoryClient() = default;

  /**
   * Given a desired (recent) past time, get the interpolated pose at that time
   * @param t past time
   * @return Pose (x, y, theta) in current local frame
   */
  Pose GetRelativePoseAtTime(const ros::Time& t);

  /**
   * Register callback for /pose_history topic
   * @param handle NodeHandle to use for subscription
   */
  ros::Subscriber RegisterCallback(ros::NodeHandle& handle);

private:
  void callback(const nav_msgs::PathConstPtr& path_msg);

  nav_msgs::PathConstPtr history_;
};

}  // namespace rr
