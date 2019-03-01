#include "pose_tracker/RelativePoseHistoryClient.h"
#include "pose_tracker/angle_utils.hpp"

/**
 * Interpolate in time domain between two poses
 * @param p1 pose with smaller timestamp
 * @param p2 pose with larger timestamp
 * @param t target time
 * @return pose at time t
 */
RelativePoseHistoryClient::Pose linear_interp(const geometry_msgs::PoseStamped& p1,
                                              const geometry_msgs::PoseStamped& p2,
                                              const ros::Time& t) {
  RelativePoseHistoryClient::Pose out_pose;

  // lambda = similarity to p2
  double dt1 = (t - p1.header.stamp).toSec();
  double dt2 = (p2.header.stamp - p1.header.stamp).toSec();
  double lambda = dt1 / dt2;

  out_pose.x = p2.pose.position.x * lambda + p1.pose.position.x * (1 - lambda);
  out_pose.y = p2.pose.position.y * lambda + p1.pose.position.y * (1 - lambda);

  double theta1 = rr::poseStampedToYaw(p1);
  double theta2 = rr::poseStampedToYaw(p2);
  out_pose.theta = rr::fix_angle(theta1 + lambda * rr::heading_diff(theta1, theta2));

  return out_pose;
}

void RelativePoseHistoryClient::RegisterCallback(ros::NodeHandle& nh) {
  auto callback = [this](const nav_msgs::PathConstPtr& path_msg) {
    history_ = path_msg;
  };
  nh.subscribe<nav_msgs::Path>("/pose_history", 1, callback);
}

RelativePoseHistoryClient::Pose RelativePoseHistoryClient::GetRelativePoseAtTime(const ros::Time& t) {
  Pose out_pose;
  auto now = ros::Time::now();

  if (!history_ || history_->poses.empty()) {
    // case 1: no pose history. Return the current position
    out_pose.x = out_pose.y = out_pose.theta = 0;
    ROS_WARN("[RelativePoseHistoryClient] Requesting relative pose but no pose history available");
  }

  else {
    // case 2: we have pose history
    auto& poses = history_->poses;

    // find the index of the last pose before the requested time
    int i = 0;
    while (poses[i].header.stamp > t && i < poses.size()) {
      i++;
    }

    if (i == 0) {
      // case 2.1: requested time is newer than any in history
      if (t >= now) {
        // too new; use current pose
        out_pose.x = out_pose.y = out_pose.theta = 0;
      } else {
        // interpolate between newest point in history and now
        geometry_msgs::PoseStamped now_pose;  // x = y = z = r = p = y = 0
        now_pose.header.stamp = now;
        out_pose = linear_interp(poses[0], now_pose, t);
      }
    }

    else if (i >= poses.size()) {
      // case 2.2: requested time is older than any in history
      out_pose.x = poses.back().pose.position.x;
      out_pose.y = poses.back().pose.position.y;
      out_pose.theta = rr::poseStampedToYaw(poses.back());
    }

    else {
      // case 2.3: requested time is between two existing points
      out_pose = linear_interp(poses[i], poses[i-1], t);
    }
  }

  return out_pose;
}
