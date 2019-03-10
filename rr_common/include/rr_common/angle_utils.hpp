#pragma once

#include <cmath>
#include <tf/transform_datatypes.h>

namespace rr {

/**
 * If an angle is outside [0, 2*pi) range, rectify this
 * @param theta original angle
 * @return theta mapped onto [0, 2*pi)
 */
double fix_angle(double theta) {
  while (theta < 0) {
    theta += 2 * M_PI;
  }
  while (theta >= 2 * M_PI) {
    theta -= 2 * M_PI;
  }
  return theta;
}


/**
 * Find the smallest angle between two headings
 * @param theta1 heading at time t
 * @param theta2 heading at time t+1
 * @return smallest angle to get from theta1 to theta2
 */
double heading_diff(double theta1, double theta2) {
  theta1 = fix_angle(theta1);
  theta2 = fix_angle(theta2);
  double naive_diff = theta2 - theta1;

  // enumerate possible smallest diffs and select the lowest magnitude
  auto diff_options = {naive_diff, naive_diff - 2*M_PI, naive_diff + 2*M_PI};
  auto abs_compare = [](double x, double y) {
    return std::abs(x) < std::abs(y);
  };
  return std::min(diff_options, abs_compare);
}


/**
 * Get the yaw value from a pose
 * @param pose_stamped any geometry_msgs::PoseStamped
 * @return yaw Euler angle
 */
double poseStampedToYaw(const geometry_msgs::PoseStamped& pose_stamped) {
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose_stamped.pose.orientation, q);

  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace rr
