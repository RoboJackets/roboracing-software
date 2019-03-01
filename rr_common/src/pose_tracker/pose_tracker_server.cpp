#include <deque>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

#include <rr_platform/chassis_state.h>
#include <rr_platform/axes.h>

#include "pose_tracker/angle_utils.hpp"


// Types
using ChassisState = rr_platform::chassis_state;
using Orientation = rr_platform::axes;
using PoseHistoryMsg = nav_msgs::Path;

struct HistoryPoint {
  ros::Time time;
  double speed;
  double yaw;
};


// globals
double speed_;
double yaw_;


void chassis_state_callback(const ChassisState::ConstPtr& msg) {
  speed_ = msg->speed_mps;
}

void orientation_callback(const Orientation::ConstPtr& msg) {
  yaw_ = msg->yaw;
}


/**
 * Work from the current time and calculate where we must have been previously
 * @param next_pose Pose at the end of this time slice
 * @param h1 HistoryPoint (time, speed, yaw) at beginning of time slice
 * @param h2 HistoryPoint at end of time slice
 * @return Pose at the beginning of this time slice
 */
geometry_msgs::PoseStamped step_reverse(const geometry_msgs::PoseStamped& next_pose,
                                        const HistoryPoint& h1, const HistoryPoint& h2) {
  // set travel heading as average between start and end of time slice
  // note that this is relative to current heading
  double travel_heading_end = rr::poseStampedToYaw(next_pose);
  double heading_change = rr::heading_diff(h1.yaw, h2.yaw);
  double travel_heading_avg = rr::fix_angle(travel_heading_end - (heading_change / 2));
  double travel_heading_start = rr::fix_angle(travel_heading_end - heading_change);

  // same averaging for speed
  double travel_speed = (h2.speed + h1.speed) / 2;
  double dist_traveled = travel_speed * (h2.time - h1.time).toSec();

  // pose at beginning of this time slice
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = h1.time;

  pose_stamped.pose.position = next_pose.pose.position;
  pose_stamped.pose.position.x -= dist_traveled * std::cos(travel_heading_avg);
  pose_stamped.pose.position.y -= dist_traveled * std::sin(travel_heading_avg);
  pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(travel_heading_start);

  return pose_stamped;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_tracker_server");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // get params
  bool all_defined = true;

  std::string chassis_state_topic;
  all_defined &= nh_private.getParam("chassis_state_topic", chassis_state_topic);

  std::string angles_topic;
  all_defined &= nh_private.getParam("angles_topic", angles_topic);

  double time_horizon_temp;
  all_defined &= nh_private.getParam("time_horizon", time_horizon_temp);
  ros::Duration time_horizon(time_horizon_temp);

  double update_hz;
  all_defined &= nh_private.getParam("update_hz", update_hz);

  int publish_resolution;
  all_defined &= nh_private.getParam("publish_resolution", publish_resolution);

  if (!all_defined) {
    ROS_WARN("[pose_tracker] not all roslaunch params defined");
  }

  // subscribers
  auto sub1 = nh.subscribe<ChassisState>(chassis_state_topic, 1, chassis_state_callback);
  auto sub2 = nh.subscribe<Orientation>(angles_topic, 1, orientation_callback);

  auto history_publisher = nh.advertise<PoseHistoryMsg>("/pose_history", 10);

  // main data structure. New data goes in the front
  std::deque<HistoryPoint> history;

  speed_ = 0;
  yaw_ = 0;

  ros::Rate rate(update_hz);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();

    auto now = ros::Time::now();

    // remove any history which is too old
    while (!history.empty() && history.back().time + time_horizon < now) {
      history.pop_back();
    }

    // record current state
    auto& history_front = history.emplace_front();
    history_front.time = now;
    history_front.speed = speed_;
    history_front.yaw = yaw_;

    // estimate previous path
    if (history.size() >= 2) {
      PoseHistoryMsg pose_history_msg;
      pose_history_msg.header.stamp = now;
      pose_history_msg.header.frame_id = "base_footprint";

      auto& origin = pose_history_msg.poses.emplace_back().pose;  // default constructor puts this at the origin
      origin.orientation = tf::createQuaternionMsgFromYaw(0);

      // the notation here is slightly confusing but we are moving from "next" to "current"
      int counter = 1;
      auto hist_it_next = history.begin();
      auto hist_it_current = history.begin() + 1;
      geometry_msgs::PoseStamped pose_next = pose_history_msg.poses.front();

      while (hist_it_current != history.end()) {
        // step path backwards in time
        auto pose_current = step_reverse(pose_next, *hist_it_current, *hist_it_next);

        // record in output
        if (counter % publish_resolution == 0) {
          pose_history_msg.poses.push_back(pose_current);
        }

        pose_next = pose_current;
        hist_it_current++;
        hist_it_next++;
        counter++;
      }

      history_publisher.publish(pose_history_msg);
    }
  }
}
