#ifndef RR_COMMON_FOLLOWER_H
#define RR_COMMON_FOLLOWER_H

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rr_msgs/msg/speed.hpp>
#include <rr_msgs/msg/steering.hpp>
#include <std_msgs/msg/float64.hpp>

#include <string>

#include "flann/flann.hpp"
float MIN_FRONT_VISION;
float MAX_FRONT_VISION;
float GOAL_DIST;
float GOAL_MARGIN_OF_ERR;
float MIN_SIDE_VISION;
float MAX_SIDE_VISION;
float FOLLOWER_SPEED;

std::string topic_from_plant;
std::string setpoint_topic;

rclcpp::Publisher<rr_msgs::msg::Speed> speed_pub;
rclcpp::Publisher<rr_msgs::msg::Steering> steer_pub; 
pid_speed_pub, pid_setpoint_pub, visonBox_pub;
std::string topic_from_controller;

#endif  // RR_COMMON_FOLLOWER_H
