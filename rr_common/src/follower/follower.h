#ifndef RR_COMMON_FOLLOWER_H
#define RR_COMMON_FOLLOWER_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>
#include <std_msgs/Float64.h>
#include <string>
#include "flann/flann.hpp"
//#include <asdl.h>

float MIN_FRONT_VISION;
float MAX_FRONT_VISION;
float GOAL_DIST;
float GOAL_MARGIN_OF_ERR;
float MIN_SIDE_VISION;
float MAX_SIDE_VISION;
float FOLLOWER_SPEED;

std::string topic_from_plant;
std::string setpoint_topic;
std::string topic_from_controller;

ros::Publisher speed_pub, steer_pub, pid_speed_pub, pid_setpoint_pub;

#endif  // RR_COMMON_FOLLOWER_H
