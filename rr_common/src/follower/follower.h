#ifndef RR_COMMON_FOLLOWER_H
#define RR_COMMON_FOLLOWER_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <string>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>
#include "flann/flann.hpp"

float MIN_FRONT_VISION;
float MAX_FRONT_VISION;
float GOAL_DIST;
float GOAL_MARGIN_OF_ERR;
float MIN_SIDE_VISION;
float MAX_SIDE_VISION;
float FOLLOWER_SPEED;

ros::Publisher speed_pub, steer_pub;

#endif  // RR_COMMON_FOLLOWER_H
