//
// Created by Charles Jenkins on 11/27/20.
//

#pragma once

#include <vector>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <utility>

class tag_detection {
public:
    tag_detection(ros::NodeHandle *nh, std::string camera_frame, const std::string &pointcloud);

private:
    ros::Publisher pub_pointcloud;
    ros::Subscriber sub_detections;
    pcl::PointCloud<pcl::PointXYZ> opponent_cloud;
    std::string camera_frame;

    void draw_opponents(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

    void draw_opponent(const int &id, geometry_msgs::Pose_<std::allocator<void>> april_tag_center);

    void publishPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);

    void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
};