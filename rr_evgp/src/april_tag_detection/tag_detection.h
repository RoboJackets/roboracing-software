//
// Created by Charles Jenkins on 11/27/20.
//

#pragma once

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <utility>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

class tag_detection {
public:
    tag_detection(ros::NodeHandle *nh, std::string camera_frame, const std::string &pointcloud,
                  const std::string &tag_detections_topic, std::string destination_frame);

private:
    ros::Publisher pub_pointcloud;
    ros::Subscriber sub_detections;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    pcl::PointCloud<pcl::PointXYZ> opponent_cloud;
    std::string camera_frame;
    std::string destination_frame;

    void draw_opponents(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

    void draw_opponent(const int &id, geometry_msgs::Pose_<std::allocator<void>> april_tag_center);

    void publishPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);

    void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
};