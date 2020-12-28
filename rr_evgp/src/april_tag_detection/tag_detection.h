//
// Created by Charles Jenkins on 11/27/20.
//

#pragma once

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>

#include <utility>

class tag_detection {
public:
    tag_detection(ros::NodeHandle *nh, std::string camera_frame, const std::string &pointcloud,
                  const std::string &tag_detections_topic, std::string destination_frame, double x_offset,
                  double y_offset, double px_per_m, double width, double height);

private:
    ros::Publisher pub_pointcloud;
    ros::Publisher pub_markers;
    ros::Subscriber sub_detections;
    tf::StampedTransform camera_w;
    pcl::PointCloud<pcl::PointXYZ> pcl_outline;

    tf::TransformListener tf_listener;
    std::vector<std::array<int, 3>> colors{{255, 0,   0},
                                           {255, 127, 0},
                                           {0,   255, 215},
                                           {126, 0,   255},
                                           {0,   0,   255}};

    pcl::PointCloud<pcl::PointXYZ> opponent_cloud;
    std::string camera_frame, destination_frame;
    double x_offset, y_offset, px_per_m, width, height;

    void draw_opponent(int id, geometry_msgs::Pose april_camera);

    void publishPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);

    void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

    void create_marker(const int &id, geometry_msgs::Pose april_w);
};