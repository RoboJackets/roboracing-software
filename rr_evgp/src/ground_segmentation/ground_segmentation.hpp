//
// Created by Charles Jenkins on 11/27/20.
//

#pragma once

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <utility>

#include "geometry_msgs/PoseArray.h"

#include "ground_segmenter/ground_segmenter.hpp"

class ground_segmentation {
  public:
    ground_segmentation(ros::NodeHandle *nh, GroundSegmenterParams params);

  private:
    ros::Publisher pcl_ground_pub_;
    ros::Publisher pcl_obstacle_pub_;
    ros::Subscriber velodyne_sub_;  // Used to keep subscriber alive

    tf::TransformListener tf_listener;

    pcl::PointCloud<pcl::PointXYZ> pcl_ground;
    pcl::PointCloud<pcl::PointXYZ> pcl_obstacle;

    GroundSegmenterParams segmentation_params;

    void publishPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, ros::Publisher &pub, std::string frame_id);
    constexpr static const double pose_distance = 0.1;

    void callback(const sensor_msgs::PointCloud2 &cloud);

    static tf::Pose poseAverage(std::vector<tf::Pose> poses);

    static tf::Quaternion getAverageQuaternion(std::vector<tf::Quaternion> &quaternions, std::vector<double> &weights);

    double robot_width = .25;
    double robot_depth_forward = .15;
    double robot_depth_backward = .5;
};
