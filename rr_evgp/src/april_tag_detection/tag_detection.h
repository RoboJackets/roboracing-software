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

#include <utility>

class tag_detection {
public:
    tag_detection(ros::NodeHandle *nh, const std::string &camera_frame, const std::string &pointcloud,
                  const std::string &tag_detections_topic, const std::string &destination_frame,
                  const std::string &tag_detection_markers, double x_offset,
                  double y_offset, double px_per_m, double width, double height);

private:
    ros::Publisher pub_pointcloud;
    ros::Publisher pub_markers;
    ros::Subscriber sub_detections; //Used to keep subscriber alive
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
    double width;

    void publishPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);

    void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

    void draw_opponents(std::vector<std::vector<std::pair<int, geometry_msgs::Pose>>> *real_tags);

    static tf::Pose poseAverage(std::vector<tf::Pose> poses);

    static tf::Quaternion
    getAverageQuaternion(std::vector<tf::Quaternion> &quaternions, std::vector<double> &weights);
};
