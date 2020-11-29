//
// Created by Charles Jenkins on 11/27/20.
//

#include "tag_detection.h"

tag_detection::tag_detection(ros::NodeHandle *nh, std::string camera_frame, const std::string &pointcloud,
                             const std::string &tag_detections_topic) {
    this->camera_frame = std::move(camera_frame);
    sub_detections = nh->subscribe(tag_detections_topic, 1, &tag_detection::callback, this);
    pub_pointcloud = nh->advertise<sensor_msgs::PointCloud2>(pointcloud, 1);
}

void tag_detection::callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
    auto msgs = msg->detections;
    draw_opponents(msg);
    for (const auto &message : msgs) {  // Iterate through all discovered April Tags
        ROS_INFO_STREAM((message.id[0]));
    }
}

void tag_detection::draw_opponents(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
    opponent_cloud.clear();
    auto msgs = msg->detections;
    for (const auto &message : msgs) {  // Iterate through all discovered April Tags
        draw_opponent(message.id[0], message.pose.pose.pose);
    }
    publishPointCloud(opponent_cloud);
}

void tag_detection::publishPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud) {
    sensor_msgs::PointCloud2 outmsg;
    pcl::toROSMsg(cloud, outmsg);
    outmsg.header.frame_id = camera_frame;
    pub_pointcloud.publish(outmsg);
}

void tag_detection::draw_opponent(const int &id, geometry_msgs::Pose_<std::allocator<void>> april_tag_center) {
    int side_length = 5;
    int num_points = 10;
    double ratio = (double)side_length / num_points;
    double start_y = (april_tag_center.position.y - side_length / 2.0);
    double start_x = april_tag_center.position.x - side_length / 2.0;
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix = matrix_rotation(april_tag_center.orientation.x, april_tag_center.orientation.y,
                                      april_tag_center.orientation.z);

    for (int x = 0; x < num_points; x++) {
        for (int y = 0; y < num_points; y++) {
            for (int z = 0; z < num_points; z++) {
                Eigen::Vector3f coords;
                coords << (start_x + x * ratio), (start_y + y * ratio), (april_tag_center.position.z + z * ratio);
                coords = rotation_matrix * coords;
                opponent_cloud.push_back(pcl::PointXYZ((float)(coords.x()), (float)(coords.y()), (float)(coords.z())));
            }
        }
    }
}

Eigen::Matrix3f tag_detection::matrix_rotation(double x, double y, double z) {
    Eigen::Matrix3f x_matrix, y_matrix, z_matrix;
    x_matrix << 1, 0, 0, 0, cos(x), -sin(x), 0, sin(x), cos(x);
    y_matrix << cos(y), 0, sin(y), 0, 1, 0, -sin(y), 0, cos(y);
    z_matrix << cos(z), -sin(z), 0, sin(z), cos(z), 0, 0, 0, 1;

    return x_matrix * y_matrix * z_matrix;
}
