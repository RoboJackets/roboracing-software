//
// Created by Charles Jenkins on 11/27/20.
//

#include "tag_detection.h"

#include <utility>

tag_detection::tag_detection(ros::NodeHandle *nh, const std::string &camera_frame, const std::string &pointcloud,
                             const std::string &tag_detections_topic, const std::string &destination_frame,
                             const std::string &tag_detection_markers, double x_offset,
                             double y_offset, double px_per_m, double width, double height,
                             std::vector<april_robot> robots) {
    this->camera_frame = camera_frame;
    this->destination_frame = destination_frame;
    this->width = width;
    this->robots = std::move(robots);

    double left = x_offset - width / 2;
    double bottom = y_offset;
    for (double x = 0; x < width; x += 1 / px_per_m) {
        for (double y = 0; y < height; y += 1 / px_per_m) {
            geometry_msgs::Point point;
            point.x = left + x;
            point.y = bottom + y;
            point.z = 0;

            pcl_outline.push_back(pcl::PointXYZ(point.x, point.y, 0));
            marker_outline.push_back(point);
        }
    }

    //Storing as an instance variable keeps subscriber alive
    sub_detections = nh->subscribe(tag_detections_topic, 1, &tag_detection::callback, this);
    pub_pointcloud = nh->advertise<sensor_msgs::PointCloud2>(pointcloud, 1);
    pub_markers = nh->advertise<visualization_msgs::Marker>(tag_detection_markers, 0);
}

void tag_detection::callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
    opponent_cloud.clear();
    auto msgs = msg->detections;
    std::vector<std::vector<std::pair<int, geometry_msgs::Pose>>> tag_groups(5); //TODO: Add as param
    for (const auto &message : msgs) {  // Iterate through all discovered April Tags
        int car_number = message.id[0] / 10; // By the specs, car 1 has tags 10,11,12... car 2 has 20,21,22...
        tag_groups[car_number].push_back(std::pair(message.id[0], message.pose.pose.pose));
//        message.id[0]
        draw_opponent(message.id[0], message.pose.pose.pose);
    }
    publishPointCloud(opponent_cloud);
}

void tag_detection::publishPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud) {
    sensor_msgs::PointCloud2 outmsg;
    pcl::toROSMsg(cloud, outmsg);
    outmsg.header.frame_id = destination_frame;
    pub_pointcloud.publish(outmsg);
}

void tag_detection::draw_opponent(int id, geometry_msgs::Pose april_camera_msg) {
    tf_listener.lookupTransform(this->destination_frame, this->camera_frame, ros::Time(0), camera_w);

//    april_robot& curr;
    for (april_robot &robot : robots) {

    }

    tf::Pose april_w;
    tf::poseMsgToTF(april_camera_msg, april_w);
    april_w = camera_w * april_w;

    tfScalar roll, pitch, yaw;
    april_w.getBasis().getRPY(roll, pitch, yaw);
    tf::Matrix3x3 april_w_basis;
    april_w_basis.setRPY(0, 0, yaw);
    april_w.setBasis(april_w_basis);

    tf::Vector3 april_w_origin = april_w.getOrigin();
    april_w_origin.setZ(0);
    april_w.setOrigin(april_w_origin);

    pcl::PointCloud<pcl::PointXYZ> car_outline;
    Eigen::Affine3d affine3d;
    tf::transformTFToEigen(april_w, affine3d);
    pcl::transformPointCloud(pcl_outline, car_outline, affine3d);
    opponent_cloud += (car_outline);

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Pose april_msg;
    tf::poseTFToMsg(april_w, april_msg);
    marker.points = marker_outline;
    marker.pose = april_msg;
    marker.id = id;
    marker.color.a = 1;
    marker.color.r = colors[id % colors.size()][0];
    marker.color.g = colors[id % colors.size()][1];
    marker.color.b = colors[id % colors.size()][2];
    marker.scale.x = .1;
    marker.scale.y = .1;
    marker.scale.z = .1;
    marker.header.stamp = ros::Time(0);
    marker.header.frame_id = destination_frame;
    pub_markers.publish(marker);
}
