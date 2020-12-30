//
// Created by Charles Jenkins on 11/27/20.
//

#include "tag_detection.h"

tag_detection::tag_detection(ros::NodeHandle *nh, std::string camera_frame, const std::string &pointcloud,
                             const std::string &tag_detections_topic, std::string destination_frame, double x_offset,
                             double y_offset, double px_per_m, double width, double height) {
    this->camera_frame = std::move(camera_frame);
    this->destination_frame = std::move(destination_frame);
    this->x_offset = x_offset;
    this->y_offset = y_offset;
    this->px_per_m = px_per_m;
    this->width = width;
    this->height = height;

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

    sub_detections = nh->subscribe(tag_detections_topic, 1, &tag_detection::callback, this);
    pub_pointcloud = nh->advertise<sensor_msgs::PointCloud2>(pointcloud, 1);
    pub_markers = nh->advertise<visualization_msgs::Marker>("april_tag_detections/markers", 0);
}

void tag_detection::callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
    opponent_cloud.clear();
    auto msgs = msg->detections;
    for (const auto &message : msgs) {  // Iterate through all discovered April Tags
        draw_opponent(message.id[0], message.pose.pose.pose);
    }
    publishPointCloud(opponent_cloud);
    for (const auto &message : msgs) {  // Iterate through all discovered April Tags
        ROS_INFO_STREAM((message.id[0]));
    }
}

void tag_detection::publishPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud) {
    sensor_msgs::PointCloud2 outmsg;
    pcl::toROSMsg(cloud, outmsg);
    outmsg.header.frame_id = destination_frame;
    pub_pointcloud.publish(outmsg);
}

void tag_detection::draw_opponent(int id, geometry_msgs::Pose april_camera_msg) {
    tf_listener.lookupTransform(this->destination_frame, this->camera_frame, ros::Time(0), camera_w);

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

    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> points = car_outline.points;
    geometry_msgs::Pose pose;

    visualization_msgs::Marker marker;
    std::vector<geometry_msgs::Point> pointList;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Pose april_msg;
    tf::poseTFToMsg(april_w, april_msg);
    marker.points = marker_outline;
    marker.pose = april_msg;
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
