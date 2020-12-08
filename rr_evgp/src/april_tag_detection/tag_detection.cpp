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

    sub_detections = nh->subscribe(tag_detections_topic, 1, &tag_detection::callback, this);
    pub_pointcloud = nh->advertise<sensor_msgs::PointCloud2>(pointcloud, 1);
    pub_markers = nh->advertise<visualization_msgs::Marker>("april_tag_detections/markers", 0);
}

void tag_detection::callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
    opponent_cloud.clear();
    auto msgs = msg->detections;
    for (const auto &message : msgs) {  // Iterate through all discovered April Tags
        tf::Pose april_w = draw_opponent(message.pose.pose.pose);
        geometry_msgs::Pose april_geo_w;
        tf::poseTFToMsg(april_w, april_geo_w);
        create_marker(message.id[0], april_geo_w);
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

tf::Pose tag_detection::draw_opponent(geometry_msgs::Pose april_tag_center) {
    tf::StampedTransform tf_transform;
    tf::Pose april_w;
    tf::poseMsgToTF(april_tag_center, april_w);
    tf_listener.lookupTransform(destination_frame, camera_frame, ros::Time(0), tf_transform);
    april_w = tf_transform * april_w;
    april_w.getOrigin().setZ(0);
    double left = (april_w.getOrigin().x() + x_offset);
    double bottom = (april_w.getOrigin().y() + y_offset) - height / 2.0;

    for (double x = 0; x < width; x += 1 / px_per_m)
        for (double y = 0; y < height; y += 1 / px_per_m)
            opponent_cloud.push_back(pcl::PointXYZ(left + x, bottom + y, april_w.getOrigin().z()));

    return april_w;
}

void tag_detection::create_marker(const int &id, geometry_msgs::Pose april_w) {
    visualization_msgs::Marker marker;
    marker.pose = april_w;
    marker.pose.position.z += height/2;
    marker.pose.position.x += height/2;
    marker.id = id;
    marker.header.stamp = ros::Time(0);
    marker.header.frame_id = destination_frame;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1;
    marker.color.r = id % 10 * 100 % 255;
    marker.color.g = id / 10 % 10 * 100 % 255;
    marker.color.b = id / 100 % 10 * 100 % 255;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    pub_markers.publish(marker);
}
