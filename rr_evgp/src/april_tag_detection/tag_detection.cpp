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

void tag_detection::draw_opponent(int id, geometry_msgs::Pose april_tag_center) {
    tf::StampedTransform tf_transform;
    tf::Pose april_w;
    tf::poseMsgToTF(april_tag_center, april_w);
    tf_listener.lookupTransform(destination_frame, camera_frame, ros::Time(0), tf_transform);
    april_w = tf_transform * april_w;
    april_w.getOrigin().setZ(0);
    double left = (april_w.getOrigin().x() + x_offset);
    double bottom = (april_w.getOrigin().y() + y_offset) - height / 2.0;

    visualization_msgs::Marker marker;
    std::vector<geometry_msgs::Point> pointList;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    for (double x = 0; x < width; x += 1 / px_per_m) {
        for (double y = 0; y < height; y += 1 / px_per_m) {
            opponent_cloud.push_back(pcl::PointXYZ((left + x) * april_w.getRotation().getX(),
                                                   (bottom + y) * april_w.getRotation().getY(),
                                                   april_w.getOrigin().z()));
            geometry_msgs::Point point;
            point.x = left + x;
            point.y = bottom + y;
            point.z = april_w.getOrigin().z();
            pointList.push_back(point);
        }
    }

    marker.points = pointList;
    geometry_msgs::Pose pose;
    marker.pose = pose;
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
