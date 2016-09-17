//
// Created by robojackets on 9/17/16.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

ros::Publisher pc_pub;

laser_geometry::LaserProjection projector;

void scanCallback(const sensor_msgs::LaserScanConstPtr &msg) {
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*msg, cloud);
    pc_pub.publish(cloud);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "scanToPointCloud");

    ros::NodeHandle nh;

    auto scan_sub = nh.subscribe("scan", 1, scanCallback);

    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("scan/pointcloud", 1);

    ros::spin();

    return 0;
}