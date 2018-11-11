//
// Created by robojackets on 9/17/16.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pc_pub;
double filtering_distance;


laser_geometry::LaserProjection projector;

void scanCallback(const sensor_msgs::LaserScanConstPtr &msg) {
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*msg, cloud);

    pcl::PCLPointCloud2 cloud_pc2;
    pcl_conversions::toPCL(cloud,cloud_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud_pc2, *cloud_pc);

    for(auto iter = cloud_pc->begin(); iter != cloud_pc->end();) {
        auto point = *iter;
        auto distance = std::sqrt((point.x*point.x) + (point.y*point.y));
        if(distance < filtering_distance) {
            iter = cloud_pc->erase(iter);
        } else {
            iter++;
        }
    }

    pcl::toPCLPointCloud2(*cloud_pc,cloud_pc2);
    pcl_conversions::fromPCL(cloud_pc2,cloud);

    pc_pub.publish(cloud);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "scanToPointCloud");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    
    nhp.param("min_point_dist", filtering_distance, double(1.0));//why am i assinging it a value here at all? Do the launch files 
    //parameters override it 

    auto scan_sub = nh.subscribe("scan", 1, scanCallback);

    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("scan/pointcloud", 1);

    ros::spin();

    return 0;
}
