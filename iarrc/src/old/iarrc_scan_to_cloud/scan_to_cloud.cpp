#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <laser_geometry/laser_geometry.h>

using namespace cv;
using namespace std;
using namespace ros;
using namespace pcl;


Publisher cloud_publisher;
laser_geometry::LaserProjection projector_;
sensor_msgs::PointCloud2 cloud;


void convertToCloudCB(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
    projector_.projectLaser(*scan_in, cloud);
    cloud_publisher.publish(cloud);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "scan_to_cloud");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string laser_topic;
    nhp.param(std::string("laser_topic"), laser_topic, std::string("/scan"));
    ros::Subscriber cone_detect_sub = nh.subscribe(laser_topic, 1, convertToCloudCB);
    cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
    ros::spin();
}
