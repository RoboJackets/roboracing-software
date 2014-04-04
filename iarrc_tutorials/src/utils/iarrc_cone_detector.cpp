
#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

std::string img_file;


using namespace cv;
using namespace std;


ros::Publisher cone_publisher;


void ConeDetectorCB(const sensor_msgs::LaserScan::ConstPtr& msg) {





}

void help(std::ostream& ostr) {
    ostr << "Usage: iarrc_cone_detector _laser_topic:=<laser-topic> _img_file:=<file-name>" << std::endl;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "iarrc_cone_detector");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // FIXME: Not expected behavior
    if(argc >= 2) {
        help(std::cerr);
        exit(1);
    }

    std::string laser_topic;
    nhp.param(std::string("laser_topic"), laser_topic, std::string("/scan"));

    ROS_INFO("Laser topic:= %s", laser_topic.c_str());
    ROS_INFO("Laser file:= %s", img_file.c_str());

    // Subscribe to ROS topic with callback
    ros::Subscriber cone_detect_sub = nh.subscribe(laser_topic, 1, ConeDetectorCB);
    cone_publisher = nh.advertise<sensor_msgs::LaserScan>("hist_cone_pub", 1); //EDIT LATER


    ROS_INFO("IARRC cone detection node ready.");
    ros::spin();
    ROS_INFO("Shutting down IARRC cone detection node.");
    return 0;
}
