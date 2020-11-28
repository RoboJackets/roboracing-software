#include "tag_detection.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "april_tag_pointcloud");
    ros::NodeHandle nhp("~");
    ros::NodeHandle nh;
    std::string camera_frame, pointcloud;
    nhp.getParam("/camera_frame", camera_frame);
    nhp.getParam("/pointcloud", pointcloud);
    tag_detection tagDetection(camera_frame, pointcloud);

    ros::spin();
    return 0;
}
