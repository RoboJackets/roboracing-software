//
// Created by robojackets on 9/17/16.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "avc/calibrate_image.h"

using namespace ros;

sensor_msgs::Image image;
bool got_image = false;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    image = *msg;
    got_image = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trigger_calibration");

    NodeHandle nh;

    ServiceClient client = nh.serviceClient<avc::calibrate_image>("/calibrate_image");

    ros::Subscriber subscriber = nh.subscribe("/camera/image_rect", 1, imageCallback);

    while(!got_image) {
        ros::spinOnce();
    }

    avc::calibrate_image srv;
    srv.request.image = image;
    srv.request.mapPixelsPerMeter = 100;
    srv.request.chessboardRows = 6;
    srv.request.chessboardCols = 8;
    srv.request.squareWidth = 0.033;

    if(client.call(srv)) {
        ROS_INFO("SUCCESS");
    }

    return 0;
}