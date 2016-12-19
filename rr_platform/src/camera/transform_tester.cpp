#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include "avc/transform_image.h"

using namespace std;


ros::ServiceClient client;
ros::Publisher pub;

void imageCB(const sensor_msgs::ImageConstPtr& msg) {
    avc::transform_image srv;
    srv.request.image = *msg;
    auto success = client.call(srv);
    if(success) {
        //ROS_INFO("successful service request and response");
        pub.publish(srv.response.image);
    } else {
        ROS_ERROR("failed response from transform_image");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "transform_tester");

    ros::NodeHandle nh;
    ros::Subscriber img_sub = nh.subscribe("/camera/image_rect", 1, imageCB);
    client = nh.serviceClient<avc::transform_image>("transform_image");
    pub = nh.advertise<sensor_msgs::Image>(string("/transformed_image"), 1);

    ros::spin();
    return 0;
}