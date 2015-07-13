#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <iarrc/constants.hpp>

using namespace cv_bridge;

ros::Publisher proj_pub;

void imageCallback(const sensor_msgs::Image::ConstPtr& image) {
    CvImageConstPtr cv_ptr;
    cv_ptr = toCvShare(image, "bgr8");
    cv::Mat proj;

    cv::warpPerspective(cv_ptr->image, proj, constants::H, cv::Size(800, 800));

    std_msgs::Header header;
    CvImage projImage(header, "bgr8", proj);

    proj_pub.publish(projImage.toImageMsg());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "projection");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("/image_lines", 1, imageCallback);
    proj_pub = n.advertise<sensor_msgs::Image>("image_projected", 100);

    ros::spin();
    return 0;
}
