#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

#include <rr_common/color_filter.h>

std::unique_ptr<rr::ColorFilter> filter;

void callback(const sensor_msgs::ImageConstPtr& msg) {
    auto bridge = cv_bridge::toCvCopy(msg, "bgr8");
    auto img = bridge->image;

    auto t0 = ros::WallTime::now();
    auto filtered = filter->Filter(img);
    ROS_INFO_STREAM("filtering took " << (ros::WallTime::now() - t0).toSec() << " seconds");

    cv::imshow("orig", img);
    cv::imshow("filtered", filtered);
    cv::waitKey(10);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "color_filter_test_node");

    ros::NodeHandle nh;
    auto sub = nh.subscribe("/camera_center/image_color_rect", 1, callback);

    filter = std::make_unique<rr::ColorFilter>(ros::NodeHandle("~filter"));

    ros::spin();
}
