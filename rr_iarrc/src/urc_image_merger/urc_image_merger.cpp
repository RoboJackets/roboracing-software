#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include <rr_platform/speed.h>
#include <rr_platform/steering.h>

using namespace std;

cv_bridge::CvImagePtr cv_ptr;
ros::Publisher image_pub;

rr_platform::speed speed_message;
rr_platform::steering steer_message;

cv::Mat mergeImagesSideBySide(cv::Mat leftImg, cv::Mat rightImg) {
    int outputRows = std::max(leftImg.rows, rightImg.rows);
    int outputCols = leftImg.cols + rightImg.cols;
    cv::Mat merged(outputRows, outputCols, leftImg.type(), cv::Scalar(0,0,0));
    cv::Rect leftHalf(0, 0, leftImg.cols, leftImg.rows);
    cv::Rect rightHalf(leftImg.cols, 0, rightImg.cols, rightImg.rows);
    leftImg.copyTo(merged(leftHalf));
    rightImg.copyTo(merged(rightHalf));

    return merged;
}

void publishMessage(const ros::Publisher pub, const cv::Mat& img, std::string img_type) {
    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = img;
        cv_ptr->encoding = img_type;
        cv_ptr->toImageMsg(outmsg);
        pub.publish(outmsg);
    }
}

/**
 * Reads in an image from two side cameras, rotates them, then merges them.
 * The resize is to make it work with the dragrace center line keeper.
 *
 * @param leftMsg image input from left side camera
 * @param rightMsg image input from the right side camera
 */
void img_callback(const sensor_msgs::ImageConstPtr& leftMsg, const sensor_msgs::ImageConstPtr& rightMsg) {
    //Convert msg to Mat image
    cv_ptr = cv_bridge::toCvCopy(leftMsg, "mono8");
    cv::Mat leftFrame = cv_ptr->image;
    cv_ptr = cv_bridge::toCvCopy(rightMsg, "mono8");
    cv::Mat rightFrame = cv_ptr->image;

    cv::rotate(leftFrame, leftFrame, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::rotate(rightFrame, rightFrame, cv::ROTATE_90_CLOCKWISE);

    cv::Mat debug = mergeImagesSideBySide(leftFrame, rightFrame);

    int resize_dim = 300;
    int original_height = debug.rows;
    int original_width = debug.cols;
    cv::resize(debug, debug, cv::Size(resize_dim * original_width / original_height, resize_dim));

    publishMessage(image_pub, debug, "mono8");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "urc_center_keeper");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string leftCamera_sub_name, rightCamera_sub_name, merged_img_publisher;
    nhp.param("camera_left_subscription", leftCamera_sub_name, std::string("/camera_left/image_color_rect"));
    nhp.param("camera_right_subscription", rightCamera_sub_name, std::string("/camera_right/image_color_rect"));
    nhp.param("merged_img_publisher", merged_img_publisher, std::string("/urc_side_lanes"));

    message_filters::Subscriber<sensor_msgs::Image> leftCamera_sub(nh, leftCamera_sub_name, 1);
    message_filters::Subscriber<sensor_msgs::Image> rightCamera_sub(nh, rightCamera_sub_name, 1);

    image_pub = nh.advertise<sensor_msgs::Image>(merged_img_publisher, 1); //publish debug image

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(3), leftCamera_sub, rightCamera_sub);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));

    ros::spin();
    return 0;
}
