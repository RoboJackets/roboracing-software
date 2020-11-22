#include <cv_bridge/cv_bridge.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Bool.h>
#include <math.h>

//Determines a start light change from red to green

ros::Publisher debug_img_pub;
ros::Publisher bool_pub;
sensor_msgs::Image outmsg;

std_msgs::Bool prev_start_msg;
ros::Time last_red_time;

double circularityThreshold;

cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT, cv::Size(x, y));
}

bool colorOn(cv::Mat color_img) {
    std::vector<std::vector<cv::Point>> contours;
    findContours(color_img, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    for (const auto &contour : contours) {
        double perimeter = cv::arcLength(contour, true);
        double area = cv::contourArea(contour, false);
        double circularity = 4 * M_PI * (area / (perimeter * perimeter));

        if (circularityThreshold < circularity && area > 100)  // These could be launch params
            return true;
    }
    return false;
}

void img_callback(const sensor_msgs::Image::ConstPtr &msg) {
    // keeps publishing true if green was previously seen
    if (prev_start_msg.data) {
        bool_pub.publish(prev_start_msg);
        return;
    }

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    cv::Mat hsv_frame, red_found, green_found;
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_frame, cv::Scalar(0, 100, 140), cv::Scalar(10, 255, 255), red_found);
    cv::inRange(hsv_frame, cv::Scalar(20, 120, 120), cv::Scalar(100, 255, 255), green_found);

    cv::GaussianBlur(frame, frame, cv::Size(3,3), 0, 0);

    cv::morphologyEx(green_found, green_found, cv::MORPH_OPEN, kernel(3, 3));
    cv::morphologyEx(red_found, red_found, cv::MORPH_OPEN, kernel(3, 3));

    cv::dilate(green_found, green_found, kernel(3, 3));
    cv::dilate(red_found, red_found, kernel(3, 3));

    bool red_on = colorOn(red_found);
    bool green_on = colorOn(green_found);

    if (red_on)
        last_red_time = msg->header.stamp;

    prev_start_msg.data = green_on && (msg->header.stamp - last_red_time).toSec() < 1;
    bool_pub.publish(prev_start_msg);

    sensor_msgs::Image outmsg;
    cv_ptr->image = red_found;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(outmsg);
    debug_img_pub.publish(outmsg);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "stoplight_watcher_v2");
    ros::NodeHandle nhp("~");

    std::string img_topic;
    std::string stoplight_topic;
    nhp.param("img_topic", img_topic, std::string("/camera/image_color_rect"));
    nhp.param("stoplight_watcher_topic", stoplight_topic, std::string("/start_detected"));

    nhp.param("circularity_threshold", circularityThreshold, 0.7);

    // Subscribe to ROS topic with callback
    prev_start_msg.data = false;
    ros::Subscriber img_sub = nhp.subscribe(img_topic, 1, img_callback);
    debug_img_pub = nhp.advertise<sensor_msgs::Image>("/startlight_debug", 1);
    bool_pub = nhp.advertise<std_msgs::Bool>(stoplight_topic, 1);

    ros::spin();
    return 0;
}