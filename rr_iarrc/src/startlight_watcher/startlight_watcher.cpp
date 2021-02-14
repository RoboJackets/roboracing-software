#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>

#include <opencv2/opencv.hpp>

// Determines when the start light changes from red to green

ros::Publisher debug_img_pub;
ros::Publisher bool_pub;
sensor_msgs::Image outmsg;

std_msgs::Bool prev_start_msg;
ros::Time last_red_time;

std::vector<cv::Moments> lastRedCircles;

double circularityThreshold;
int minArea;

int minGreenHue, maxGreenHue, minRedHue, maxRedHue;
double redToGreenTime;

bool keepPublishing;

int tolerance;
cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT, cv::Size(x, y));
}

std::vector<cv::Moments> findCircularContours(cv::Mat color_img) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Moments> circles;
    findContours(color_img, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    for (const auto &contour : contours) {
        double perimeter = cv::arcLength(contour, true);
        double area = cv::contourArea(contour, false);
        double circularity = 4 * M_PI * (area / (perimeter * perimeter));

        if (circularityThreshold < circularity && area > minArea) {
            cv::Moments moments = cv::moments(contour);
            circles.push_back(moments);
        }
    }
    return circles;
}

bool greenOn(std::vector<cv::Moments> greenCircles, std::vector<cv::Moments> lastRedCircles) {
    if (greenCircles.size() > 0) {
        for (const auto &greenCircle : greenCircles) {
            int greenCenterX = greenCircle.m10 / greenCircle.m00;
            int greenCenterY = greenCircle.m01 / greenCircle.m00;
            for (const auto &redCircle : lastRedCircles) {
                int redCenterX = redCircle.m10 / redCircle.m00;
                int redCenterY = redCircle.m01 / redCircle.m00;
                int distanceSquared = pow((redCenterX - greenCenterX), 2) + pow((redCenterY - greenCenterY), 2);
                if (distanceSquared < tolerance * tolerance) {
                    return true;
                }
            }
        }
    }
    return false;
}

void img_callback(const sensor_msgs::Image::ConstPtr &msg) {
    // keeps publishing true if green was previously seen
    if (prev_start_msg.data && !keepPublishing) {
        bool_pub.publish(prev_start_msg);
        return;
    }

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    cv::Mat hsv_frame, red_found, green_found, debugImage;
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_frame, cv::Scalar(minRedHue, 100, 140), cv::Scalar(maxRedHue, 255, 255), red_found);
    cv::inRange(hsv_frame, cv::Scalar(minGreenHue, 120, 120), cv::Scalar(maxGreenHue, 255, 255), green_found);

    cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0, 0);

    cv::morphologyEx(green_found, green_found, cv::MORPH_OPEN, kernel(3, 3));
    cv::morphologyEx(red_found, red_found, cv::MORPH_OPEN, kernel(3, 3));

    cv::dilate(green_found, green_found, kernel(3, 3));
    cv::dilate(red_found, red_found, kernel(3, 3));

    std::vector<cv::Moments> redCircles = findCircularContours(red_found);
    std::vector<cv::Moments> greenCircles = findCircularContours(green_found);

    if (redCircles.size() != 0) {
        last_red_time = msg->header.stamp;
        lastRedCircles = redCircles;
    }

    prev_start_msg.data =
          (msg->header.stamp - last_red_time).toSec() < redToGreenTime && greenOn(greenCircles, lastRedCircles);

    bool_pub.publish(prev_start_msg);

    sensor_msgs::Image outmsg;

    // sets the found pixels in the image to either red or green
    cv::cvtColor(red_found, red_found, cv::COLOR_GRAY2RGB);
    red_found.setTo(cv::Scalar(255, 0, 0), red_found);
    cv::cvtColor(green_found, green_found, cv::COLOR_GRAY2RGB);
    green_found.setTo(cv::Scalar(0, 255, 0), green_found);

    debugImage = red_found + green_found;

    cv_ptr->image = debugImage;
    cv_ptr->encoding = "rgb8";
    cv_ptr->toImageMsg(outmsg);
    debug_img_pub.publish(outmsg);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "startlight_watcher");
    ros::NodeHandle nhp("~");

    std::string img_topic;
    std::string startlight_topic;
    nhp.param("img_topic", img_topic, std::string("/camera/image_color_rect"));
    nhp.param("startlight_watcher_topic", startlight_topic, std::string("/start_detected"));

    nhp.param("circularity_threshold", circularityThreshold, 0.7);

    nhp.param("min_green_hue", minGreenHue, 20);
    nhp.param("max_green_hue", maxGreenHue, 100);
    nhp.param("min_red_hue", minRedHue, 0);
    nhp.param("max_red_hue", maxRedHue, 20);

    nhp.param("min_area", minArea, 100);

    nhp.param("red_to_green_time", redToGreenTime, 1.0);

    nhp.param("keep_publishing", keepPublishing, false);

    nhp.param("tolerance", tolerance, 20);

    // Subscribe to ROS topic with callback
    prev_start_msg.data = false;
    ros::Subscriber img_sub = nhp.subscribe(img_topic, 1, img_callback);
    debug_img_pub = nhp.advertise<sensor_msgs::Image>("/startlight_debug", 1);
    bool_pub = nhp.advertise<std_msgs::Bool>(startlight_topic, 1);

    ros::spin();
    return 0;
}