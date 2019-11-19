/*
 * Publishes the number of times the robot has crossed the
 * finish line.
 * Utilize the color_detector and hsv_tuner to detect the
 * finish line.
 */

#include <cv_bridge/cv_bridge.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <rr_msgs/speed.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int8.h>
#include <climits>
#include <cmath>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace ros;

using uchar = unsigned char;

// Publisher debug_pub;
Publisher crosses_pub;
int blockSky_height, blockWheels_height, blockBumper_height;

// Debug publisher for my image processing stuff
Publisher debug_pub;
cv_bridge::CvImage debug_img;

// Define the cutoff slope to detect the finish line, should be paramaterized
double slope_cutoff;
double length_cutoff;

bool publish_when_detected;

int cooldown_value = 7;

// Quick cooldown variable to prevent double-detections
int cooldown = 0;

#define HIGH 1
#define LOW 0

// Quick state for making sure we don't publish many times
int state = LOW;

int number_of_crosses = 0;

void blockEnvironment(const cv::Mat& img) {
    cv::rectangle(img, cv::Point(0, 0), cv::Point(img.cols, blockSky_height), cv::Scalar(0), CV_FILLED);

    cv::rectangle(img, cv::Point(0, img.rows), cv::Point(img.cols, blockWheels_height), cv::Scalar(0), CV_FILLED);

    cv::rectangle(img, cv::Point(img.cols / 3, img.rows), cv::Point(2 * img.cols / 3, blockBumper_height),
                  cv::Scalar(0), CV_FILLED);
}

void ImageCB(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    Mat frame;
    Mat output;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV-Bridge error: %s", e.what());
        return;
    }

    frame = cv_ptr->image;
    blockEnvironment(frame);

    // Do contour detection
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat contourImg = frame.clone();
    findContours(contourImg, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    Mat drawing = Mat::zeros(contourImg.size(), CV_8UC3);
    double maxLength = 0.0;
    double maxLengthYPos = 0.0;
    for (size_t i = 0; i < contours.size(); i++) {
        // Find the average slopes and filter out ones with big slope
        Point start = contours.at(i).at(0);
        Point end = contours.at(i).at((contours.at(i).size() - 1) / 2);
        double rise = start.y - end.y;
        double run = start.x - end.x + 0.01;
        double slope = std::abs(rise / run);
        double length = arcLength(contours.at(i), false);

        if (slope > 0.0001 && slope < slope_cutoff && length > length_cutoff) {
            drawContours(drawing, contours, (int)i, Scalar(255, 0, 0), 2, LINE_8, hierarchy, 0);

            if (length > maxLength) {
                maxLength = length;
                maxLengthYPos = contours.at(i).at((int)(contours.at(i).size() / 2)).y;
            }

        } else {
            drawContours(drawing, contours, (int)i, Scalar(0, 255, 0), 2, LINE_8, hierarchy, 0);
        }
    }

    // Convert to ros image format
    debug_img.header = msg->header;
    debug_img.encoding = "bgr8";
    debug_img.image = drawing;

    bool detected = maxLength > length_cutoff;

    if (publish_when_detected) {
        // Publish when first detected but don't keep on increasing number of crosses, so make use of states again
        if (detected && state == LOW && cooldown == 0) {
            number_of_crosses++;
            ROS_INFO_STREAM("Finish line crossed: " << to_string(number_of_crosses));
            state = HIGH;
            cooldown = cooldown_value;

        } else if (state == HIGH && !detected) {
            state = LOW;
        }

    } else {
        // When the line is first detected, set to a high state then when we stop detecting (we've crossed), report that
        if (state == LOW && detected && cooldown == 0) {
            state = HIGH;

        } else if (state == HIGH && !detected) {
            state = LOW;
            number_of_crosses++;
            cooldown = cooldown_value;
            ROS_INFO_STREAM("Finish line crossed: " << to_string(number_of_crosses));
        }
    }

    if (cooldown > 0)
        cooldown--;
}

int main(int argc, char** argv) {
    init(argc, argv, "finish_line_watcher");

    NodeHandle nh;
    NodeHandle nhp("~");

    string img_topic;
    nhp.getParam("img_topic", img_topic);

    nhp.param("blockSky_height", blockSky_height, 0);
    nhp.param("blockWheels_height", blockWheels_height, 800);
    nhp.param("blockBumper_height", blockBumper_height, 800);

    nhp.param("publish_when_detected", publish_when_detected, true);

    nhp.param("slope_cutoff", slope_cutoff, 0.05);
    nhp.param("length_cutoff", length_cutoff, 800.0);

    nhp.param("cooldown", cooldown_value, 7);

    ROS_INFO("Finish line watching %s", img_topic.c_str());

    Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageCB);

    crosses_pub = nh.advertise<std_msgs::Int8>("finish_line_crosses", 1);
    debug_pub = nh.advertise<sensor_msgs::Image>("finish_line_detection/debug", 1);

    Rate rate(30);
    while (ros::ok()) {
        std_msgs::Int8 intmsg;
        intmsg.data = number_of_crosses;
        crosses_pub.publish(intmsg);
        debug_pub.publish(debug_img.toImageMsg());

        spinOnce();
        rate.sleep();
    }

    return 0;
}
