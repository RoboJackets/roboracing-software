/*
 * Publishes the number of times the robot has crossed the
 * finish line.
 * Utilize the color_detector and hsv_tuner and contours
 * to detect the finish line.
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
#include <opencv2/opencv.hpp>

using namespace std;

using uchar = unsigned char;

// Publisher debug_pub;
ros::Publisher crosses_pub;
int blockSky_height, blockWheels_height, blockBumper_height;

// Debug publisher for my image processing stuff
ros::Publisher debug_pub;
cv_bridge::CvImage debug_img;

// Define the cutoff angle and width to detect the line and the cutoff area for contours
double angle_cutoff;
double width_cutoff;
double min_contour_area;
double area_cutoff;

// Defines whether to publish when the line is first detected or when we cross it
bool publish_when_detected;

// In frames, the number cooldown is set to when the finish line is detected
int cooldown_value;

// Cooldown variable to prevent double-detections
int cooldown = 0;

#define HIGH 1
#define LOW 0

// State for making sure we don't publish many times
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
    cv::Mat frame;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV-Bridge error: %s", e.what());
        return;
    }

    frame = cv_ptr->image;
    blockEnvironment(frame);

    // Matrix used to draw debug info
    cv::Mat debugDrawing = cv::Mat::zeros(frame.size(), CV_8UC3);
    cv::cvtColor(frame, debugDrawing, cv::COLOR_GRAY2BGR);

    // Find contours
    vector<vector<cv::Point>> contours;
    cv::findContours(frame, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // Filter out very small contours
    vector<cv::Point> filteredContourPoints;
    for (int i = 0; i < contours.size(); i++) {
        cv::Scalar color(0, 0, 255);

        if (cv::contourArea(contours.at(i)) > min_contour_area) {
            filteredContourPoints.insert(std::end(filteredContourPoints), std::begin(contours.at(i)),
                                         std::end(contours.at(i)));
            color = cv::Scalar(0, 255, 0);
        }

        // Debug draw
        cv::drawContours(debugDrawing, contours, i, color, 2, 8);
    }

    bool detected = false;
    bool drawRectDebug = false;
    cv::RotatedRect fitRect;
    cv::Size2f size;
    float angle;

    if (filteredContourPoints.size() > 0) {
        // Fit a rect to the points and detect based on angle and width
        fitRect = cv::minAreaRect(filteredContourPoints);
        size = fitRect.size;
        drawRectDebug = true;
        // Check the detection criteria (must look at both width and height because OpenCV is inconsistent about how it
        // assigns them)
        if (size.height > size.width)
            angle = std::abs(std::abs(fitRect.angle) - 90);

        detected = angle < angle_cutoff && (size.width > width_cutoff || size.height > width_cutoff);

        cv::Point2f rectPoints[4];
        fitRect.points(rectPoints);

        for (int i = 0; i < 4; i++)
            cv::line(debugDrawing, rectPoints[i], rectPoints[(i + 1) % 4], cv::Scalar(255, 0, 0), 2, 12);
    }

    auto incrementCrossNum = false;

    // Update state (HIGH=line currently visible, LOW=not visible), set cooldown, and set whether we should increment
    // number of crosses based on whether we want to publish when detected or when we lose detection
    if (state == LOW && detected && cooldown == 0) {
        incrementCrossNum = publish_when_detected;
        state = HIGH;
    } else if (state == HIGH && !detected) {
        incrementCrossNum = !publish_when_detected;
        state = LOW;
    }

    if (incrementCrossNum) {
        number_of_crosses++;
        cooldown = cooldown_value;
        ROS_INFO_STREAM("Finish line crossed: " << to_string(number_of_crosses));
    }

    if (cooldown > 0)
        cooldown--;

    // Publish stuff
    std_msgs::Int8 intmsg;
    intmsg.data = number_of_crosses;

    if (debug_pub.getNumSubscribers() > 0) {
        // Draw some debug info
        if (drawRectDebug) {
            std::string angle_str = std::string("Angle: ") + std::to_string(angle);
            cv::putText(debugDrawing, angle_str, cv::Point(5, 100), cv::FONT_HERSHEY_SIMPLEX, 2,
                        cv::Scalar(0, 143, 143), 2);
            std::string width_str = std::string("Width: ") + std::to_string(size.width);
            cv::putText(debugDrawing, width_str, cv::Point(5, 200), cv::FONT_HERSHEY_SIMPLEX, 2,
                        cv::Scalar(0, 143, 143), 2);
            std::string height_str = std::string("Height: ") + std::to_string(size.height);
            cv::putText(debugDrawing, height_str, cv::Point(5, 300), cv::FONT_HERSHEY_SIMPLEX, 2,
                        cv::Scalar(0, 143, 143), 2);
        }
        std::string mode_str =
              std::string("Mode: ") + std::string(publish_when_detected ? "when detected" : "when crossed");
        cv::putText(debugDrawing, mode_str, cv::Point(5, 400), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 143, 143), 2);
        std::string detected_str = std::string("Detected: ") + std::to_string(detected);
        cv::putText(debugDrawing, detected_str, cv::Point(5, 500), cv::FONT_HERSHEY_SIMPLEX, 2,
                    (detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 143, 143)), 2);
        std::string count_str = std::string("Num detections: ") + std::to_string(number_of_crosses);
        cv::putText(debugDrawing, count_str, cv::Point(5, 600), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 143, 143),
                    2);

        // Convert to ros image format and publish
        debug_img.header = msg->header;
        debug_img.encoding = "bgr8";
        debug_img.image = debugDrawing;
        debug_pub.publish(debug_img.toImageMsg());
    }

    if (crosses_pub.getNumSubscribers() > 0)
        crosses_pub.publish(intmsg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "finish_line_watcher");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    string img_topic;
    nhp.getParam("img_topic", img_topic);

    nhp.param("blockSky_height", blockSky_height, 0);
    nhp.param("blockWheels_height", blockWheels_height, 800);
    nhp.param("blockBumper_height", blockBumper_height, 800);

    nhp.param("publish_when_detected", publish_when_detected, true);

    nhp.param("min_contour_area", min_contour_area, 5.0);
    nhp.param("angle_cutoff", angle_cutoff, 20.0);
    nhp.param("width_cutoff", width_cutoff, 275.0);

    ROS_INFO("Finish line watching %s", img_topic.c_str());

    ros::Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageCB);

    crosses_pub = nh.advertise<std_msgs::Int8>("finish_line_crosses", 1);
    debug_pub = nh.advertise<sensor_msgs::Image>("finish_line_detection/debug", 1);

    ros::spin();

    return 0;
}
