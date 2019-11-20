/*
 * Publishes the number of times the robot has crossed the
 * finish line.
 * Utilize the color_detector and hsv_tuner and Hough lines
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
using namespace cv;
using namespace ros;

using uchar = unsigned char;

// Publisher debug_pub;
Publisher crosses_pub;
int blockSky_height, blockWheels_height, blockBumper_height;

// Debug publisher for my image processing stuff
Publisher debug_pub;
cv_bridge::CvImage debug_img;

// Define the cutoff slope to detect the finish line
double slope_cutoff;
double length_cutoff;

// Defines whether to publish when the line is first detected or when we cross it
bool publish_when_detected;

// Threshold for counting non-zero pixels
int count_thresh;

// In frames, the number cooldown is set to when the finish line is detected
int cooldown_value;

// Cooldown variable to prevent double-detections
int cooldown = 0;

#define HIGH 1
#define LOW 0

// State for making sure we don't publish many times
int state = LOW;

int number_of_crosses = 0;

// Some params for Canny and Hough detection
double canny_thresh1;
double canny_thresh2;
double hough_rho;
double hough_theta;
int hough_thresh;
double hough_minLineLength;
double hough_maxLineGap;

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

    Mat src = frame.clone();
    Mat dst;
    cv::Canny(src, dst, canny_thresh1, canny_thresh2, 3);
    Mat drawing = Mat::zeros(src.size(), CV_8UC3);
    // Do line detection with probabilistic Hough line detection
    vector<Vec4i> linesP;
    cv::HoughLinesP(dst, linesP, hough_rho, hough_theta, hough_thresh, hough_minLineLength, hough_maxLineGap);
    // Calculate slopes and lengths
    double length_sum = 0;
    for (size_t i = 0; i < linesP.size(); i++) {
        Vec4i l = linesP[i];
        cv::Point p1 = cv::Point(l[0], l[1]);
        cv::Point p2 = cv::Point(l[2], l[3]);

        // Calculate length
        cv::Point diff = p1 - p2;
        double length = cv::sqrt(diff.x * diff.x + diff.y * diff.y);

        // Calculate slope
        double slope = cv::abs((double)diff.y / diff.x);
        Scalar color;

        // Filter based on length and slope to remove false positives
        if (length > length_cutoff && slope < slope_cutoff) {
            color = Scalar(0, 255, 0);
            // Add to the running total of length
            length_sum += length;
        } else {
            color = Scalar(0, 0, 255);
        }

        cv::line(drawing, p1, p2, color, 3, LINE_AA);
    }

    // Quick count of pixels as a final sanity check
    auto count = cv::countNonZero(frame);

    // Actually calculate if we've detected the line
    bool detected = length_sum > 2.5 * length_cutoff && count > count_thresh;

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

    // Publish stuff
    std_msgs::Int8 intmsg;
    intmsg.data = number_of_crosses;

    if (debug_pub.getNumSubscribers() > 0) {
        // Draw some debug info
        std::string mode_str =
              std::string("Mode: ") + std::string(publish_when_detected ? "when detected" : "when crossed");
        cv::putText(drawing, mode_str, cv::Point(5, 100), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 143, 143), 2);
        cv::putText(drawing, std::string("Length sum: ") + std::to_string(length_sum), cv::Point(5, 200),
                    cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 143, 143), 2);
        cv::putText(drawing, std::string("Detected: ") + std::string(detected ? "true" : "false"), cv::Point(5, 300),
                    cv::FONT_HERSHEY_SIMPLEX, 2, detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 143, 143), 2);
        cv::putText(drawing, std::string("Num detections: ") + std::to_string(number_of_crosses), cv::Point(5, 400),
                    cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 143, 143), 2);
        cv::putText(drawing, std::string("Count: ") + std::to_string(count), cv::Point(5, 500),
                    cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 143, 143), 2);
        // Convert to ros image format and publish
        debug_img.header = msg->header;
        debug_img.encoding = "bgr8";
        debug_img.image = drawing;
        debug_pub.publish(debug_img.toImageMsg());
        ROS_INFO_STREAM(std::to_string(cv::countNonZero(frame)));
    }

    if (crosses_pub.getNumSubscribers() > 0) {
        crosses_pub.publish(intmsg);
    }
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

    nhp.param("count_thresh", count_thresh, 2000);

    nhp.param("canny_thresh1", canny_thresh1, 50.);
    nhp.param("canny_thresh2", canny_thresh1, 200.);
    nhp.param("hough_rho", hough_rho, 1.);
    nhp.param("hough_theta", hough_theta, CV_PI / 180);
    nhp.param("hough_thresh", hough_thresh, 50);
    nhp.param("hough_minLineLength", hough_minLineLength, 50.);
    nhp.param("hough_maxLineGap", hough_maxLineGap, 35.);

    ROS_INFO("Finish line watching %s", img_topic.c_str());

    Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageCB);

    crosses_pub = nh.advertise<std_msgs::Int8>("finish_line_crosses", 1);
    debug_pub = nh.advertise<sensor_msgs::Image>("finish_line_detection/debug", 1);

    spin();

    return 0;
}
