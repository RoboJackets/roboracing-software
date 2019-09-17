/*
 * Publishes the number of times the robot has crossed the
 * finish line.
 * Utilize the color_detector and hsv_tuner to detect the
 * finish line.
 */

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <climits>
#include <rr_platform/speed.h>
#include <std_msgs/Int8.h>

using namespace std;
using namespace cv;
using namespace ros;

using uchar = unsigned char;

// Publisher debug_pub;
Publisher crosses_pub;
int blockSky_height, blockWheels_height, blockBumper_height;

#define HIGH 1
#define LOW 0

int state = LOW;

int number_of_crosses = 0;

void blockEnvironment(const cv::Mat& img) {
    cv::rectangle(img,
                  cv::Point(0,0),
                  cv::Point(img.cols, blockSky_height),
                  cv::Scalar(0),CV_FILLED);

    cv::rectangle(img,
                  cv::Point(0,img.rows),
                  cv::Point(img.cols, blockWheels_height),
                  cv::Scalar(0),CV_FILLED);

    cv::rectangle(img,
                  cv::Point(img.cols/3,img.rows),
                  cv::Point(2 * img.cols / 3, blockBumper_height),
                  cv::Scalar(0),CV_FILLED);
}

int getWidth(const Mat& image) {

    int width = 0;

    bool in_line = false;
    int start;

    for(int r = 0; r < image.rows; r++) {
        auto row = image.ptr<uchar>(r);
        for(int c = 0; c < image.cols; c++) {
            if(!in_line && row[c]) {
                in_line = true;
                start = c;
            } else if(in_line && !row[c]) {
                in_line = false;
                auto my_width = c - start;
                width = max(width, my_width);
            }
        }
    }
    return width;
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

    auto count = countNonZero(frame);

    auto width = getWidth(frame);


    if(state == LOW && count > 2000 && width > 700) {
        state = HIGH;
    } else if(state == HIGH && count < 2000) {
        state = LOW;
        number_of_crosses++;
        ROS_INFO_STREAM("Finish line crossed: " << to_string(number_of_crosses));
    }

    // if(count > 2000 && width > 600) {
    //     number_of_crosses = 1;
    //     ROS_INFO_STREAM("Finish line crossed: " << to_string(number_of_crosses));
    // }
}

int main(int argc, char** argv) {

    init(argc, argv, "finish_line_watcher");

    NodeHandle nh;
    NodeHandle nhp("~");

    string img_topic;
    nhp.getParam("img_topic", img_topic);

    nhp.param("blockSky_height",    blockSky_height, 0);
    nhp.param("blockWheels_height", blockWheels_height, 800);
    nhp.param("blockBumper_height", blockBumper_height, 800);

    ROS_INFO("Finish line watching %s", img_topic.c_str());

    Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageCB);

    crosses_pub = nh.advertise<std_msgs::Int8>("finish_line_crosses", 1);

    Rate rate(30);
    while(ros::ok()) {
        std_msgs::Int8 intmsg;
        intmsg.data = number_of_crosses;
        crosses_pub.publish(intmsg);

        spinOnce();
        rate.sleep();
    }

    return 0;
}
