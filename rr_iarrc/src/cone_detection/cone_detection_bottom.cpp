#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

cv::Mat kernel(int, int);
void cutEnvironment(const cv::Mat&);
void publishMessage(ros::Publisher,const cv::Mat&, std::string);
cv::Mat overlayBinaryGreen(cv::Mat&, const cv::Mat&);
cv_bridge::CvImagePtr cv_ptr;

ros::Publisher pub, pub1, pub2;
int low_H, high_H, low_S, low_V;
int blockSky_height, blockWheels_height, blockBumper_height;

/**
 * Gets the bottom edge of the cones
 * @param msg image from camera
 */
void img_callback(const sensor_msgs::ImageConstPtr& msg) {
    //Convert msg to Mat image
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    int originalHeight = frame.rows;
    int originalWidth = frame.cols;
    cv::resize(frame, frame, cv::Size(400, 400));

    //Get Orange-HSV Cones
    cv::Mat hsv_frame, orange_found, orange_found_rot, nonZeroCoordinates, green_lines;
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_frame, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, 255, 255), orange_found);
    cutEnvironment(orange_found);

    //Rotates Image because findNonZero() gives points already sorted by Y-coordinate
    //Then gets the bottom by getting first Y-coordinate
    //Rotates Image again to make left side the bottom
    cv::rotate(orange_found, orange_found_rot, cv::ROTATE_90_CLOCKWISE);
    cv::findNonZero(orange_found_rot, nonZeroCoordinates);
    cv::Mat bottom_edges(orange_found_rot.size(), CV_8UC1, cv::Scalar::all(0));
    for (int i = 0, last = 0; i < nonZeroCoordinates.total(); i++) {
        int x = nonZeroCoordinates.at<cv::Point>(i).x;
        int y = nonZeroCoordinates.at<cv::Point>(i).y;
        if(y != last) {
            bottom_edges.at<uchar>(y,x) = 255;
            last = y;
        }
    }
    cv::rotate(bottom_edges, bottom_edges, cv::ROTATE_90_COUNTERCLOCKWISE);

    //Dilate, make green overlay, and resize image to initial dimensions
    cv::dilate(bottom_edges, bottom_edges, kernel(2,2));
    green_lines = overlayBinaryGreen(frame, bottom_edges);
    cv::resize(bottom_edges, bottom_edges, cv::Size(originalWidth, originalHeight));

    //publish Images
    publishMessage(pub, bottom_edges, "mono8");
    publishMessage(pub2, orange_found, "mono8");
    publishMessage(pub1, green_lines, "bgr8");
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "coneDetectionBottom");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string subscription_node;

    nhp.param("orange_low_H", low_H, 5);
    nhp.param("orange_high_H", high_H, 15);
    nhp.param("orange_low_S", low_S, 140);
    nhp.param("orange_low_V", low_V, 140);

    nhp.param("blockSky_height", blockSky_height, 220);
    nhp.param("blockWheels_height", blockWheels_height, 200);
    nhp.param("blockBumper_height", blockBumper_height, 200);

    nhp.param("subscription_node", subscription_node, std::string("/camera/image_color_rect"));

    pub = nh.advertise<sensor_msgs::Image>("/cones/bottom/detection_img", 1); //test publish of image
    pub1 = nh.advertise<sensor_msgs::Image>("/cones/bottom/debug_overlay", 1);
    pub2 = nh.advertise<sensor_msgs::Image>("/cones/bottom/debug_hsv", 1);
    auto img_real = nh.subscribe(subscription_node, 1, img_callback);

    ros::spin();
    return 0;
}

cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT,cv::Size(x,y));
}

void cutEnvironment(const cv::Mat& img) {
    cv::rectangle(img,
                  cv::Point(0,0),
                  cv::Point(img.cols,img.rows / 3 + blockSky_height),
                  cv::Scalar(0,0,0),CV_FILLED);

    cv::rectangle(img,
                  cv::Point(0,img.rows),
                  cv::Point(img.cols,2 * img.rows / 3 + blockWheels_height),
                  cv::Scalar(0,0,0),CV_FILLED);

    cv::rectangle(img,
                  cv::Point(img.cols/3,img.rows),
                  cv::Point(2 * img.cols / 3, 2 * img.rows / 3 + blockBumper_height),
                  cv::Scalar(0,0,0),CV_FILLED);
}

void publishMessage(ros::Publisher pub, const cv::Mat& img, std::string img_type) {
    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = img;
        cv_ptr->encoding = img_type;
        cv_ptr->toImageMsg(outmsg);
        pub.publish(outmsg);
    }
}

cv::Mat overlayBinaryGreen(cv::Mat& frame, const cv::Mat& binary) {
    return frame.setTo(cv::Scalar(0,255,0), binary != 0);
}
