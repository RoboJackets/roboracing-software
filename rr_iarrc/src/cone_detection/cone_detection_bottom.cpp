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

using namespace cv;

cv_bridge::CvImagePtr cv_ptr;
ros::Publisher pub, pub1, pub2;

cv::Mat kernel(int, int);
void cutEnvironment(cv::Mat);
void publishMessage(ros::Publisher, Mat, std::string);
Mat overlayBinaryGreen(Mat, Mat);

int blockSky_height, blockWheels_height, blockBumper_height;
int low_H = 5, high_H = 15, low_S = 140, low_V = 140;

void img_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    Mat frame = cv_ptr->image;

    cv::Mat hsv_frame, orange_found;
    Mat frame_cut = frame;
    cutEnvironment(frame_cut);
    cv::cvtColor(frame_cut, hsv_frame, cv::COLOR_BGR2HSV);

    cv::inRange(hsv_frame, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, 255, 255), orange_found);

    Mat orange_found_rot;
    cv::rotate(orange_found, orange_found_rot, cv::ROTATE_90_CLOCKWISE);

    Mat nonZeroCoordinates;
    findNonZero(orange_found_rot, nonZeroCoordinates);
    Mat bottom_edges(orange_found_rot.size(),CV_8UC1,cv::Scalar::all(0));
    for (int i = 0, last = 0; i < nonZeroCoordinates.total(); i++) {
        int x = nonZeroCoordinates.at<Point>(i).x;
        int y = nonZeroCoordinates.at<Point>(i).y;
        if(y != last) {
            bottom_edges.at<uchar>(y,x) = 255;
            last = y;
        }
    }
    cv::rotate(bottom_edges, bottom_edges, cv::ROTATE_90_COUNTERCLOCKWISE);
    dilate(bottom_edges, bottom_edges, kernel(2,2));

    Mat green_lines = overlayBinaryGreen(frame, bottom_edges);

//	publish Images
    publishMessage(pub, bottom_edges, "mono8");
    publishMessage(pub1, green_lines, "bgr8");
    publishMessage(pub2, orange_found, "mono8");
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "coneDetection");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string subscription_node;
//    nhp.param("perfect_lines_min_cut", perfect_lines_min_cut, 200);
//    nhp.param("Laplacian_threshold", Laplacian_threshold, 2);


    nhp.param("blockSky_height", blockSky_height, 220);
    nhp.param("blockWheels_height", blockWheels_height, 200);
    nhp.param("blockBumper_height", blockBumper_height, 200);

    nhp.param("subscription_node", subscription_node, std::string("/camera/image_color_rect"));

    pub = nh.advertise<sensor_msgs::Image>("/cone_detection_img", 1); //test publish of image
    pub1 = nh.advertise<sensor_msgs::Image>("/cone_Bottom_Overlay", 1);
    pub2 = nh.advertise<sensor_msgs::Image>("/cone_orange_found", 1);
    auto img_real = nh.subscribe(subscription_node, 1, img_callback);

    ros::spin();
    return 0;
}


cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT,cv::Size(x,y));
}

void cutEnvironment(cv::Mat img) {
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


void publishMessage(ros::Publisher pub, Mat img, std::string img_type) {
    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = img;
        cv_ptr->encoding = img_type;
        cv_ptr->toImageMsg(outmsg);
        pub.publish(outmsg);
    }
}

Mat overlayBinaryGreen(Mat frame, Mat binary) {
    Mat green(frame.rows,frame.cols,CV_8UC3,cv::Scalar::all(3));
    Mat green_edges, weight;

    green = cv::Scalar(0,255,0);
    green.copyTo(green_edges, binary);
    addWeighted(frame, .7, green_edges, 1, 0.0, weight);

    return weight;
}
