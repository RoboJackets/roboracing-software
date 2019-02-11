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
ros::Publisher pub, pub1, pub2, pub3;

cv::Mat kernel(int, int);
cv::Mat fillColorLines(cv::Mat, cv::Mat);
void cutEnvironment(cv::Mat);
cv::Mat cutSmall(cv::Mat, int);
void publishMessage(ros::Publisher, Mat, std::string);
Mat overlayBinaryGreen(Mat, Mat);

int blockSky_height, blockWheels_height, blockBumper_height;
int perfect_lines_min_cut, Laplacian_threshold;


void img_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    Mat frame = cv_ptr->image;

    //Image Height Debugger
    //cutEnvironment(frame);

    cv::Mat frame_gray, frame_blur, detected_edges;
    GaussianBlur(frame, frame_blur, Size(5,5), 0, 0, BORDER_DEFAULT);
    cv::cvtColor(frame_blur, frame_gray, cv::COLOR_BGR2GRAY);

    Mat thres;
    adaptiveThreshold(frame_gray, thres, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 5, -1);
    cutEnvironment(thres);
    Mat cut = cutSmall(thres, perfect_lines_min_cut);   //Make sure only Lines

    Mat lapl;
    Laplacian(frame_gray, lapl, CV_16S, 3, 1, 0, BORDER_DEFAULT);
    lapl = min(lapl, 0);
    convertScaleAbs(lapl, lapl);
    threshold(lapl, lapl, Laplacian_threshold, 255, 0);

    Mat fill = fillColorLines(lapl, cut);
    fill = cutSmall(fill, perfect_lines_min_cut);

    Mat green_lines = overlayBinaryGreen(frame, fill);

//	publish images
    publishMessage(pub, fill, "mono8");
    publishMessage(pub1, cut, "mono8");
    publishMessage(pub2, lapl, "mono8");
    publishMessage(pub3, green_lines, "bgr8");
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "Laplacian");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string subscription_node;
    nhp.param("perfect_lines_min_cut", perfect_lines_min_cut, 200);
    nhp.param("Laplacian_threshold", Laplacian_threshold, 2);

    nhp.param("blockSky_height", blockSky_height, 220);
    nhp.param("blockWheels_height", blockWheels_height, 200);
    nhp.param("blockBumper_height", blockBumper_height, 200);

    nhp.param("subscription_node", subscription_node, std::string("/camera/image_color_rect"));

    pub = nh.advertise<sensor_msgs::Image>("/Combined_lines", 1); //test publish of image
    pub1 = nh.advertise<sensor_msgs::Image>("/Adpt_Thres_lines", 1);
    pub2 = nh.advertise<sensor_msgs::Image>("/Laplacian_lines", 1);
    pub3 = nh.advertise<sensor_msgs::Image>("/Colored_combined_lines", 1);
    auto img_real = nh.subscribe(subscription_node, 1, img_callback);

    ros::spin();
    return 0;
}


cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT,cv::Size(x,y));
}

cv::Mat fillColorLines(cv::Mat lines, cv::Mat color_found) {
    cv::Mat color_left, lines_found(lines.rows,lines.cols,CV_8UC1,cv::Scalar::all(0));
    Mat lines_remaining = lines.clone();
    std::vector<cv::Point> locations;

    lines.copyTo(color_left, color_found);
    cv::findNonZero(color_left, locations);
    while(!locations.empty()) {
        floodFill(lines_remaining, locations[0], cv::Scalar(0));
        bitwise_xor(lines, lines_remaining, lines_found);
        bitwise_and(lines_remaining, color_left, color_left);
        cv::findNonZero(color_left, locations);
    }
    return lines_found;
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

cv::Mat cutSmall(cv::Mat color_edges, int size_min) {
    cv::Mat contours_color(color_edges.rows,color_edges.cols,CV_8UC1,cv::Scalar::all(0));
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(color_edges, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    for( int i = 0; i < contours.size(); i++ ) {
        if (size_min < cv::arcLength(contours[i], false) ) {
            cv::drawContours(contours_color, contours, i, cv::Scalar(255), CV_FILLED, 8, hierarchy);
        }
    }
    return contours_color;
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
