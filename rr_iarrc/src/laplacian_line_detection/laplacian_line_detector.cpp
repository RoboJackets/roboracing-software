#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

void blockEnvironment(const cv::Mat&);
cv::Mat getAdaptiveThres(const cv::Mat&);
cv::Mat getIgnoreColorMask(const cv::Mat&);
cv::Mat getBlurredGrayImage(const cv::Mat&);
cv::Mat kernel(int, int);
cv::Mat floorfillAreas(const cv::Mat&, const cv::Mat&);
cv::Mat cutSmall(const cv::Mat&, int);
void publishMessage(ros::Publisher&, const cv::Mat&, std::string, ros::Time&);
cv::Mat overlayBinaryGreen(cv::Mat&, const cv::Mat&);
cv::Mat removeAngels(const cv::Mat& img, int distanceFromEarth);
cv::Mat createDebugImage(cv::Mat&, const cv::Mat&, const cv::Mat&, const cv::Mat&, const cv::Mat&);

cv_bridge::CvImagePtr cv_ptr;
int original_width, original_height;
int blockSky_height, blockWheels_height, blockBumper_height;
int ignore_color_low_H, ignore_color_high_H, ignore_color_low_S, ignore_color_high_S, ignore_color_low_V, ignore_color_high_V;
bool ignore_adaptive;
int min_blob_area, laplacian_threshold_min, laplacian_threshold_max, adaptive_mean_threshold;
ros::Publisher pub_line_detector, pub_debug_img;
int resize_dim = 400;

/**
 * Performs Adaptive Threshold to find areas where we are certain there are lines then
 * those areas are floodfilled on a Laplacian that has more noise but the entirety of the line.
 *
 * @param msg image input from camera
 */
void img_callback(const sensor_msgs::ImageConstPtr& msg) {
    auto time_stamp = msg->header.stamp;

    //Convert msg to Mat image
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    original_height = frame.rows;
    original_width = frame.cols;
    cv::resize(frame, frame, cv::Size(resize_dim, resize_dim));

    cv::Mat ignore_color_mask = getIgnoreColorMask(frame);
    cv::Mat frame_gray = getBlurredGrayImage(frame);

    cv::Mat lapl, adaptive, true_lines, floodfill_blobs;
    cv::Laplacian(frame_gray, lapl, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);
    inRange(lapl, laplacian_threshold_min, laplacian_threshold_max, lapl);
    blockEnvironment(lapl);
    lapl.setTo(cv::Scalar(0, 0, 0), ignore_color_mask);

    if (!ignore_adaptive) {
        adaptive = getAdaptiveThres(frame_gray);
        floodfill_blobs = cutSmall(adaptive, min_blob_area);

        cv::Mat fill = floorfillAreas(lapl, floodfill_blobs);
        true_lines = cutSmall(fill, min_blob_area);
    } else {
        cv::Mat lightness;
        threshold(frame_gray, lightness, 5, 255, 0);
        lapl.setTo(cv::Scalar(0, 0, 0), lightness == 0);

        cv::erode(lapl, lapl, kernel(3,1));
        true_lines = cutSmall(lapl, min_blob_area);

        cv::Mat black(frame.rows,frame.cols,CV_8UC1,cv::Scalar::all(0));
        adaptive = black;
        floodfill_blobs = black;
    }

    cv::Mat img_debug = createDebugImage(frame_gray, adaptive, lapl, floodfill_blobs, ignore_color_mask);
    cv::resize(true_lines, true_lines, cv::Size(original_width, original_height));

    publishMessage(pub_line_detector, true_lines, "mono8", time_stamp);
    publishMessage(pub_debug_img, img_debug, "bgr8", time_stamp);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Laplacian");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string subscription_node, detection_node, debug_node;
    nhp.param("laplacian_threshold_min", laplacian_threshold_min, -255);
    nhp.param("laplacian_threshold_max", laplacian_threshold_max, -20);
    nhp.param("min_blob_area", min_blob_area, 60);

    nhp.param("blockSky_height",    blockSky_height, 0);
    nhp.param("blockWheels_height", blockWheels_height, 800);
    nhp.param("blockBumper_height", blockBumper_height, 800);

    nhp.param("ignore_adaptive", ignore_adaptive, false);
    nhp.param("adaptive_mean_threshold", adaptive_mean_threshold, 1);

    nhp.param("ignore_color_low_H",  ignore_color_low_H,  -1);
    nhp.param("ignore_color_high_H", ignore_color_high_H, -1);
    nhp.param("ignore_color_low_S",  ignore_color_low_S,  -1);
    nhp.param("ignore_color_high_S", ignore_color_high_S, -1);
    nhp.param("ignore_color_low_V",  ignore_color_low_V,  -1);
    nhp.param("ignore_color_high_V", ignore_color_high_V, -1);

    nhp.param("subscription_node", subscription_node, std::string("/camera_center/image_color_rect"));

    auto img_real = nh.subscribe(subscription_node, 1, img_callback);

    pub_line_detector = nh.advertise<sensor_msgs::Image>("lines/detection_img", 1); //test publish of image
    pub_debug_img = nh.advertise<sensor_msgs::Image>("lines/debug_img", 1);

    ros::spin();
    return 0;
}

cv::Mat getIgnoreColorMask(const cv::Mat& frame) {
    cv::Mat hsv_frame, ignore_color_mask;
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_frame, cv::Scalar(ignore_color_low_H, ignore_color_low_S, ignore_color_low_V),
                cv::Scalar(ignore_color_high_H, ignore_color_high_S, ignore_color_high_V), ignore_color_mask);
    cv::erode(ignore_color_mask, ignore_color_mask, kernel(2,2));
    return ignore_color_mask;
}

cv::Mat getBlurredGrayImage(const cv::Mat& frame) {
    cv::Mat frame_gray, frame_blur;
    cv::GaussianBlur(frame, frame_blur, cv::Size(5,5), 0);
    cv::cvtColor(frame_blur, frame_gray, cv::COLOR_BGR2GRAY);
    return frame_gray;
}

cv::Mat getAdaptiveThres(const cv::Mat& frame_gray) {
    cv::Mat thres;
    cv::adaptiveThreshold(frame_gray, thres, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 5, -adaptive_mean_threshold);
    blockEnvironment(thres);
    cv::erode(thres, thres, kernel(3,1));
    return thres;
}

cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT,cv::Size(x,y));
}

cv::Mat floorfillAreas(const cv::Mat& lines, const cv::Mat& color_found) {
    cv::Mat color_left, lines_found(lines.rows,lines.cols,CV_8UC1,cv::Scalar::all(0));
    cv::Mat lines_remaining = lines.clone();
    lines.copyTo(color_left, color_found);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(color_left, contours,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    for(int i = 0; i < contours.size(); i++ ) {
        cv::floodFill(lines_remaining, contours[i][0], cv::Scalar(0));
    }
    cv::bitwise_xor(lines, lines_remaining, lines_found);
    return lines_found;
}

void blockEnvironment(const cv::Mat& img) {
    cv::rectangle(img,
                  cv::Point(0,0),
                  cv::Point(img.cols, blockSky_height * resize_dim / original_height),
                  cv::Scalar(0, 0, 0),CV_FILLED);

    cv::rectangle(img,
                  cv::Point(0,img.rows),
                  cv::Point(img.cols, blockWheels_height * resize_dim / original_height),
                  cv::Scalar(0),CV_FILLED);

    cv::rectangle(img,
                  cv::Point(img.cols/3,img.rows),
                  cv::Point(2 * img.cols / 3, blockBumper_height * resize_dim / original_height),
                  cv::Scalar(0),CV_FILLED);
}

cv::Mat cutSmall(const cv::Mat& color_edges, int size_min) {
    cv::Mat contours_color(color_edges.rows,color_edges.cols,CV_8UC1,cv::Scalar::all(0));
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(color_edges, contours,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    for( int i = 0; i < contours.size(); i++ ) {
        if (size_min < cv::contourArea(contours[i], false) ) {
            cv::drawContours(contours_color, contours, i, cv::Scalar(255), CV_FILLED, 8);
        }
    }
    return contours_color;
}

void publishMessage(ros::Publisher& pub, const cv::Mat& img, std::string img_type, ros::Time& time_stamp) {
    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = img;
        cv_ptr->encoding = img_type;
        cv_ptr->toImageMsg(outmsg);
        outmsg.header.stamp = time_stamp;
        pub.publish(outmsg);
    }
}

cv::Mat overlayBinaryGreen(cv::Mat& frame, const cv::Mat& binary) {
    return frame.setTo(cv::Scalar(0,255,0), binary != 0);
}

cv::Mat removeAngels(const cv::Mat& img, int distanceFromEarth) {
    //Removes anything that extends from the top of the image to the bottom like glare from the sun
    cv::Mat top = img.clone();
    cv::rectangle(top,
                  cv::Point(0,img.rows / 3 + blockSky_height - distanceFromEarth),
                  cv::Point(img.cols,img.rows),
                  cv::Scalar(0),CV_FILLED);

    std::vector<cv::Point> locations;
    cv::findNonZero(top, locations);
    int number_of_angles = 20;
    for (int i = 0; i < number_of_angles; ++i) {
        cv::floodFill(img, locations[i], cv::Scalar(0));
        cv::floodFill(top, locations[i], cv::Scalar(0));
        cv::findNonZero(top, locations);
    }
    return img;
}

cv::Mat createDebugImage(cv::Mat &img_debug, const cv::Mat &adaptive, const cv::Mat &lapl,
                         const cv::Mat &cut, const cv::Mat& ignore_color) {
    cv::cvtColor(img_debug, img_debug, CV_GRAY2BGR);

    // Highlight ROI
    double alpha = .8;
    cv::Mat img_ROI(img_debug.rows, img_debug.cols, CV_8UC3, cv::Scalar::all(255));
    cv::Mat img_ROI_binary, floodfill_pnt;
    blockEnvironment(img_ROI);
    cv::cvtColor(img_ROI, img_ROI_binary, CV_BGR2GRAY);
    img_ROI.setTo(cv::Scalar(0, 255, 0), img_ROI_binary == 255);
    cv::addWeighted(img_debug, alpha, img_ROI, 1 - alpha, 0.0, img_debug);

    //Add highlighted section colors
    img_debug.setTo(cv::Scalar(0, 0, 130), adaptive != 0);
    img_debug.setTo(cv::Scalar(130, 0, 50), lapl != 0);
    img_debug.setTo(cv::Scalar(204, 0, 204), cut != 0);
    cv::bitwise_and(cut, lapl, floodfill_pnt);
    img_debug.setTo(cv::Scalar(0, 255, 0), floodfill_pnt != 0);
    img_debug.setTo(cv::Scalar(0, 255, 255), ignore_color != 0);

    //Add Text
    cv::putText(img_debug, "Area Visible", cv::Point(5,20), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(0, 200, 0), 1);
    cv::putText(img_debug, "Adaptive", cv::Point(5,40), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(0, 0, 130), 1);
    cv::putText(img_debug, "Adaptive (Big Enough)", cv::Point(5,60), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(204, 0, 204), 1);
    cv::putText(img_debug, "Laplacian", cv::Point(5,80), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(130, 0, 50), 1);
    cv::putText(img_debug, "Flood Points (Adpt. && Lapl.)", cv::Point(5,100), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(0, 255, 0), 1);
    cv::putText(img_debug, "Color Being Ignored", cv::Point(5,120), cv::FONT_HERSHEY_DUPLEX, .7, cv::Scalar(0, 255, 255), 1);

    return img_debug;
}
