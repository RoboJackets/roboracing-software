#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include <rr_platform/speed.h>
#include <rr_platform/steering.h>

using namespace std;

typedef vector<cv::Point> Contour;
typedef cv::Vec4f Line;
typedef vector<vector<cv::Point>> Contours;

cv_bridge::CvImagePtr cv_ptr;
ros::Publisher image_pub;
ros::Publisher speed_pub;
ros::Publisher steer_pub;

rr_platform::speed speed_message;
rr_platform::steering steer_message;

double speed;
double kP;
double left_angle_offset;
double right_angle_offset;


cv::Mat mergeImagesSideBySide(cv::Mat leftImg, cv::Mat rightImg) {
    int outputRows = std::max(leftImg.rows, rightImg.rows);
    int outputCols = leftImg.cols + rightImg.cols;
    cv::Mat merged(outputRows, outputCols, leftImg.type(), cv::Scalar(0,0,0));
    cv::Rect leftHalf(0, 0, leftImg.cols, leftImg.rows);
    cv::Rect rightHalf(leftImg.cols, 0, rightImg.cols, rightImg.rows);
    leftImg.copyTo(merged(leftHalf));
    rightImg.copyTo(merged(rightHalf));

    return merged;
}

std::vector<cv::Point> findLaneContour(const cv::Mat& img) {
    Contours contours;
    Contour biggestContour;
    cv::findContours(img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    double maxArea = 10;
    for (Contour cnt : contours) {
        double currArea = cv::contourArea(cnt, false);
        if (currArea > maxArea) {
            biggestContour = cnt;
            maxArea = currArea;
        }
    }
    return biggestContour;
}

float get_pnt_on_line(int y, const Line& line) {
    float Vx = line[0];
    float Vy = line[1];
    float x0 = line[2];
    float y0 = line[3];
    float x  = 0;

    x = (Vx / Vy) * (y - y0) + x0;
    return x;
}

double findDistance(Contour contour, int y, int x_anchor) {
    Line line;
    int distance = -1;

    if (!contour.empty()) {
        cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);

        float Vx = line[0];
        float Vy = line[1];
        float x0 = line[2];
        float y0 = line[3];

        float x = (Vx / Vy) * (y - y0) + x0;
        distance = x - x_anchor;
    }
    return distance;
}

void publishMessage(const ros::Publisher pub, const cv::Mat& img, std::string img_type) {
    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = img;
        cv_ptr->encoding = img_type;
        cv_ptr->toImageMsg(outmsg);
        pub.publish(outmsg);
    }
}



/**
 * Reads in an image from two side cameras, determines the relative
 * position and angle of the car to the lane lines, and
 * attempts to stay in the middle of the lane.
 * This is done by fitting a line to seen lane blobs,
 * then trying to keep an equal distance to each lane boundary.
 * This does not account for our current angle relative to lines
 * in steering.
 *
 * @param leftMsg image input from left side camera
 * @param rightMsg image input from the right side camera
 */
void img_callback(const sensor_msgs::ImageConstPtr& leftMsg, const sensor_msgs::ImageConstPtr& rightMsg) {
    //Convert msg to Mat image
    cv_ptr = cv_bridge::toCvCopy(leftMsg, "mono8");
    cv::Mat leftFrame = cv_ptr->image;
    cv_ptr = cv_bridge::toCvCopy(rightMsg, "mono8");
    cv::Mat rightFrame = cv_ptr->image;

    cv::rotate(leftFrame, leftFrame, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::rotate(rightFrame, rightFrame, cv::ROTATE_90_CLOCKWISE);

    cv::Mat debug = mergeImagesSideBySide(leftFrame, rightFrame);

    int resize_dim = 300;
    int original_height = debug.rows;
    int original_width = debug.cols;
    cv::resize(debug, debug, cv::Size(resize_dim * original_width / original_height, resize_dim));

//    const double angleOffset = CV_PI / 2; //since it is 90 degrees rotated
//    Contour leftContour = findLaneContour(leftFrame);
//    Contour rightContour = findLaneContour(rightFrame);
//
//    double leftLaneDist  = findDistance(leftContour,  leftFrame.rows / 2,  leftFrame.cols);
//    double rightLaneDist = findDistance(rightContour, rightFrame.rows/ 2, 0);
//
//    double error = std::abs(distToLeft - distToRight); //goal is them to be equal dist, that is centered
//
//    double errorTolerance = 4.0; //allowable angle difference
//
//    //Try to be centered
//    double steering;
//    if (std::isnan(error)) {
//        steering = 0.0;
//    } else {
//        steering = error * kP; //- angleOffset) ) * kP;
//    }
//
//    auto now = ros::Time::now();
//
//    speed_message.speed = speed;
//    speed_message.header.stamp = now;
//
//    steer_message.angle = steering;
//    steer_message.header.stamp = now;
//
//    speed_pub.publish(speed_message);
//    steer_pub.publish(steer_message);
//
//    //debugging stuff


//    cv::Mat debug = make_debug_img(leftFrame, rightFrame, leftContour, rightContour);
//
//    cv::putText(debug, "Steer: " + to_string(steering), cv::Point(20,250), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,0), 2);

    publishMessage(image_pub, debug, "mono8");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "urc_center_keeper");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string leftCamera_sub_name;
    std::string rightCamera_sub_name;
    nhp.param("camera_left_subscription", leftCamera_sub_name, std::string("/camera_left/image_color_rect"));
    nhp.param("camera_right_subscription", rightCamera_sub_name, std::string("/camera_right/image_color_rect"));
    nhp.param("speed", speed, 1.0);
    nhp.param("PID_kP", kP, -0.01);
    nhp.param("left_angle_offset", left_angle_offset, 0.0);
    nhp.param("right_angle_offset", right_angle_offset, 0.0);


    image_pub = nh.advertise<sensor_msgs::Image>("/urc_side_lanes", 1); //publish debug image
//    speed_pub = nh.advertise<rr_platform::speed>("plan/speed", 1);
//    steer_pub = nh.advertise<rr_platform::steering>("plan/steering", 1);


    message_filters::Subscriber<sensor_msgs::Image> leftCamera_sub(nh, leftCamera_sub_name, 1);
    message_filters::Subscriber<sensor_msgs::Image> rightCamera_sub(nh, rightCamera_sub_name, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10) //#TODO: change?
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(3), leftCamera_sub, rightCamera_sub);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));

    ros::spin();
    return 0;
}
