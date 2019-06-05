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

using namespace std;

cv_bridge::CvImagePtr cv_ptr;
ros::Publisher image_pub;

cv::Mat kernel(int x, int y);

cv::Mat mergeImagesSideBySide(cv::Mat leftImg, cv::Mat rightImg) {
    int outputRows = leftImg.rows + rightImg.rows;
    int outputCols = leftImg.cols + rightImg.cols;
    cv::Mat merged(outputRows, outputCols, leftImg.type(), cv::Scalar(0,0,0));
    cv::Rect leftHalf(0, 0, leftImg.cols, leftImg.rows);
    cv::Rect rightHalf(leftImg.cols, 0, rightImg.cols, rightImg.rows);
    leftImg.copyTo(merged(leftHalf));
    rightImg.copyTo(merged(rightHalf));

    return merged;
}


double calcLineAngle(cv::Mat &binaryImg) {
    cv::Mat edges;
    int ddepth = CV_8UC1;
    cv::Laplacian(binaryImg, edges, ddepth); //use edges to get better Hough results
    convertScaleAbs( edges, edges );

    // Standard Hough Line Transform
    std::vector<cv::Vec4i> lines; // will hold the results of the detection
    double rho = 1; //distance resolution
    double theta = CV_PI/180; //angular resolution (in radians) pi/180 is one degree res
    int threshold = 10;
    double minLineLength = 0;
    double maxLineGap = 0;

    cv::HoughLinesP(edges, lines, rho, theta, threshold, minLineLength, maxLineGap );

    //find the trend of line angles, weighted by their length
    return 0;

}



/**
 * Reads in an image from two side cameras, determines the relative
 * position and angle of the car to the lane lines, and
 * attempts to stay in the middle of the lane. Also reports the
 * current angle to the urc dead-reckoning protocol.
 *
 * @param leftMsg image input from left side camera
 * @param rightMsg image input from the right side camera
 */
void img_callback(const sensor_msgs::ImageConstPtr& leftMsg, const sensor_msgs::ImageConstPtr& rightMsg) {
    //Convert msg to Mat image
    cv_ptr = cv_bridge::toCvCopy(leftMsg, "bgr8");
    cv::Mat leftFrame = cv_ptr->image;
    cv_ptr = cv_bridge::toCvCopy(rightMsg, "bgr8");
    cv::Mat rightFrame = cv_ptr->image;


    cv::Mat debug = mergeImagesSideBySide(leftFrame, rightFrame);
    cv::cvtColor(debug, debug, cv::COLOR_GRAY2BGR);

    if (image_pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = debug;
        cv_ptr->encoding = "bgr8";
        cv_ptr->toImageMsg(outmsg);
        image_pub.publish(outmsg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "urc_lane_follower");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string subscription_node;
    //nhp.param("perfect_lines_min_cut", perfect_lines_min_cut, 200);
    //nhp.param("subscription_node", subscription_node, std::string("/camera/image_color_rect"));

    image_pub = nh.advertise<sensor_msgs::Image>("/urc_lane_follower", 1); //publish debug image


    message_filters::Subscriber<sensor_msgs::Image> leftCamera_sub(nh, "/camera_left/color_image_rect", 1);
    message_filters::Subscriber<sensor_msgs::Image> rightCamera_sub(nh, "/camera_right/color_image_rect", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10) //#TODO: change?
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), leftCamera_sub, rightCamera_sub);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));

    ros::spin();
    return 0;
}

cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT,cv::Size(x,y));
}
