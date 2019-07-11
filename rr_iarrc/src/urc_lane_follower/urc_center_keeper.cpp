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
    int outputRows = leftImg.rows + rightImg.rows;
    int outputCols = leftImg.cols + rightImg.cols;
    cv::Mat merged(outputRows, outputCols, leftImg.type(), cv::Scalar(0,0,0));
    cv::Rect leftHalf(0, 0, leftImg.cols, leftImg.rows);
    cv::Rect rightHalf(leftImg.cols, 0, rightImg.cols, rightImg.rows);
    leftImg.copyTo(merged(leftHalf));
    rightImg.copyTo(merged(rightHalf));

    return merged;
}

//Outputs angle in radians
double calcLineAngle(cv::Mat &binaryImg, cv::Mat &colorImg) {
    cv::Mat edges;
    int ddepth = CV_8UC1;
    cv::Laplacian(binaryImg, edges, ddepth); //use edges to get better Hough results
    convertScaleAbs( edges, edges );

    // Hough Line Transform
    std::vector<cv::Vec4i> lines; // will hold the results of the detection
    double rho = 1; //distance resolution
    double theta = CV_PI/180; //angular resolution (in radians) pi/180 is one degree res
    int threshold = 10;
    double minLineLength = 0;
    double maxLineGap = 6;

    cv::HoughLinesP(edges, lines, rho, theta, threshold, minLineLength, maxLineGap );

    //find the trend of line angles, weighted by their length
    double weightedAngle = 0.0;
    double totalWeight = 0.0;
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        cv::Point p1(l[0], l[1]);
        cv::Point p2(l[2], l[3]);

        //debug
        cv::line( colorImg, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);

        //calc angle
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double currAngle = std::atan2(dy, dx); //in radians

        double currLength = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        totalWeight += currLength;
        weightedAngle += currAngle * currLength;
    }
    weightedAngle /= totalWeight; //handle divide by zero (no lines in image)

    return weightedAngle;
}


cv::Mat findLineBinary(cv::Mat image) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    int ddepth = CV_8UC1;
    cv::Laplacian(gray, gray, ddepth); convertScaleAbs( gray, gray );
    int thresh = 2;//25;
    cv::threshold( gray, gray, thresh, 255, cv::THRESH_BINARY );
    //cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(15,15));
    cv::morphologyEx(gray, gray, cv::MORPH_CLOSE, kernel );
    return gray;
}


std::vector<cv::Point> findLaneContour(const cv::Mat& img) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> biggestContour;
    cv::findContours(img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    double maxArea = 0.0;
    for (std::vector<cv::Point> cnt : contours) {
        double currArea = cv::contourArea(cnt, false);
        if (currArea > maxArea) {
            biggestContour = cnt;
            maxArea = currArea;
        }
    }
    return biggestContour;
}

double findDistance(std::vector<cv::Point> contour, int yLocation) {
    cv::Vec4f line;
    cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);

    //find our point on the line.
    //Let's show our math.
    //x = x0 + (vx)t
    //x - x0 = (vx)t
    //(x-x0) / (vx) = t
    double t = yLocation-line[3]/line[1];
    return line[0] + line[2] * t;
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
    cv_ptr = cv_bridge::toCvCopy(leftMsg, "bgr8");
    cv::Mat leftFrame = cv_ptr->image;
    cv_ptr = cv_bridge::toCvCopy(rightMsg, "bgr8");
    cv::Mat rightFrame = cv_ptr->image;

    cv::rotate(leftFrame, leftFrame, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::rotate(rightFrame, rightFrame, cv::ROTATE_90_CLOCKWISE);
    const double angleOffset = CV_PI / 2; //since it is 90 degrees rotated

    cv::Mat leftLine = findLineBinary(leftFrame);
    cv::Mat rightLine = findLineBinary(rightFrame);

    std::vector<cv::Point> leftContour = findLaneContour(leftLine);
    std::vector<cv::Point> rightContour = findLaneContour(rightLine);

    double distToLeft = -findDistance(leftContour, leftFrame.rows/2); //negative
    double distToRight = findDistance(rightContour, rightFrame.rows/2);

    double error = std::abs(distToLeft - distToRight); //goal is them to be equal dist, that is centered

    double errorTolerance = 4.0; //allowable angle difference

    //Try to be centered
    double steering;
    if (std::isnan(error)) {
        steering = 0.0;
    } else {
        steering = error * kP; //- angleOffset) ) * kP;
    }

    auto now = ros::Time::now();

    speed_message.speed = speed;
    speed_message.header.stamp = now;

    steer_message.angle = steering;
    steer_message.header.stamp = now;

    speed_pub.publish(speed_message);
    steer_pub.publish(steer_message);

    //debugging stuff
    cv::Mat debug = mergeImagesSideBySide(leftFrame, rightFrame);

    cv::putText(debug, "Steer: " + to_string(steering), cv::Point(20,250), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,0), 2);


    if (image_pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = debug;
        cv_ptr->encoding = "bgr8";
        cv_ptr->toImageMsg(outmsg);
        image_pub.publish(outmsg);
    }
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


    image_pub = nh.advertise<sensor_msgs::Image>("/urc_lane_follower", 1); //publish debug image
    speed_pub = nh.advertise<rr_platform::speed>("plan/speed", 1);
    steer_pub = nh.advertise<rr_platform::steering>("plan/steering", 1);


    message_filters::Subscriber<sensor_msgs::Image> leftCamera_sub(nh, leftCamera_sub_name, 1);
    message_filters::Subscriber<sensor_msgs::Image> rightCamera_sub(nh, rightCamera_sub_name, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10) //#TODO: change?
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), leftCamera_sub, rightCamera_sub);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));

    ros::spin();
    return 0;
}
