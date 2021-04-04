#include <angles/angles.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <rr_msgs/speed.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

cv_bridge::CvImagePtr cv_ptrLine;
ros::Publisher pub_line;
ros::Publisher pub_near_stopbar;
ros::Publisher pub_angle;

std_msgs::Bool stop_bar_near;
std_msgs::Float64 stop_bar_angle;

double stopBarGoalAngle;
double stopBarGoalAngleRange;
double stopBarTriggerDistance;
int houghThreshold;
double houghMinLineLength;
double houghMaxLineGap;
int pixels_per_meter;
double sleepConstant;

const double rho = 1;              // distance resolution
const double theta = CV_PI / 180;  // angular resolution (in radians) pi/180 is one degree res

double speed;

cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT, cv::Size(x, y));
}

/*
 * Uses probablistic Hough to find line segments and determine if they are the
 * stop bar An angle close to 0 is horizontal.
 *
 * @param frame The input overhead image to search inside
 * @param debug The debug image
 */
bool findStopBarFromHough(cv::Mat& frame, cv::Mat& debug, double& stopBarAngle) {
    cv::Mat edges;
    int ddepth = CV_8UC1;
    cv::Laplacian(frame, edges, ddepth);             // use edge to get better Hough results
    cv::dilate(frame, frame, kernel(4, 4));          // clearer debug image and slightly better detection
    cv::cvtColor(frame, debug, cv::COLOR_GRAY2BGR);  // for debugging

    // Standard Hough Line Transform
    std::vector<cv::Vec4i> lines;  // will hold the results of the detection

    cv::HoughLinesP(frame, lines, rho, theta, houghThreshold, houghMinLineLength,
                    houghMaxLineGap);  // Like hough but for line segments
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        cv::Point p1(l[0], l[1]);
        cv::Point p2(l[2], l[3]);
        cv::line(debug, p1, p2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

        // calc angle and decide if it is a stop bar
        double distanceX = p2.x - p1.x;
        double distanceY = p2.y - p1.y;
        double currAngle = fabs(atan(distanceY / distanceX));  // in radians
        cv::Point midpoint = (p1 + p2) * 0.5;
        double goalAngleRad = stopBarGoalAngle * CV_PI / 180;
        double angleDiff =
              fabs(angles::shortest_angular_distance(stopBarGoalAngle, currAngle) * 180 / CV_PI);  // in degrees
        if (angleDiff <= stopBarGoalAngleRange) {  // allows some amount of angle error
            // get distance to the line
            float dist = static_cast<float>(edges.rows - midpoint.y) / pixels_per_meter;

            if (dist <= stopBarTriggerDistance) {
                // places circle in the center of the line and displays angle of line in debug image
                cv::circle(debug, midpoint, 3, cv::Scalar(255, 0, 0), -1);
                std::stringstream streamAngle;
                streamAngle << std::fixed << std::setprecision(2)
                            << (currAngle * 180 / CV_PI);  // show angle with a couple decimals
                cv::putText(debug, streamAngle.str(), midpoint, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 1);

                // draw line to stopbar in debug image and displays the distance in meters to it
                cv::line(debug, midpoint, cv::Point(midpoint.x, edges.rows), cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
                std::stringstream streamDist;
                streamDist << std::fixed << std::setprecision(2) << dist;  // show distance with a couple decimals
                cv::putText(debug, streamDist.str(), cv::Point(midpoint.x, edges.rows - dist / 2),
                            cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 1);

                stopBarAngle = currAngle;
                if (speed != 0) {
                    double timeToStopBar = (dist / speed) * sleepConstant;
                    ros::Duration(timeToStopBar).sleep();
                }
                return true;  // stop bar detected close to us!
            }
        }
    }
    return false;  // not close enough or no stop bar here
}

void stopBar_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptrLine = cv_bridge::toCvCopy(msg, "mono8");
    cv::Mat frame = cv_ptrLine->image;
    cv::Mat debug;
    double stopBarAngle;
    bool stopBarDetected = findStopBarFromHough(frame, debug, stopBarAngle);

    // debugging draw a line where we trigger
    cv::Point leftPoint(0, debug.rows - 1 - stopBarTriggerDistance * pixels_per_meter);
    cv::Point rightPoint(debug.cols - 1, debug.rows - 1 - stopBarTriggerDistance * pixels_per_meter);
    cv::line(debug, leftPoint, rightPoint, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    stop_bar_near.data = stopBarDetected;
    if (stopBarDetected) {
        stop_bar_angle.data = stopBarAngle;
        pub_angle.publish(stop_bar_angle);
    }
    pub_near_stopbar.publish(stop_bar_near);
    if (pub_line.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptrLine->image = debug;
        cv_ptrLine->encoding = "bgr8";
        cv_ptrLine->toImageMsg(outmsg);
        pub_line.publish(outmsg);
    }
}

void speed_callback(const rr_msgs::speed& speedMsg) {
    speed = speedMsg.speed;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sign_detector");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    std::string overhead_image_sub;
    std::string stopbar_near_topic;
    std::string stopbar_angle_topic;
    std::string speed_topic;

    nh.param("stopbar_near", stopbar_near_topic, std::string("/stopbar_near"));
    nh.param("stopbar_angle", stopbar_angle_topic, std::string("/stopbar_angle"));
    nhp.param("sleep_adjust", sleepConstant, 1.0);

    nhp.param("overhead_image_subscription", overhead_image_sub, std::string("/lines/detection_img_transformed"));

    nh.param("speed_subscription", speed_topic, std::string("/speed"));

    nhp.param("stopBarGoalAngle", stopBarGoalAngle, 0.0);              // angle in degrees
    nhp.param("stopBarGoalAngleRange", stopBarGoalAngleRange, 15.0);   // angle in degrees
    nhp.param("stopBarTriggerDistance", stopBarTriggerDistance, 0.5);  // distance in meters
    nhp.param("pixels_per_meter", pixels_per_meter, 100);
    nhp.param("houghThreshold", houghThreshold, 50);
    nhp.param("houghMinLineLength", houghMinLineLength, 0.0);
    nhp.param("houghMaxLineGap", houghMaxLineGap, 0.0);

    pub_line = nh.advertise<sensor_msgs::Image>("/stopbar_detector/stop_bar",
                                                1);  // debug publish of image
    pub_near_stopbar = nh.advertise<std_msgs::Bool>(stopbar_near_topic, 1);
    pub_angle = nh.advertise<std_msgs::Float64>(stopbar_angle_topic, 1);
    auto speed_sub = nh.subscribe(speed_topic, 1, speed_callback);
    auto stopBar = nh.subscribe(overhead_image_sub, 1, stopBar_callback);

    ros::spin();
    return 0;
}