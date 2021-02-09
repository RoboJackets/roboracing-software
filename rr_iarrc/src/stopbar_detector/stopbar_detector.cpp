#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <rr_msgs/urc_sign.h>
#include <sensor_msgs/Image.h>
#include <stdlib.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

cv_bridge::CvImagePtr cv_ptrLine;
ros::Publisher pubLine;
ros::Publisher pubMove;

rr_msgs::urc_sign moveMsg;

double stopBarGoalAngle;
double stopBarGoalAngleRange;
double stopBarTriggerDistance;
double stopBarTriggerDistanceRight;
int houghThreshold;
double houghMinLineLength;
double houghMaxLineGap;
int pixels_per_meter;

const std::string RIGHT("right");
const std::string LEFT("left");
const std::string STRAIGHT("straight");
const std::string NONE("none");

struct ArrowSign {
    std::string direction;
    int area;
    int count;
};

std::vector<ArrowSign> arrowTrackList{ { RIGHT, 0, 0 }, { LEFT, 0, 0 }, { STRAIGHT, 0, 0 } };
ArrowSign bestMove = { NONE, 0, 0 };

/*
 * Uses probablistic Hough to find line segments and determine if they are the
 * stop bar An angle close to 0 is horizontal.
 *
 * @param frame The input overhead image to search inside
 * @param output debug image
 * @param stopBarGoalAngle The angle of line relative to horizontal that makes a
 * stop bar
 * @param stopBarGoalAngleRange Allowable error around stopBarGoalAngle
 * @param triggerDistance Distance to the line that we will send out the message
 * to take action
 * @param threshold HoughLinesP threshold that determines # of votes that make a
 * line
 * @param minLineLength HoughLinesP minimum length of a line segment
 * @param maxLineGap HoughLinesP maxmimum distance between points in the same
 * line
 */
bool findStopBarFromHough(cv::Mat& frame, cv::Mat& output, double& stopBarAngle, double stopBarGoalAngle,
                         double stopBarGoalAngleRange, double triggerDistance, double triggerDistanceRight,
                         int threshold, double minLineLength, double maxLineGap) {
    cv::Mat edges;
    int ddepth = CV_8UC1;
    cv::Laplacian(frame, edges, ddepth);  // use edge to get better Hough results
    convertScaleAbs(edges, edges);
    edges = frame;                                    //#TOOD: probably laplace then dilate.
    cv::cvtColor(edges, output, cv::COLOR_GRAY2BGR);  // for debugging

    // Standard Hough Line Transform
    std::vector<cv::Vec4i> lines;  // will hold the results of the detection
    double rho = 1;                // distance resolution
    double theta = CV_PI / 180;    // angular resolution (in radians) pi/180 is one degree res

    cv::HoughLinesP(edges, lines, rho, theta, threshold, minLineLength,
                    maxLineGap);  // Like hough but for line segments
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        cv::Point p1(l[0], l[1]);
        cv::Point p2(l[2], l[3]);
        cv::line(output, p1, p2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

        // calc angle and decide if it is a stop bar
        double distanceX = p2.x - p1.x;
        double distanceY = p2.y - p1.y;
        double currAngle = atan(fabs(distanceY / distanceX)) * 180 / CV_PI;  // in degrees

        cv::Point midpoint = (p1 + p2) * 0.5;

        if (fabs(stopBarGoalAngle - currAngle) <= stopBarGoalAngleRange) {  // allows some amount of angle error
            // get distance to the line
            float dist = (edges.rows - midpoint.y) / pixels_per_meter;

            if (dist <= triggerDistance) {
                cv::Point midpoint = (p1 + p2) * 0.5;
                cv::circle(output, midpoint, 3, cv::Scalar(255, 0, 0), -1);
                std::stringstream streamAngle;
                streamAngle << std::fixed << std::setprecision(2) << currAngle;  // show angle with a couple decimals
                cv::putText(output, streamAngle.str(), midpoint, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 1);

                stopBarAngle = currAngle;
                return true;  // stop bar detected close to us!
            }
        }
    }

    return 0;  // not close enough or no stop bar here
}

void stopBar_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptrLine = cv_bridge::toCvCopy(msg, "mono8");
    cv::Mat frame = cv_ptrLine->image;
    cv::Mat debug;
    double stopBarAngle;
    bool stopBarDetected = findStopBarFromHough(frame, debug, stopBarAngle, stopBarGoalAngle, stopBarGoalAngleRange,
                                               stopBarTriggerDistance, stopBarTriggerDistanceRight, houghThreshold,
                                               houghMinLineLength, houghMaxLineGap);

    // debugging draw a line where we trigger
    cv::Point leftTriggerPoint(0, debug.rows - 1 - stopBarTriggerDistance * pixels_per_meter);
    cv::Point rightTriggerPoint(debug.cols - 1, debug.rows - 1 - stopBarTriggerDistance * pixels_per_meter);
    cv::line(debug, leftTriggerPoint, rightTriggerPoint, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    cv::Point leftTriggerPoint2(0, debug.rows - 1 - stopBarTriggerDistanceRight * pixels_per_meter);
    cv::Point rightTriggerPoint2(debug.cols - 1, debug.rows - 1 - stopBarTriggerDistanceRight * pixels_per_meter);
    cv::line(debug, leftTriggerPoint2, rightTriggerPoint2, cv::Scalar(0, 150, 0), 1, cv::LINE_AA);

    if (stopBarDetected) {
        // let the world know
        moveMsg.header.stamp = ros::Time::now();
        moveMsg.direction = bestMove.direction;
        moveMsg.angle = stopBarAngle;
        pubMove.publish(moveMsg);

        // reset things
        bestMove.direction = NONE;
        bestMove.area = 0.0;
    }
    if (pubLine.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptrLine->image = debug;
        cv_ptrLine->encoding = "bgr8";
        cv_ptrLine->toImageMsg(outmsg);
        pubLine.publish(outmsg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sign_detector");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string overhead_image_sub;
    std::string sign_file_path_from_package;
    std::string sign_file_package_name;

    // stop bar detector params
    nhp.param("overhead_image_subscription", overhead_image_sub, std::string("/lines/detection_img_transformed"));
    nhp.param("stopBarGoalAngle", stopBarGoalAngle, 0.0);  // angle in degrees
    nhp.param("stopBarGoalAngleRange", stopBarGoalAngleRange,
              15.0);  // angle in degrees
    nhp.param("stopBarTriggerDistance", stopBarTriggerDistance,
              0.5);  // distance in meters
    nhp.param("stopBarTriggerDistanceRight", stopBarTriggerDistanceRight,
              0.5);  // distance in meters
    nhp.param("pixels_per_meter", pixels_per_meter, 100);
    nhp.param("houghThreshold", houghThreshold, 50);
    nhp.param("houghMinLineLength", houghMinLineLength, 0.0);
    nhp.param("houghMaxLineGap", houghMaxLineGap, 0.0);

    pubLine = nh.advertise<sensor_msgs::Image>("/stopbar_detector/stop_bar",
                                               1);  // debug publish of image
    // publish the turn move for Urban Challenge Controller
    pubMove = nh.advertise<rr_msgs::urc_sign>("/turn_detected", 1);
    auto stopBar = nh.subscribe(overhead_image_sub, 1, stopBar_callback);

    ros::spin();
    return 0;
}