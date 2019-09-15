#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <rr_iarrc/urc_sign.h>
#include <sensor_msgs/Image.h>
#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImagePtr cv_ptrLine;
ros::Publisher pub;
ros::Publisher pubLine;
ros::Publisher pubMove;

rr_iarrc::urc_sign moveMsg;

int roi_x;
int roi_y;
int roi_width;
int roi_height;

cv::Mat sign_forward;
cv::Mat sign_left;
cv::Mat sign_right;

std::vector<cv::Point> template_contour_upright;
double minContourArea;
double maxContourArea;
double straightMatchSimilarityThreshold;
double turnMatchSimilarityThreshold;

double cannyThresholdLow;
double cannyThresholdHigh;

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
int signTriggerCount;

/*
 * This sign dectector uses contours to classified shapes
 * as arrows and determine the direction.
 *
 * A sign must be seen multiple frames IN A ROW to count
 */
void sign_callback(const sensor_msgs::ImageConstPtr& msg) {
    ros::Time start = ros::Time::now();
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    cv::Mat crop;
    if (roi_x == -1 || roi_y == -1 || roi_width == -1 || roi_height == -1) {
        crop = frame;
    } else {
        cv::Rect roi(roi_x, roi_y, roi_width, roi_height);
        crop = frame(roi);  //@Note: crop is just a reference to that roi of the
                            // frame, not a copy
    }

    // cv::GaussianBlur(crop, crop, cv::Size(5,5), 0, 0, cv::BORDER_DEFAULT);
    // //may or may not help Canny

    // Color -> binary; Edge detection
    cv::Mat edges;
    cv::Canny(crop, edges, cannyThresholdLow, cannyThresholdHigh);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(edges, edges, cv::MORPH_CLOSE,
                     kernel);  // connect possible broken lines

    // Find arrow-like shapes with Contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    cv::drawContours(crop, contours, -1, cv::Scalar(0, 255, 0), 2);  // debug

    std::vector<ArrowSign> foundArrowsList;
    for (size_t i = 0; i < contours.size(); i++) {
        std::vector<cv::Point> c = contours[i];  // current contour reference
        double perimeter = cv::arcLength(c, true);
        double epsilon = 0.04 * perimeter;
        std::vector<cv::Point> approxC;
        cv::approxPolyDP(c, approxC, epsilon, true);
        //@note: if need be, you can allow size range 6-9 because it is possible one
        // edge looks like 2 depending on epsilon
        double contourArea = cv::contourArea(approxC);
        if (approxC.size() == 7 && contourArea >= minContourArea && contourArea <= maxContourArea) {
            // check the ratio is arrow-like
            cv::Rect rect = cv::boundingRect(c);
            double ratioMin = 1.2;  // 1.5; //ratio of width to height or vice versa
            double ratioMax = 2.5;  // 2.2; //expanded from real to allow us to view it off angle
            if ((rect.width * ratioMin <= rect.height && rect.width * ratioMax >= rect.height) ||
                (rect.height * ratioMin <= rect.width && rect.height * ratioMax >= rect.width)) {
                cv::rectangle(crop, rect, cv::Scalar(0, 255, 255), 3);  // debug

                if (rect.width > rect.height) {
                    // Sideways
                    double matchSimilarity = cv::matchShapes(c, template_contour_upright, CV_CONTOURS_MATCH_I1,
                                                             0);  //#TODO: change to sideways contours
                    cv::putText(crop, std::to_string(matchSimilarity), cv::Point(rect.x, rect.y + 25),
                                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0), 2);

                    if (matchSimilarity <= turnMatchSimilarityThreshold) {
                        // find top point of the arrow and test its x location
                        auto extremeY = std::minmax_element(
                              c.begin(), c.end(), [](cv::Point const& a, cv::Point const& b) { return a.y < b.y; });
                        if (extremeY.first->x < rect.x + rect.width / 2) {  // topmost point is far left
                            // left pointing arrow!
                            cv::putText(crop, LEFT, rect.tl(), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
                            ArrowSign move = { LEFT, rect.area(), 0 };
                            foundArrowsList.push_back(move);
                        } else {
                            // right pointing arrow!
                            cv::putText(crop, RIGHT, rect.tl(), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
                            ArrowSign move = { RIGHT, rect.area(), 0 };
                            foundArrowsList.push_back(move);
                        }
                    }
                } else {
                    // Straight
                    double matchSimilarity =
                          cv::matchShapes(approxC, template_contour_upright, CV_CONTOURS_MATCH_I1, 0.0);

                    cv::putText(crop, std::to_string(matchSimilarity), cv::Point(rect.x, rect.y + 50),
                                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0), 2);

                    if (matchSimilarity <= straightMatchSimilarityThreshold) {
                        cv::putText(crop, STRAIGHT, rect.tl(), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
                        ArrowSign move = { STRAIGHT, rect.area(), 0 };
                        foundArrowsList.push_back(move);
                    }
                }
            }
        }
    }

    // consistency checks
    for (int i = 0; i < arrowTrackList.size(); i++) {
        bool wasSeen = false;
        for (int j = 0; j < foundArrowsList.size(); j++) {
            ArrowSign currArrow = foundArrowsList[j];
            if (arrowTrackList[i].direction == currArrow.direction) {
                wasSeen = true;
                arrowTrackList[i].count += 1;
                if (arrowTrackList[i].area > currArrow.area) {  // update rect size
                    arrowTrackList[i].area = currArrow.area;
                }
                int areaTolerance = 150;
                if (arrowTrackList[i].count >= signTriggerCount &&
                    arrowTrackList[i].area + areaTolerance >= bestMove.area) {
                    bestMove.direction = arrowTrackList[i].direction;
                    bestMove.area = arrowTrackList[i].area;
                }
            }
        }
        if (!wasSeen) {
            arrowTrackList[i].count = 0;
            arrowTrackList[i].area = 0.0;
        }
    }

    // Some debug images for us
    // show the bestMatchRect
    cv::putText(frame, bestMove.direction, cv::Point(0, frame.rows - 1), cv::FONT_HERSHEY_PLAIN, 3,
                cv::Scalar(255, 0, 255), 2);

    // show where we are cropped to
    cv::rectangle(crop, cv::Point(0, 0), cv::Point(crop.cols - 1, crop.rows - 1), cv::Scalar(255, 0, 255), 2, 8, 0);

    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = frame;
        cv_ptr->encoding = "bgr8";
        cv_ptr->toImageMsg(outmsg);
        pub.publish(outmsg);
    }
}

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
int findStopBarFromHough(cv::Mat& frame, cv::Mat& output, double& stopBarAngle, double stopBarGoalAngle,
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
        cv::line(output, p1, p2, cv::Scalar(0, 0, 255), 2, CV_AA);

        // calc angle and decide if it is a stop bar
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double currAngle = atan(std::fabs(dy / dx)) * 180 / CV_PI;  // in degrees

        cv::Point midpoint = (p1 + p2) * 0.5;
        cv::circle(output, midpoint, 3, cv::Scalar(255, 0, 0), -1);
        std::stringstream streamAngle;
        streamAngle << std::fixed << std::setprecision(2) << currAngle;  // show angle with a couple decimals
        cv::putText(output, streamAngle.str(), midpoint, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 1);

        if (fabs(stopBarGoalAngle - currAngle) <= stopBarGoalAngleRange) {  // allows some amount of angle error
            // get distance to the line
            float dist = static_cast<float>((edges.rows - midpoint.y)) / pixels_per_meter;

            cv::line(output, midpoint, cv::Point(midpoint.x, edges.rows), cv::Scalar(0, 255, 255), 1, CV_AA);
            std::stringstream streamDist;
            streamDist << std::fixed << std::setprecision(2) << dist;  // show distance with a couple decimals
            cv::putText(output, streamDist.str(), cv::Point(midpoint.x, edges.rows - dist / 2), cv::FONT_HERSHEY_PLAIN,
                        1, cv::Scalar(0, 255, 0), 1);

            if (bestMove.direction == RIGHT && dist <= triggerDistanceRight) {
                stopBarAngle = currAngle;
                return 2;
            } else if (dist <= triggerDistance) {
                stopBarAngle = currAngle;
                return 1;  // stop bar detected close to us!
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
    int stopBarDetected = findStopBarFromHough(frame, debug, stopBarAngle, stopBarGoalAngle, stopBarGoalAngleRange,
                                               stopBarTriggerDistance, stopBarTriggerDistanceRight, houghThreshold,
                                               houghMinLineLength, houghMaxLineGap);

    // debugging draw a line where we trigger
    cv::Point leftTriggerPoint(0, debug.rows - 1 - stopBarTriggerDistance * pixels_per_meter);
    cv::Point rightTriggerPoint(debug.cols - 1, debug.rows - 1 - stopBarTriggerDistance * pixels_per_meter);
    cv::line(debug, leftTriggerPoint, rightTriggerPoint, cv::Scalar(0, 255, 0), 1, CV_AA);

    cv::Point leftTriggerPoint2(0, debug.rows - 1 - stopBarTriggerDistanceRight * pixels_per_meter);
    cv::Point rightTriggerPoint2(debug.cols - 1, debug.rows - 1 - stopBarTriggerDistanceRight * pixels_per_meter);
    cv::line(debug, leftTriggerPoint2, rightTriggerPoint2, cv::Scalar(0, 150, 0), 1, CV_AA);

    if (stopBarDetected == 2 && bestMove.direction != NONE) {
        // let the world know
        moveMsg.header.stamp = ros::Time::now();
        moveMsg.direction = bestMove.direction;
        moveMsg.angle = stopBarAngle;
        pubMove.publish(moveMsg);

        // reset things
        bestMove.direction = NONE;
        bestMove.area = 0.0;
    } else if (stopBarDetected == 1 && bestMove.direction != NONE) {
        // let the world know
        moveMsg.header.stamp = ros::Time::now();
        moveMsg.direction = bestMove.direction;
        moveMsg.angle = stopBarAngle;
        pubMove.publish(moveMsg);

        // reset things
        bestMove.direction = NONE;
        bestMove.area = 0.0;
    } else {
    }

    if (pubLine.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptrLine->image = debug;
        cv_ptrLine->encoding = "bgr8";
        cv_ptrLine->toImageMsg(outmsg);
        pubLine.publish(outmsg);
    }
}

// loads images, scales as need be, and makes the rotated versions
void loadSignImages(std::string packageName, std::string fileName) {
    std::string path = ros::package::getPath(packageName);
    sign_forward = cv::imread(path + fileName);
    ROS_ERROR_STREAM_COND(sign_forward.empty(), "Arrow sign image not found at " << path + fileName);

    cv::Mat arrowEdges;
    cv::cvtColor(sign_forward, sign_forward, CV_RGB2GRAY);
    cv::resize(sign_forward, sign_forward,
               cv::Size(sign_forward.rows / 6,
                        sign_forward.cols / 6));  // scale image down to make contour similar scale
    cv::Canny(sign_forward, arrowEdges, 100,
              100 * 3);  // get the edges as a binary, thresholds don't matter here

    //#TODO: LOAD THE ROTATED CONTOURS?
    std::vector<std::vector<cv::Point>> cnts;
    std::vector<cv::Vec4i> h;
    cv::findContours(arrowEdges, cnts, h, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    template_contour_upright = cnts[0];

    cv::Point2f center(sign_forward.rows / 2, sign_forward.cols / 2);
    cv::Mat rotateLeft = cv::getRotationMatrix2D(center, 90, 1);
    cv::warpAffine(sign_forward, sign_left, rotateLeft, cv::Size(sign_forward.cols, sign_forward.rows));
    cv::flip(sign_left, sign_right, 1);  // 1 = horizontal flip
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sign_detector");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string image_sub;
    std::string overhead_image_sub;
    std::string sign_file_path_from_package;
    std::string sign_file_package_name;

    // sign detector params
    nhp.param("roi_x", roi_x, 0);
    nhp.param("roi_y", roi_y, 0);
    nhp.param("roi_width", roi_width, -1);  //-1 will default to the whole image
    nhp.param("roi_height", roi_height, -1);
    nhp.param("front_image_subscription", image_sub, std::string("/camera/image_color_rect"));
    nhp.param("sign_file_package_name", sign_file_package_name, std::string("rr_iarrc"));
    nhp.param("sign_file_path_from_package", sign_file_path_from_package,
              std::string("/src/sign_detector/sign_forward.jpg"));
    nhp.param("minimum_contour_area", minContourArea, 300.0);
    nhp.param("maxmimum_contour_area", maxContourArea, 35000.0);
    nhp.param("required_match_count", signTriggerCount, 1);

    nhp.param("straight_match_similarity_threshold", straightMatchSimilarityThreshold, 0.3);
    nhp.param("turn_match_similarity_threshold", turnMatchSimilarityThreshold, 0.3);
    nhp.param("canny_threshold_low", cannyThresholdLow, 100.0);
    nhp.param("canny_threshold_high", cannyThresholdHigh, 100.0 * 3);

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

    loadSignImages(sign_file_package_name, sign_file_path_from_package);

    pub = nh.advertise<sensor_msgs::Image>("/sign_detector/signs",
                                           1);  // debug publish of image
    pubLine = nh.advertise<sensor_msgs::Image>("/sign_detector/stop_bar",
                                               1);  // debug publish of image
    pubMove = nh.advertise<rr_iarrc::urc_sign>("/turn_detected",
                                               1);  // publish the turn move for Urban Challenge Controller
    auto img_real = nh.subscribe(image_sub, 1, sign_callback);
    auto stopBar = nh.subscribe(overhead_image_sub, 1, stopBar_callback);

    ros::spin();
    return 0;
}
