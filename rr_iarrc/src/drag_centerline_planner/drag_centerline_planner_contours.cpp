#include <cv_bridge/cv_bridge.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <sensor_msgs/Image.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "PID.h"

using namespace std;
typedef cv::Mat Mat;
typedef cv::Point Point;
typedef vector<Point> Contour;
typedef cv::Vec4f Line;
typedef vector<vector<Point>> Contours;

ros::Publisher pub_line_detector;
ros::Publisher speed_pub;
ros::Publisher steer_pub;

rr_platform::speed speed_message;
rr_platform::steering steer_message;
cv_bridge::CvImagePtr cv_ptr;

int img_height;
int img_width;

int min_contour_area = 50;
int frame_without_two_lines;
Line last_midline;

Point anchor;

// PID IMPLEMENTATION SETUP
double kP, kI, kD;
double setpoint;
double input, maxTurnLimit;
double outputSteering, speedGoal;
PID myPID(&input, &outputSteering, &setpoint, 0.0, 0.0, 0.0, P_ON_E, REVERSE);

cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT, cv::Size(x, y));
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

Point get_bottom_most_pnt(const Contour& cnt) {
    Point extBot =
          *max_element(cnt.begin(), cnt.end(), [](const Point& lhs, const Point& rhs) { return lhs.y < rhs.y; });
    return extBot;
}

auto get_side_contours(const Mat& img) {
    Contours contours;
    Contour right_cnt, left_cnt;
    int max_left_area = 0, max_right_area = 0;
    cv::findContours(img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    for (Contour cnt : contours) {
        int cnt_area = cv::contourArea(cnt, false);
        if (min_contour_area < cnt_area) {
            Point bottom_most_pnt = get_bottom_most_pnt(cnt);
            if (bottom_most_pnt.x < img.cols / 2 && max_left_area < cnt_area) {
                max_left_area = cnt_area;
                left_cnt = cnt;
            } else if (img.cols / 2 < bottom_most_pnt.x && max_right_area < cnt_area) {
                max_right_area = cnt_area;
                right_cnt = cnt;
            }
        }
    }
    return make_tuple(left_cnt, right_cnt);
}

auto get_cnts_reg_line(const Contour& left_cnt, const Contour& right_cnt) {
    Line left_line, right_line;
    if (!left_cnt.empty()) {
        cv::fitLine(left_cnt, left_line, cv::DIST_L2, 0, 0.01, 0.01);
    }
    if (!right_cnt.empty()) {
        cv::fitLine(right_cnt, right_line, cv::DIST_L2, 0, 0.01, 0.01);
    }
    return make_tuple(left_line, right_line);
}

float get_pnt_on_line(int y, const Line& line) {
    float Vx = line[0];
    float Vy = line[1];
    float x0 = line[2];
    float y0 = line[3];
    float x = 0;

    x = (Vx / Vy) * (y - y0) + x0;
    return x;
}

auto get_line_top_bottom_pnts(int top_y, int bottom_y, const Line& line) {
    Point line_top(get_pnt_on_line(top_y, line), top_y);
    Point line_bottom(get_pnt_on_line(bottom_y, line), bottom_y);
    return make_tuple(line_top, line_bottom);
}

Line get_midline(const Line& left_line, const Line& right_line) {
    Line midline;
    Line empty;
    frame_without_two_lines++;
    if (left_line != empty && right_line != empty) {
        auto [left_line_top, left_line_bottom] = get_line_top_bottom_pnts(0, anchor.y, left_line);
        auto [right_line_top, right_line_bottom] = get_line_top_bottom_pnts(0, anchor.y, right_line);

        Point midline_top = (left_line_top + right_line_top) / 2;
        Point midline_bottom = (left_line_bottom + right_line_bottom) / 2;

        vector<Point> midline_endpnts{ midline_top, midline_bottom };
        fitLine(midline_endpnts, midline, cv::DIST_L2, 0, .01, .01);
        frame_without_two_lines = 0;
    } else if (left_line != empty) {
        midline = (Line){ left_line[0], left_line[1], (float)anchor.x, (float)anchor.y };
    } else if (right_line != empty) {
        midline = (Line){ right_line[0], right_line[1], (float)anchor.x, (float)anchor.y };
    } else {
        midline = (Line){ 0.0, 1.0, (float)anchor.x, (float)anchor.y };
    }

    if (frame_without_two_lines != 0 && frame_without_two_lines < 2) {
        midline = last_midline;
    }

    last_midline = midline;
    return midline;
}

Mat make_debug_img(const Mat& frame, const Contour& left_cnt, const Contour& right_cnt, const Line& left_line,
                   const Line& right_line, const Line& midline, double steering) {
    Mat debug_img;
    cv::cvtColor(frame, debug_img, cv::COLOR_GRAY2BGR);
    if (pub_line_detector.getNumSubscribers() > 0) {
        if (!left_cnt.empty()) {
            Point left_bottom_pnt = get_bottom_most_pnt(left_cnt);
            auto [left_line_top, left_line_bottom] = get_line_top_bottom_pnts(0, left_bottom_pnt.y, left_line);

            cv::polylines(debug_img, left_cnt, true, cv::Scalar(200, 0, 0), 2);
            cv::circle(debug_img, left_bottom_pnt, 4, cv::Scalar(255, 100, 0), -1);
            cv::line(debug_img, left_line_bottom, left_line_top, cv::Scalar(150, 255, 0), 2);
        }

        if (!right_cnt.empty()) {
            Point right_bottom_pnt = get_bottom_most_pnt(right_cnt);
            auto [right_line_top, right_line_bottom] = get_line_top_bottom_pnts(0, right_bottom_pnt.y, right_line);

            cv::polylines(debug_img, right_cnt, true, cv::Scalar(0, 0, 200), 2);
            cv::circle(debug_img, right_bottom_pnt, 4, cv::Scalar(0, 100, 255), -1);
            cv::line(debug_img, right_line_bottom, right_line_top, cv::Scalar(0, 255, 150), 2);
        }

        auto [midline_top, midline_bottom] = get_line_top_bottom_pnts(0, anchor.y, midline);
        Point midline_mid(get_pnt_on_line(img_height / 2, midline), img_height / 2);

        cv::line(debug_img, midline_bottom, midline_top, cv::Scalar(226, 43, 138), 2);
        cv::line(debug_img, Point(anchor.x, 0), Point(anchor.x, img_height), cv::Scalar(0, 255, 255), 1);

        cv::line(debug_img, Point(anchor.x, img_height / 2), midline_mid, cv::Scalar(0, 0, 255), 1);

        cv::putText(debug_img, std::to_string(steering), cv::Point(20, 100), cv::FONT_HERSHEY_PLAIN, 1,
                    cv::Scalar(0, 255, 0), 2);
    }
    return debug_img;
}

void img_callback(const sensor_msgs::ImageConstPtr& msg) {
    // Convert msg to Mat image
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    Mat frame = cv_ptr->image;
    ;

    img_height = frame.rows;
    img_width = frame.cols;
    anchor = Point(img_width / 2, img_height - 10);

    auto [left_cnt, right_cnt] = get_side_contours(frame);
    auto [left_line, right_line] = get_cnts_reg_line(left_cnt, right_cnt);
    Line midline = get_midline(left_line, right_line);

    cv::Point goal(get_pnt_on_line(img_height / 2, midline), img_height / 2);
    ;

    input = static_cast<double>(goal.x);
    setpoint = img_width / 2;
    myPID.Compute();
    double steering = outputSteering;

    Mat debug_img = make_debug_img(frame, left_cnt, right_cnt, left_line, right_line, midline, steering);

    speed_message.speed = speedGoal;
    steer_message.angle = steering;
    speed_pub.publish(speed_message);
    steer_pub.publish(steer_message);

    publishMessage(pub_line_detector, debug_img, "bgr8");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "drag_center_lane_planner");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string subscription_node;
    nhp.param("subscription_node", subscription_node, std::string("/camera_center/lines/detection_img_transformed"));

    nhp.param("PID_kP", kP, 0.0001);
    nhp.param("PID_kI", kI, 0.0);
    nhp.param("PID_kD", kD, 0.0);
    nhp.param("speed", speedGoal, 1.0);

    nhp.param("maxTurnLimitRadians", maxTurnLimit, 0.44);

    // setup PID controllers
    myPID.SetTunings(kP, kI, kD);
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-maxTurnLimit, maxTurnLimit);

    auto img_real = nh.subscribe(subscription_node, 1, img_callback);

    pub_line_detector = nh.advertise<sensor_msgs::Image>("/drag_centerline_debug", 1);  // test publish of image
    speed_pub = nh.advertise<rr_platform::speed>("/plan/speed", 1);
    steer_pub = nh.advertise<rr_platform::steering>("/plan/steering", 1);

    ros::spin();
    return 0;
}
