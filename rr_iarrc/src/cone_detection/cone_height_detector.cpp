#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "std_msgs/Float64MultiArray.h"

using namespace std;

// Image Processing
void blockEnvironment(const cv::Mat&);
cv::Mat find_orange_in_ROI(const cv::Mat&);
cv::Mat cutSmall(const cv::Mat&, int);
cv::Mat canny_cut_bodies(const cv::Mat&, const cv::Mat&);
cv::Mat find_unique_centers(const cv::Mat&, double);
cv::Mat perform_watershed(const cv::Mat&, const cv::Mat&, const cv::Mat&);
cv::Mat draw_box_and_calc_dist(const cv::Mat&, const cv::Mat&);
cv::Mat kernel(int, int);
void publishImage(ros::Publisher, cv::Mat, std::string);

ros::Publisher pub_debug_pos, pub_debug_mask, pub_pointcloud, pub_closest_point;
int blockSky_height, blockWheels_height, blockBumper_height;
typedef cv::Mat Mat;
typedef cv::Point Point;
int low_H, high_H, low_S, low_V;
int canny_cut_min_threshold, minimum_area_cut;
double percent_max_distance_transform;

// PointCloud Processing
pcl::PointCloud<pcl::PointXYZ> draw_cone_circles();
void drawCircle(pcl::PointCloud<pcl::PointXYZ>&, pcl::PointXYZ, double, int);
void fovCallback(const sensor_msgs::CameraInfoConstPtr& msg);
void loadCameraFOV(ros::NodeHandle& nh);
void publish_closest_point();

cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
double fy, real_cone_height, real_cone_radius;
double camera_fov_horizontal;
cv::Size imageSize;
bool fov_callback_called;
double angle_constant;
vector<pcl::PointXYZ> cone_points;

void publish_closest_point() {
    if (cone_points.size() != 0) {
        double min = cone_points[0].x;
        for (pcl::PointXYZ cone_point : cone_points) {
            if (cone_point.x < min) {
                min = cone_point.x;
            }
        }
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(min);
        pub_closest_point.publish(msg);
    }
}

void fovCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    double fx = msg->P[0];  // horizontal focal length of rectified image, in px
    //    fy = msg->P[5]; //vertical focal length
    imageSize = cv::Size(msg->width, msg->height);
    camera_fov_horizontal = 2 * atan2(imageSize.width, 2 * fx);
    fov_callback_called = true;
    angle_constant = camera_fov_horizontal / 2.0 / imageSize.width;
}

void loadCameraFOV(ros::NodeHandle& nh) {
    auto infoSub = nh.subscribe("/camera/camera_info", 1, fovCallback);

    fov_callback_called = false;
    while (!fov_callback_called) {
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    ROS_INFO("Using horizontal FOV %f ", camera_fov_horizontal);
}

void publishPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud) {
    sensor_msgs::PointCloud2 outmsg;
    pcl::toROSMsg(cloud, outmsg);
    outmsg.header.frame_id = "base_footprint";
    pub_pointcloud.publish(outmsg);
}

void drawCircle(pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointXYZ point, double radius, int numPoints) {
    const double pi = boost::math::constants::pi<double>();
    double angleStep = (2.0 * pi) / numPoints;

    double angle = 0;  // start angle

    for (int i = 0; i < numPoints; i++) {
        // draw a circle
        double x = radius * cos(angle);
        double y = radius * sin(angle);

        cloud.push_back(pcl::PointXYZ(point.x + x, point.y + y,
                                      0));  // point.x + radius is center of circle
        angle = angle + angleStep;
    }
}

pcl::PointCloud<pcl::PointXYZ> draw_cone_circles() {
    pcl::PointCloud<pcl::PointXYZ> cone_cloud;
    for (pcl::PointXYZ cone_point : cone_points) {
        drawCircle(cone_cloud, cone_point, real_cone_radius, 20);
    }
    return cone_cloud;
}

void publishImage(ros::Publisher pub, Mat img, std::string img_type) {
    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = img;
        cv_ptr->encoding = img_type;
        cv_ptr->toImageMsg(outmsg);
        pub.publish(outmsg);
    }
}

cv::Mat draw_box_and_calc_dist(const cv::Mat& frame, const cv::Mat& sure_bodies) {
    vector<vector<Point>> contours;
    findContours(sure_bodies, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    vector<cv::Rect> rect(contours.size());
    cone_points.clear();

    for (int i = 0; i < contours.size(); i++) {
        rect[i] = boundingRect(contours[i]);
        if (rect[i].height < 10 || rect[i].width < 5 || rect[i].width > 4 * rect[i].height) {
            continue;
        }

        cv::Moments m = moments(contours[i], true);
        Point center = Point(m.m10 / m.m00, m.m01 / m.m00);

        int offset = 0;
        double distance = (real_cone_height * fy) / rect[i].height;
        int px_horz_dist = frame.cols / 2 - center.x;
        double horz_dist = distance * px_horz_dist / fy;

        if (distance > 8) {
            continue;
        }

        cone_points.push_back(pcl::PointXYZ(distance, horz_dist, 0));

        // Black Line Down Middle
        line(frame, Point(frame.cols / 2, 0), Point(frame.cols / 2, frame.rows), (0, 255, 0), 2);

        // Debugging Printing
        // printf("Pixels: (%d, %d) Real: (%.2f, %.2f)\n", rect[i].height,
        // px_horz_dist, distance, horz_dist);

        stringstream stream_dist, stream_horz_dist;
        stream_dist << fixed << setprecision(1) << distance;
        stream_horz_dist << fixed << setprecision(1) << horz_dist;
        string position = "(" + stream_dist.str() + ", " + stream_horz_dist.str() + ")";

        drawContours(frame, contours, i, cv::Scalar(0, 255), 1, 8, vector<cv::Vec4i>(), 0, Point());
        rectangle(frame, rect[i].tl(), rect[i].br(), cv::Scalar(0, 255), 3, 8, 0);
        circle(frame, center, 5, cv::Scalar(255, 0, 255), cv::FILLED, 8, 0);
        putText(frame, position, Point(center.x - 80, center.y - 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                cv::Scalar(255, 255, 255), 2, true);
    }

    return frame;
}

cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT, cv::Size(x, y));
}

cv::Mat find_unique_centers(const cv::Mat& img, double thres_value) {
    Mat dist_transform, thres, dist_mask, mask;
    Mat sure_front = Mat::zeros(img.size(), CV_8UC1);
    vector<vector<Point>> contours1;
    double min, max;

    distanceTransform(img, dist_transform, CV_DIST_L2, 3);
    findContours(img, contours1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours1.size(); i++) {
        mask = Mat::zeros(img.size(), CV_8UC1);
        dist_mask = cv::Scalar::all(0);

        drawContours(mask, contours1, i, 255, cv::FILLED);

        dist_transform.copyTo(dist_mask, mask);

        cv::minMaxLoc(dist_mask, &min, &max);

        threshold(dist_mask, thres, max * thres_value, 255, CV_THRESH_BINARY);
        thres.convertTo(thres, CV_8UC1);
        bitwise_or(thres, sure_front, sure_front);
    }

    return sure_front;
}

cv::Mat perform_watershed(const cv::Mat& frame, const cv::Mat& orange_found, const cv::Mat& detected_bodies) {
    Mat sure_back, dist_transform, unknown, markers;

    Mat sure_front = find_unique_centers(detected_bodies, percent_max_distance_transform);
    sure_front.convertTo(sure_front, CV_8UC1);

    dilate(orange_found, sure_back, kernel(3, 3));

    subtract(sure_back, sure_front, unknown);

    connectedComponents(sure_front, markers);
    markers += 1;
    markers.setTo(0, unknown == 255);

    watershed(frame, markers);

    Mat lines = Mat::zeros(orange_found.size(), CV_8UC1);
    lines.setTo(cv::Scalar(255), markers == -1);
    rectangle(lines, Point(0, 0), Point(frame.cols, frame.rows), cv::Scalar(0), 2);

    dilate(lines, lines, kernel(2, 1));
    floodFill(lines, Point(0, 0), cv::Scalar(255));

    Mat sure_body;
    bitwise_not(lines, sure_body);

    sure_body = cutSmall(sure_body, minimum_area_cut);

    return sure_body;
}

cv::Mat canny_cut_bodies(const cv::Mat& frame, const cv::Mat& orange_found) {
    Mat frame_gray, detected_edges, detected_bodies;

    cv::Mat frame_copy = frame.clone();
    GaussianBlur(frame_copy, frame_copy, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
    cv::cvtColor(frame_copy, frame_gray, cv::COLOR_BGR2GRAY);
    bitwise_and(frame_gray, orange_found, frame_gray);

    cv::Canny(frame_gray, detected_edges, canny_cut_min_threshold, canny_cut_min_threshold * 3, 3);
    dilate(detected_edges, detected_edges, kernel(2, 2));

    floodFill(detected_edges, Point(0, 0), cv::Scalar(255));

    bitwise_not(detected_edges, detected_bodies);
    morphologyEx(detected_bodies, detected_bodies, cv::MORPH_OPEN, kernel(3, 3));
    detected_bodies = cutSmall(detected_bodies, minimum_area_cut);

    return detected_bodies;
}

cv::Mat cutSmall(const cv::Mat& color_edges, int size_min) {
    cv::Mat contours_color(color_edges.rows, color_edges.cols, CV_8UC1, cv::Scalar::all(0));
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(color_edges, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); i++) {
        if (size_min < cv::arcLength(contours[i], false)) {
            cv::drawContours(contours_color, contours, i, cv::Scalar(255), cv::FILLED, 8);
        }
    }
    return contours_color;
}

cv::Mat find_orange_in_ROI(const cv::Mat& img) {
    cv::Mat hsv_frame, orange_found, frame_cut;
    frame_cut = img.clone();
    blockEnvironment(frame_cut);
    cv::cvtColor(frame_cut, hsv_frame, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_frame, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, 255, 255), orange_found);
    orange_found = cutSmall(orange_found, minimum_area_cut);
    return orange_found;
}

void blockEnvironment(const cv::Mat& img) {
    cv::rectangle(img, cv::Point(0, 0), cv::Point(img.cols, blockSky_height), cv::Scalar(0, 0, 0), cv::FILLED);

    cv::rectangle(img, cv::Point(0, img.rows), cv::Point(img.cols, blockWheels_height), cv::Scalar(0, 0, 0), cv::FILLED);

    cv::rectangle(img, cv::Point(img.cols / 3, img.rows), cv::Point(2 * img.cols / 3, blockBumper_height),
                  cv::Scalar(0, 0, 0), cv::FILLED);
}

void img_callback(const sensor_msgs::ImageConstPtr& msg) {
    // Convert msg to Mat image
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    // Find Orange HSV in Region of interest
    cv::Mat orange_found = find_orange_in_ROI(frame);

    // Cut HSV to find individual cones
    //    cv::Mat detected_bodies = canny_cut_bodies(frame, orange_found);
    //    cv::Mat sure_bodies = perform_watershed(frame, orange_found,
    //    detected_bodies);

    // Find Distance
    frame = draw_box_and_calc_dist(frame, orange_found);

    // publish Images
    publishImage(pub_debug_pos, frame, "bgr8");
    publishImage(pub_debug_mask, orange_found, "mono8");

    // Add Circles for cones to PointCloud and Publish
    pcl::PointCloud<pcl::PointXYZ> cone_cloud = draw_cone_circles();
    publishPointCloud(cone_cloud);

    // Publish Closest Point
    publish_closest_point();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "coneDetectionHeight");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string subscription_node;
    nhp.param("orange_low_H", low_H, 5);
    nhp.param("orange_high_H", high_H, 25);
    nhp.param("orange_low_S", low_S, 140);
    nhp.param("orange_low_V", low_V, 140);

    nhp.param("canny_min_threshold", canny_cut_min_threshold, 40);
    nhp.param("percent_max_distance_transform", percent_max_distance_transform, 0.7);
    nhp.param("minimum_area_cut", minimum_area_cut, 40);

    nhp.param("camera_focal_length_y", fy, 611.46);
    nhp.param("real_cone_height", real_cone_height, .2286);
    nhp.param("real_cone_radius", real_cone_radius, .07);

    nhp.param("blockSky_height", blockSky_height, 220);
    nhp.param("blockWheels_height", blockWheels_height, 200);
    nhp.param("blockBumper_height", blockBumper_height, 200);

    nhp.param("subscription_node", subscription_node, std::string("/camera/image_color_rect"));

    loadCameraFOV(nh);
    pub_debug_pos = nh.advertise<sensor_msgs::Image>("/cones/height/debug_coordinates", 1);
    pub_debug_mask = nh.advertise<sensor_msgs::Image>("/cones/height/debug_bodies_mask", 1);
    pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/cones/height/pointcloud", 1);
    pub_closest_point = nh.advertise<std_msgs::Float32>("/cones/height/closest_point", 1);
    auto img_real = nh.subscribe(subscription_node, 1, img_callback);

    ros::spin();
    return 0;
}