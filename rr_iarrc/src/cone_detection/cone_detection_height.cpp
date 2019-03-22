#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/Float64MultiArray.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/math/constants/constants.hpp>
#include <algorithm>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sensor_msgs/CameraInfo.h>

using namespace cv;
using namespace std;
using namespace ros;

cv::Mat kernel(int, int);
Mat cutEnvironment(cv::Mat);
void publishMessage(ros::Publisher, Mat, std::string);
cv::Mat getCenter(cv::Mat, double);
cv::Mat cutBodies(cv::Mat, Mat);
cv::Mat doWatershed(Mat, cv::Mat, Mat);
cv::Mat drawAndCalc(cv::Mat, Mat);
void drawCircle(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointXYZ point, double radius, int numPoints);
void fovCallback(const sensor_msgs::CameraInfoConstPtr& msg);
void loadCameraFOV(NodeHandle& nh);
cv::Mat cutSmall(cv::Mat, int);

cv_bridge::CvImagePtr cv_ptr;
ros::Publisher pub, pub1, pub2;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
int blockSky_height, blockWheels_height, blockBumper_height;
int low_H, high_H, low_S, low_V;
int canny_cut_min_threshold, minimum_area_cut;
double percent_max_distance_transform;

double fy, real_cone_height, real_cone_radius;
double camera_fov_horizontal;  // radians
Size imageSize;
bool fov_callback_called;
double angle_constant;
vector<pcl::PointXYZ> cone_points;

void img_callback(const sensor_msgs::ImageConstPtr& msg) {
    //Convert msg to Mat image
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    Mat frame = cv_ptr->image;

    ros::Time begin = ros::Time::now();

    //Record original dimensions and resize
    int originalHeight = frame.rows;
    int originalWidth = frame.cols;
    cv::resize(frame, frame, cv::Size(500, 500));

    //Get ROI of frame and HSV color threshold
    cv::Mat hsv_frame, orange_found, frame_cut;
    frame_cut = cutEnvironment(frame.clone());
    cv::cvtColor(frame_cut, hsv_frame, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_frame, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, 255, 255), orange_found);
    orange_found = cutSmall(orange_found, minimum_area_cut);

    //Cut HSV to find individual cones
    Mat detected_bodies = cutBodies(frame_cut, orange_found);
    detected_bodies = cutSmall(detected_bodies, minimum_area_cut);
    Mat sure_bodies = doWatershed(frame, orange_found, detected_bodies);
    sure_bodies = cutSmall(sure_bodies, minimum_area_cut);

    //Find Distance
    frame = drawAndCalc(frame, sure_bodies);
    ros::Time end = ros::Time::now();

//    cerr<< 1/((end - begin).toSec()) << endl;

//	publish Images
    publishMessage(pub, frame, "bgr8");
    publishMessage(pub1, detected_bodies, "mono8");

    pcl::PointCloud<pcl::PointXYZ> cone_cloud;
    for (int i = 0; i < cone_points.size(); i++) {
        drawCircle(cone_cloud, cone_points[i], real_cone_radius, 20);
    }
    sensor_msgs::PointCloud2 outmsg;
    pcl::toROSMsg(cone_cloud, outmsg);
    outmsg.header.frame_id = "base_footprint";
    pub2.publish(outmsg);
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

    nhp.param("canny_cut_min_threshold", canny_cut_min_threshold, 40);
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
    pub = nh.advertise<sensor_msgs::Image>("/cones/height/debug_coordinates", 1); //test publish of image
    pub1 = nh.advertise<sensor_msgs::Image>("/cones/height/separated_bodies", 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("/cones/height/pointcloud", 1); //test publish of image
    auto img_real = nh.subscribe(subscription_node, 1, img_callback);

    ros::spin();
    return 0;
}


cv::Mat cutSmall(cv::Mat color_edges, int size_min) {
    cv::Mat contours_color(color_edges.rows,color_edges.cols,CV_8UC1,cv::Scalar::all(0));
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(color_edges, contours,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    for( int i = 0; i < contours.size(); i++ ) {
        if (size_min < cv::arcLength(contours[i], false) ) {
            cv::drawContours(contours_color, contours, i, cv::Scalar(255), CV_FILLED, 8);
        }
    }
    return contours_color;
}


cv::Mat cutBodies(cv::Mat frame_cut, cv::Mat orange_found) {
    Mat frame_gray, detected_edges, detected_bodies;

    GaussianBlur(frame_cut, frame_cut, Size(3,3), 0, 0, BORDER_DEFAULT );
    cv::cvtColor(frame_cut, frame_gray, cv::COLOR_BGR2GRAY );
    bitwise_and(frame_gray, orange_found, frame_gray);

    cv::Canny(frame_gray, detected_edges, canny_cut_min_threshold, canny_cut_min_threshold*3, 3);
    dilate(detected_edges, detected_edges,kernel(2,2));

    floodFill(detected_edges, Point(0,0), cv::Scalar(255));

    bitwise_not(detected_edges, detected_bodies);
    morphologyEx(detected_bodies, detected_bodies, cv::MORPH_OPEN, kernel(3,3));

    return detected_bodies;
}

cv::Mat getCenter(cv::Mat img, double thres_value) {
    Mat dist_transform, thres, dist_mask, mask;
    Mat sure_front = Mat::zeros(img.size(), CV_8UC1);
    vector<vector<Point>> contours1;
    double min, max;

    distanceTransform(img, dist_transform, CV_DIST_L2, 3);
    findContours(img, contours1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    for( int i = 0; i < contours1.size(); i++ ) {
        mask = Mat::zeros(img.size(), CV_8UC1);
        dist_mask = Scalar::all(0);

        drawContours(mask, contours1, i, 255, CV_FILLED);

        dist_transform.copyTo(dist_mask, mask);

        cv::minMaxLoc(dist_mask, &min, &max);

        threshold(dist_mask, thres, max * thres_value, 255, CV_THRESH_BINARY);
        thres.convertTo(thres, CV_8UC1);
        bitwise_or(thres, sure_front, sure_front);
    }

    return sure_front;
}

cv::Mat doWatershed(cv::Mat frame,cv::Mat orange_found, cv::Mat detected_bodies) {
    Mat sure_back, dist_transform, unknown, markers;

    Mat sure_front = getCenter(detected_bodies, percent_max_distance_transform);
    sure_front.convertTo(sure_front, CV_8UC1);

    dilate(orange_found, sure_back, kernel(3,3));

    subtract(sure_back, sure_front, unknown);

    connectedComponents(sure_front, markers);
    markers += 1;
    markers.setTo(0, unknown == 255);

    watershed(frame, markers);

    Mat lines = Mat::zeros(orange_found.size(), CV_8UC1);
    lines.setTo(Scalar(255), markers == -1);
    rectangle(lines, Point(0,0), Point(frame.cols, frame.rows), Scalar(0), 2);

    dilate(lines, lines, kernel(2,1));
    floodFill(lines, Point(0,0), cv::Scalar(255));

    Mat sure_body;
    bitwise_not(lines, sure_body);

    return sure_body;
}

cv::Mat drawAndCalc(cv::Mat frame, cv::Mat sure_bodies) {
    vector<vector<Point> > contours;
    findContours(sure_bodies, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    vector<Rect> rect( contours.size() );
    std_msgs::Float64MultiArray list;
    cone_points.clear();

    for( int i = 0; i < contours.size(); i++ ) {

        rect[i] = boundingRect(contours[i]);

        if (rect[i].height < 10 || rect[i].width < 5) {
            continue;
        }

        Moments m = moments(contours[i], true);
        Point center = Point(m.m10 / m.m00, m.m01 / m.m00);

        int offset = 0;
        double distance = (real_cone_height * fy) / rect[i].height;
        int px_horz_dist = frame.cols/2 - center.x;
        double horz_dist = distance * px_horz_dist / fy;
        double horz_dist2 = distance * tan(px_horz_dist * angle_constant);

        cone_points.push_back(pcl::PointXYZ(distance, horz_dist, 0));

        line(frame, Point(frame.cols/2 +offset,0), Point(frame.cols/2+offset, frame.rows), (0,255,0), 2);
//        printf("Pixels: (%d, %d) Real: (%.2f, %.2f)\n", rect[i].height, px_horz_dist, distance, horz_dist);
        string str = "("+std::to_string(((int)(distance))) + ", " + std::to_string((int)horz_dist)+")";

        stringstream stream;
        stream << fixed << setprecision(1) << distance;
        string s = "("+stream.str();

        stringstream stream1;
        stream1 << fixed << setprecision(1) << horz_dist;
        s += ", " + stream1.str() +")";

        Scalar color =  Scalar(0, 255);
        drawContours( frame, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        rectangle(frame, rect[i].tl(), rect[i].br(), color, 3, 8, 0);
        circle(frame, center, 5,  Scalar(255,0,255), CV_FILLED, 8, 0);
        putText(frame,s, Point(center.x-80,center.y-30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2,true);
    }

    return frame;
}

cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT,cv::Size(x,y));
}

Mat cutEnvironment(cv::Mat img) {
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
    return img;
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

void drawCircle(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointXYZ point, double radius, int numPoints) {
    const double pi = boost::math::constants::pi<double>();
    double angleStep = (2.0 * pi) / numPoints;

    double angle = 0; //start angle

    for (int i = 0; i < numPoints; i++) {
        //draw a circle
        double x = radius * cos(angle);
        double y = radius * sin(angle);

        cloud.push_back(pcl::PointXYZ(point.x + x, point.y + y , 0)); //point.x + radius is center of circle
        angle = angle + angleStep;

    }
}

void fovCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    double fx = msg->P[0]; //horizontal focal length of rectified image, in px
//    fy = msg->P[5]; //vertical focal length
    imageSize = Size(msg->width, msg->height);
    camera_fov_horizontal = 2 * atan2(imageSize.width, 2*fx);
    fov_callback_called = true;
    angle_constant = camera_fov_horizontal / 2.0 /imageSize.width;
}

void loadCameraFOV(NodeHandle& nh) {
    auto infoSub = nh.subscribe("/camera/camera_info", 1, fovCallback);

    fov_callback_called = false;
    while (!fov_callback_called) {
        spinOnce();
        Duration(1).sleep();
    }
    ROS_INFO("Using horizontal FOV %f ", camera_fov_horizontal);
}
