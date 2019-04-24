#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#define HIST_FRAMES 5     // Number of frames to keep in history
#define LIGHT_DIST_PIX 35 // Pixels between the center of red and green lights
#define MAXSUMRESULTSRED2GREEN 1000000 // experimental value of the max of the 2d traffic light filter
#define TRIGGERPERCENTAGE 1.0 // The percentage an event must be from experimental, accounting for noise, to trigger
#define LIGHT_SIZE_PIX 35 // The width or height of the observable stoplight light halo

using namespace cv;
using namespace std;

ros::Publisher img_pub;
ros::Publisher bool_pub;
sensor_msgs::Image rosimage;
std_msgs::Bool green;

// ROS image callback

// What happens when the stoplight changes from red to green?
// The stoplight has a red light on from the start
// This light is very bright, and can wash out the image
// It typically looks like a reddish halo around a white circle
// The stoplight then turns on the green light, and both are on
// This looks like two white circles one over the other,
// The top having a red halo, and the bottom a green halo
// This lasts for a few frames, then the red light turns off
void ImageCB(const sensor_msgs::Image::ConstPtr& msg) {
    // Intialize variables
    if(green.data) {
        bool_pub.publish(green);
        return; // do not to all this computing if the signal has already been seen and go broadcast
    }

    cv_bridge::CvImagePtr cv_ptr;
    Mat circlesImg, circlesImgRed, circlesImgGreen;
    static Mat history[HIST_FRAMES];
    static int counttostart = 1;


    // Convert ROS to OpenCV
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV-Bridge error: %s", e.what());
        return;
    }

    Mat frame = cv_ptr.get()->image;


    for(int i = HIST_FRAMES-1; i > 0; i--) {
        history[i] = history[i - 1];
    }
    history[0] = frame;
    if (!(counttostart == HIST_FRAMES)) {
        counttostart++;
        return;
    }

    Mat past, cur;

    past = history[HIST_FRAMES - 1]; // oldest frame in buffer
    cur = history[0]; // newest frame in buffer

    Mat translate = (Mat_<double>(2, 3) << 1 , 0 , 0 , 0 , 1 , LIGHT_DIST_PIX);

    Mat gaining = cur - past;
    Mat losing = past - cur;
    warpAffine(losing, losing, translate, past.size()); // put the red loosing light on the green gaining light

    Mat channel[3];
    split(gaining, channel);
    Mat gaininggreen = channel[0] - channel[2]; // we use blue because the green light is more blue than green
    split(losing, channel);
    Mat losingred = channel[2] - channel[0]/2 - channel[1]/2;

    Mat geometricmean;
    gaininggreen.convertTo(gaininggreen, CV_16UC1, 1, 0);
    losingred.convertTo(losingred, CV_16UC1, 1, 0);
    geometricmean = gaininggreen.mul(losingred);
    Mat centerLightChangeness(gaining.size(), CV_32FC1);
    Mat kernal = Mat::ones(LIGHT_SIZE_PIX, LIGHT_SIZE_PIX, CV_32FC1)/(LIGHT_SIZE_PIX*LIGHT_SIZE_PIX);
    filter2D(geometricmean, centerLightChangeness, CV_32FC1, kernal);

    double minResult, maxResult;
    minMaxLoc(centerLightChangeness, &minResult, &maxResult);
    ROS_INFO("maxResult %f", maxResult);
    if (maxResult > TRIGGERPERCENTAGE * MAXSUMRESULTSRED2GREEN) {
        green.data = true;
    }

    bool_pub.publish(green);

    geometricmean.convertTo(geometricmean, CV_8UC1, 1, 0);
    cv_ptr->image=geometricmean;
    cv_ptr->encoding="mono8";
    cv_ptr->toImageMsg(rosimage);
    img_pub.publish(rosimage);

}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "iarrc_image_display");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    std::string img_topic, stoplight_topic;
    nhp.getParam(string("img_topic"), img_topic);
    nhp.getParam("stoplight_topic", stoplight_topic);

    ROS_INFO("Stoplight watching %s", img_topic.c_str());

    // Subscribe to ROS topic with callback
    ros::Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageCB);
    img_pub = nh.advertise<sensor_msgs::Image>("/image_circles", 1);
    bool_pub = nh.advertise<std_msgs::Bool>(stoplight_topic, 1);

    green.data = false;


    ROS_INFO("IARRC stoplight watcher node ready.");
    ros::spin();
    ROS_INFO("Shutting down IARRC stoplight watcher node.");
    return 0;
}
