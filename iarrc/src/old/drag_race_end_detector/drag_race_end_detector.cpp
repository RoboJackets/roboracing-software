#define DEBUG true
#define FORWARDCHECK_DIST 1.5 // meters
#define LINE_WIDTH_METERS (0.0254 * 2.5 * 3) // finish line
#define FORWARDCHECK_WIDTH 1.0 // meters
#define FORWARDCHECK_HEIGHT 1.0 // meters

#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <std_msgs/Bool.h>
#include <iarrc/image_utils.hpp>
#include <iarrc/constants.hpp>
#include <vector>
#include <cmath>

using namespace std;
using namespace cv;
using namespace image_utils;

ros::Publisher finishline_pub;
sensor_msgs::Image rosimage;
std_msgs::Bool stop;

sensor_msgs::Image CvMatToRosImage(cv::Mat& img, std::string encoding) {
	cv_bridge::CvImage cv_img;
	sensor_msgs::Image ros_img;
	cv_img.image=img;
    cv_img.encoding=encoding;
    cv_img.toImageMsg(ros_img);
    return ros_img;
}

// ROS image callback
void ImageSaverCB(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

	// Convert ROS to OpenCV
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

    Mat image = cv_ptr->image.clone();
    Mat smallimage;
    const float line_width_pixels = LINE_WIDTH_METERS * constants::pixels_per_meter;
    const float image_scale = 3/line_width_pixels;
    resize(image, smallimage, Size(image.cols*image_scale, image.rows*image_scale), 0, 0, INTER_LANCZOS4);
    static const Rect forwardRect(
            image_scale*(image.cols/2 - constants::pixels_per_meter * FORWARDCHECK_WIDTH/2),
            image_scale*(image.rows - constants::pixels_per_meter * (FORWARDCHECK_HEIGHT + FORWARDCHECK_DIST)),
            image_scale*(constants::pixels_per_meter * FORWARDCHECK_WIDTH),
            image_scale*(constants::pixels_per_meter * FORWARDCHECK_HEIGHT)
    );

if(DEBUG) {
    Mat debug;
    namedWindow("Debug");
    smallimage.copyTo(debug);
    rectangle(debug, forwardRect, Scalar(255, 0, 255));
    imshow("Debug", debug);
    waitKey(100);
}
//    cerr << "forwardrect: " << forwardrect << endl;

//    cerr << "smallimage height" << smallimage.rows << endl;
//    cerr << "smallimage width" << smallimage.rows << endl;

    Mat forwardCheck(smallimage, forwardRect);

//    cerr << "Made ROIs" << endl;

    Mat finImage = 255 * Mat::ones(smallimage.size(), CV_8UC1);

    LINES foundLines;
    foundLines = hasLines(forwardCheck);
    if(foundLines == LINES_WHITE || foundLines == LINES_BOTH)
	stop.data = true;

    finishline_pub.publish(stop);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "iarrc_finishline");
	ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // FIXME: Not expected behavior
    if(argc >= 2) {
	    //cerr << "unexpected arguments" << endl;//help(std::cerr);
	    exit(1);
    }

    std::string img_topic;
    nhp.param(std::string("img_topic"), img_topic, std::string("/image_projected"));

    ROS_INFO("Image topic:= %s", img_topic.c_str());

    // Subscribe to ROS topic with callback
    ros::Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageSaverCB);
    finishline_pub = nh.advertise<std_msgs::Bool>("/finishline", 1);
    stop.data = false;

	ROS_INFO("IARRC finishline detection node ready.");

	ros::spin();
	ROS_INFO("Shutting down IARRC finishline detection node.");
    return 0;
}

