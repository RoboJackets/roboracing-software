#define DEBUG true
#define SIDECHECK_DIST 1.75 // meters
#define LINE_WIDTH_METERS (0.0254 * 2.2) // Camera needs calibration, use 2-2.7 to pick up big yellow lines
#define SIDECHECK_ANGLE (M_PI / 30) // radians
#define SIDECHECK_WIDTH 0.75 // meters
#define SIDECHECK_HEIGHT 1.25 // meters
#define FORWARDCHECK_DIST 1.7 // meters
#define FORWARDCHECK_WIDTH 0.5 // meters
#define FORWARDCHECK_HEIGHT 1.25 // meters

#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <iarrc/image_utils.hpp>
#include <iarrc/constants.hpp>
#include <vector>
#include <cmath>

using namespace std;
using namespace cv;
using namespace image_utils;

std::string img_file;
ros::Publisher img_pub;
ros::Publisher debug_pub;
ros::Publisher finishline_pub;
std_msgs::Bool stop;

sensor_msgs::Image rosimage;

void TurnRight(Mat& image);
void TurnLeft(Mat& image);
void SharpRight(Mat& image);
void SharpLeft(Mat& image);

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
    //cerr << "error -1" << endl;
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

    Mat finImage;

for(int size = 1; size >=1; size -= 3) {
    float line_width_pixels = size * LINE_WIDTH_METERS * constants::pixels_per_meter;
    float image_scale = 3/line_width_pixels;
    resize(image, smallimage, Size(image.cols*image_scale, image.rows*image_scale), 0, 0, INTER_LANCZOS4);

    Rect leftRect(
            image_scale*(image.cols/2 - constants::pixels_per_meter * (SIDECHECK_DIST * sin(SIDECHECK_ANGLE) + SIDECHECK_WIDTH)),
            image_scale*(image.rows - constants::pixels_per_meter * (SIDECHECK_DIST * cos(SIDECHECK_ANGLE) + SIDECHECK_HEIGHT)),
            image_scale*(constants::pixels_per_meter * SIDECHECK_WIDTH),
	    image_scale*(constants::pixels_per_meter * SIDECHECK_HEIGHT)
    );
    Rect rightRect(
            image_scale*(image.cols/2 + constants::pixels_per_meter * (SIDECHECK_DIST * sin(SIDECHECK_ANGLE))),
            image_scale*(image.rows - constants::pixels_per_meter * (SIDECHECK_DIST * cos(SIDECHECK_ANGLE) + SIDECHECK_HEIGHT)),
            image_scale*(constants::pixels_per_meter * SIDECHECK_WIDTH),
	    image_scale*(constants::pixels_per_meter * SIDECHECK_HEIGHT)
    );
    Rect forwardRect(
            image_scale*(image.cols/2 - constants::pixels_per_meter * FORWARDCHECK_WIDTH/2),
            image_scale*(image.rows - constants::pixels_per_meter * (FORWARDCHECK_HEIGHT + FORWARDCHECK_DIST)),
            image_scale*(constants::pixels_per_meter * FORWARDCHECK_WIDTH),
            image_scale*(constants::pixels_per_meter * FORWARDCHECK_HEIGHT)
    );

if(DEBUG && size == 1) {
    Mat debug;
    namedWindow("Debug");
    smallimage.copyTo(debug);
    rectangle(debug, rightRect, Scalar(0, 0, 255));
    rectangle(debug, leftRect, Scalar(0, 255, 0));
    rectangle(debug, forwardRect, Scalar(255, 0, 255));
    imshow("Debug", debug);
    waitKey(100);
}
    //cerr << "leftRect: " << leftRect << endl;
    //cerr << "rightRect: " << rightRect << endl;
    //cerr << "forwardrect: " << forwardRect << endl;

    //cerr << "smallimage height" << smallimage.rows << endl;
    //cerr << "smallimage width" << smallimage.rows << endl;

    Mat leftCheck(smallimage, leftRect);
    Mat rightCheck(smallimage, rightRect);
    Mat forwardCheck(smallimage, forwardRect);

    //cerr << "Made ROIs" << endl;

    finImage = 255 * Mat::ones(smallimage.size(), CV_8UC1);

    LINES leftLines, forwardLines, rightLines;
    //cerr << "lC.rows" << leftCheck.rows << endl;
    //cerr << "lC.cols" << leftCheck.cols << endl;
    leftLines = hasLines(leftCheck);
    forwardLines = hasLines(forwardCheck);
    rightLines = hasLines(rightCheck);
    if(size == 4 && (
 		((leftLines == LINES_WHITE) && (forwardLines == LINES_WHITE))  ||
		((leftLines == LINES_WHITE) && (rightLines == LINES_WHITE)) ||
		((rightLines == LINES_WHITE) && (forwardLines == LINES_WHITE))))
    {
        stop.data = true;
        finishline_pub.publish(stop);
    } else if (size == 1) {
    if(leftLines == LINES_WHITE || leftLines == LINES_BOTH)
	TurnRight(finImage);
    //cerr << "error 3" << endl;
    if(rightLines == LINES_YELLOW || rightLines == LINES_BOTH)
        TurnLeft(finImage);
    //cerr << "error 4" << endl;
    if(forwardLines == LINES_WHITE || forwardLines == LINES_BOTH)
        SharpRight(finImage);
    //cerr << "error 5" << endl;
    if(forwardLines == LINES_YELLOW ||  forwardLines == LINES_BOTH)
        SharpLeft(finImage);
    //cerr << "error 6" << endl;
    }
}
    resize(finImage, finImage, Size(cv_ptr->image.cols, cv_ptr->image.rows), 0, 0, INTER_LANCZOS4);

    cv_ptr->image=finImage;
    cv_ptr->encoding="mono8";
    cv_ptr->toImageMsg(rosimage);
    img_pub.publish(rosimage);
}

void TurnRight(Mat& image) {
    Point start(3*image.cols/4, 0);
    Point end(0, image.rows);
    line(image, start, end, 0, 3);
}
void TurnLeft(Mat& image) {
    Point start(1*image.cols/4, 0);
    Point end(image.cols, image.rows);
    line(image, start, end, 0, 3);
}
void SharpRight(Mat& image) {
    Point start(0, image.rows);
    Point end(image.cols, image.rows/2);
    line(image, start, end, 0, 3);
}
void SharpLeft(Mat& image) {
    Point start(0, image.cols*2/3);
    Point end(image.rows, image.cols);
    line(image, start, end, 0, 3);

}

void help(std::ostream& ostr) {
	ostr << "Usage: iarrc_line_detection _img_topic:=<image-topic>" << std::endl;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "iarrc_line_detection");
	ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // FIXME: Not expected behavior
    if(argc >= 2) {
	    help(std::cerr);
	    exit(1);
    }

    std::string img_topic;
    stop.data = false;
    nhp.param(std::string("img_topic"), img_topic, std::string("/image_projected"));
    nhp.param(std::string("img_file"), img_file, std::string("iarrc_image.png"));
    std::string finishline_topic;
    nhp.param(std::string("finishline_topic"), finishline_topic, std::string("/finishline"));
    finishline_pub = nh.advertise<std_msgs::Bool>(finishline_topic, 1);
    ROS_INFO("Image topic:= %s", img_topic.c_str());
    ROS_INFO("Image file:= %s", img_file.c_str());

    // Subscribe to ROS topic with callback
    ros::Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageSaverCB);
    img_pub = nh.advertise<sensor_msgs::Image>("/image_lines", 1);//image publisher

    // Debug publisher
    debug_pub = nh.advertise<sensor_msgs::Image>("/image_debug", 1);

	ROS_INFO("IARRC line detection node ready.");

	ros::spin();
	ROS_INFO("Shutting down IARRC line detection node.");
    return 0;
}

