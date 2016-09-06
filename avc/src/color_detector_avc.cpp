#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;
using namespace ros;

//img size: 480 x 640

Publisher img_pub;

Mat mask;

Mat detectGrey(const Mat& image){
	Mat frame;
	image.copyTo(frame);
	return frame;
} 

void ImageCB(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cv_ptr;
	Mat frame;
	Mat output;
	
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("CV-Bridge error: %s", e.what());
		return;
	}
	
	//applying 
	frame = cv_ptr->image;
    frame = frame.mul(mask);
    Mat grey_img = detectGrey(frame);
    output = grey_img;

    sensor_msgs::Image outmsg;

    cv_ptr->image = output;
	cv_ptr->encoding = "bgr8";
	cv_ptr->toImageMsg(outmsg);
	img_pub.publish(outmsg);

	imshow("Image Window", output); //display image in "Image Window"
    waitKey(0); //Waits for a keystroke in window so it doesn't immediately exit
}

int main(int argc, char** argv){
	init(argc, argv, "color_detector_avc");

	NodeHandle nh;

	Subscriber img_saver_sub = nh.subscribe("/ps3_eye/image_raw", 1, ImageCB);
	
	img_pub = nh.advertise<sensor_msgs::Image>(string("/colors_img"), 1);
        
    spin();

    //create "Image Window"
    namedWindow("Image Window", WINDOW_NORMAL);

	return 0;

}
