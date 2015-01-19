#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iarrc_software/image_utils.hpp>

using namespace std;
using namespace cv;

ros::Publisher img_pub;
sensor_msgs::Image rosimage;
Mat element;

void ImageCB(const sensor_msgs::Image::ConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	Mat frame;
	Mat output;
	
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("CV-Bridge error: %s", e.what());
		return;
	}
	
	frame = cv_ptr->image;

	
	cvtColor(frame, output, CV_BGR2HSV);

	int width = cv_ptr->image.cols;
	int height = cv_ptr->image.rows;

	Rect myRect = Rect(0,height*3/4,width,height/4);
	Mat(frame, myRect).copyTo(output);
	
	int lowH = 0;
	int highH = 20;
	int lowS = 70; //decrease this if shade becomes an issue. 
	int highS = 200;
	int lowV = 150;
	int highV =255;

	inRange(output, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), output);
	
	erode(output, output, element);
	dilate(output, output, element);
	
	image_utils::transform_perspective(output, output);
	
	cv_ptr->image = output;
	cv_ptr->encoding = "mono8";
	cv_ptr->toImageMsg(rosimage);
	img_pub.publish(rosimage);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "visual_cone_detector");
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	
	element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5), Point(2,2));
	
	string img_topic;
	nhp.param(string("img_topic"), img_topic, string("/image_raw"));
	ros::Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageCB);
	
	img_pub = nh.advertise<sensor_msgs::Image>(string("/cones_img"), 1);
	
	ROS_INFO("IARRC Cone Detector node ready.");
	ros::spin();
	ROS_INFO("Shutting down IARRC Cone Detector node.");
	
	return 0;
}
