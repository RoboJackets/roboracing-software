#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

using namespace cv;
using namespace std;

ros::Publisher img_pub;
ros::Publisher bool_pub;
sensor_msgs::Image rosimage;
Mat element;

// ROS image callback
void ImageCB(const sensor_msgs::Image::ConstPtr& msg) { 
	cv_bridge::CvImagePtr cv_ptr;

	// Convert ROS to OpenCV
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("CV-Bridge error: %s", e.what());
		return;
	}
	
	Mat gray;
	Rect ROI(0, 2*cv_ptr->image.rows/3, cv_ptr->image.cols, cv_ptr->image.rows/3);

	cvtColor(cv_ptr->image(ROI), gray, CV_BGR2GRAY);
	
	equalizeHist(gray, gray);

	threshold(gray, gray, 240, 255, THRESH_BINARY);
	
	Mat output = Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
	
	erode(gray, gray, element);
	dilate(gray, gray, element);
	
	Mat scanline = Mat::zeros(gray.rows, gray.cols, CV_8UC1);
	
	line(scanline, Point(0,scanline.rows-100), Point(scanline.cols,scanline.rows-100), Scalar::all(1), 10);
	
	multiply(scanline, gray, output);
	
	int count = sum(output)[0] / 255;
	
	double percent = ((double)count) / 64.;
	if(percent > 30)
	{
		std::cout << percent << "%" << std::endl;
		std_msgs::Bool b;
		b.data = true;
		bool_pub.publish(b);
	}
	
	//gray.copyTo(output(ROI));

	cv_ptr->image=output;
	cv_ptr->encoding="mono8";
	cv_ptr->toImageMsg(rosimage);
	img_pub.publish(rosimage);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "iarrc_race_end_detector");
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");

	std::string img_topic;
	nhp.param(std::string("img_topic"), img_topic, std::string("/image_raw"));

	ROS_INFO("Image topic:= %s", img_topic.c_str());

	element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3), Point(1,1));

	// Subscribe to ROS topic with callback
	ros::Subscriber img_sub = nh.subscribe(img_topic, 1, ImageCB);
	img_pub = nh.advertise<sensor_msgs::Image>("/image_end", 1);
	bool_pub = nh.advertise<std_msgs::Bool>("/race_end",1);


	ROS_INFO("IARRC race ender node ready.");
	ros::spin();
	ROS_INFO("Shutting down IARRC race ender node.");
	return 0;
}
