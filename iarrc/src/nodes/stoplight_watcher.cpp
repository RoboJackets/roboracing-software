#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <Eigen/Dense>
#include <fstream>

using namespace cv;
using namespace std;

ros::Publisher img_pub;
ros::Publisher bool_pub;
sensor_msgs::Image rosimage;
Point lastCenter;
Mat element;
int last_dy = 0;
ofstream file;

int last_rSum = 0;

// ROS image callback
void ImageCB(const sensor_msgs::Image::ConstPtr& msg) { 
	cv_bridge::CvImagePtr cv_ptr;
	Mat blurImg;
	Mat grayscaleImg;
	Mat circlesImg;

	// Convert ROS to OpenCV
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("CV-Bridge error: %s", e.what());
		return;
	}

	GaussianBlur(cv_ptr->image,circlesImg,Size(3,3), 0 ,0);
	
	vector<Mat> channels;
	split(circlesImg, channels);
	int gSum = 0;
	int rSum = 0;
	//for(int i = 0; i < 3; i++)
	//	equalizeHist(channels[i], channels[i]);
	gSum = sum(channels[1])[0];
	rSum = sum(channels[2])[0];
	merge(channels, circlesImg);

	int dr = rSum - last_rSum;
	last_rSum = rSum;
	
	if(dr < -100000)
	{
		std_msgs::Bool b;
		b.data = true;
		bool_pub.publish(b);
	}
	
	file << gSum << "\t" << rSum << "\t" << dr << std::endl;

	cv_ptr->image=circlesImg;
	cv_ptr->encoding="bgr8";
	cv_ptr->toImageMsg(rosimage);
	img_pub.publish(rosimage);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "iarrc_image_display");
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	
	file.open("/home/matthew/Desktop/data.csv");
	file << "test" << endl;
	file.close();
	file.open("/home/matthew/Desktop/data.csv");

	std::string img_topic;
	nhp.param(std::string("img_topic"), img_topic, std::string("/image_raw"));

	ROS_INFO("Image topic:= %s", img_topic.c_str());

	element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5), Point(2,2));

	// Subscribe to ROS topic with callback
	ros::Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageCB);
	img_pub = nh.advertise<sensor_msgs::Image>("/image_circles", 1);
	bool_pub = nh.advertise<std_msgs::Bool>("/light_change",1);


	ROS_INFO("IARRC stoplight watcher node ready.");
	ros::spin();
	ROS_INFO("Shutting down IARRC stoplight watcher node.");
	file.close();
	return 0;
}
