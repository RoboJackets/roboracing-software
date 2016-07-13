#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <climits>
#include <iarrc/constants.hpp>

using namespace std;
using namespace cv;

ros::Publisher frame_pub;
sensor_msgs::Image rosimage;
Mat cones, lines, frame;

bool mergeFrames() {
	if(cones.rows != lines.rows || cones.cols != lines.cols)
	{
		//ROS_ERROR("Frame merger size mismatch");
		return false;
	}
	add(cones, lines, frame);
	return true;
}

void conesCB(const sensor_msgs::Image::ConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;

	try {
		cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("CV-Bridge error: %s", e.what());
		return;
	}
	cones = cv_ptr->image;

	if(mergeFrames()) {
		cv_ptr->image = frame;
		cv_ptr->encoding = "mono8";
		cv_ptr->toImageMsg(rosimage);
		frame_pub.publish(rosimage);
	}
}

void linesCB(const sensor_msgs::Image::ConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("CV-Bridge error: %s", e.what());
		return;
	}
	lines = cv_ptr->image;
	
	if(mergeFrames()) {
		cv_ptr->image = frame;
		cv_ptr->encoding = "mono8";
		cv_ptr->toImageMsg(rosimage);
		frame_pub.publish(rosimage);
	}
}

int main(int argc, char** argv) {
	frame = Mat::zeros(1500, 2000, CV_8UC1);
	lines = Mat::zeros(1500, 2000, CV_8UC1);
	cones = Mat::zeros(1500, 2000, CV_8UC1);

	ros::init(argc, argv, "drag_race_steerer");
	
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	
	string cones_topic;
	nhp.param(string("cones_topic"), cones_topic, string("/cones_img"));
	ros::Subscriber cones_sub = nh.subscribe(cones_topic, 1, conesCB);
	
	string lines_topic;
	nhp.param(string("lines_topic"), lines_topic, string("/lines_img"));
	ros::Subscriber lines_sub = nh.subscribe(lines_topic, 1, linesCB);
	
	string frame_topic;
	nhp.param(string("frame_topic"), frame_topic, string("/joint_frame"));
	frame_pub = nh.advertise<sensor_msgs::Image>(frame_topic,1);
	
	ROS_INFO("IARRC frame merger node ready.");
	ros::spin();
	ROS_INFO("Shutting down IARRC frame merger node.");
	return 0;
}
