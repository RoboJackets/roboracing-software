#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int main(int argc, char** argv) {
	ros::init(argc, argv, "videoconverter");
	ros::NodeHandle handle;

	VideoCapture vid("/home/sastorer/IARRC/SparkFun AVC 2016 Course Preview - Ground Competition.mp4");
	if(!vid.isOpened()) {
		cout << "You done fucked up. Fix your video." << endl;
		return 0;
	} 

	Mat frame;
	rosbag::Bag bag;
	bag.open("bagvid.bag", rosbag::bagmode::Write);

	//namedWindow("window", WINDOW_NORMAL);
	ros::Rate rate(30);

	bag.setCompression(rosbag::compression::BZ2);

	while(vid.grab()) {
		vid.retrieve(frame);
		cv_bridge::CvImage cvimage;
		cvimage.image = frame;
		cvimage.encoding = "bgr8";
		cvimage.header.stamp = ros::Time::now();

		sensor_msgs::Image rawimg;
		cvimage.toImageMsg(rawimg);

		bag.write("camera/image_raw", ros::Time::now(), rawimg);

		/*imshow("window", frame);
		if(waitKey(1) == ' ') {
			break;
		}*/

		rate.sleep();
	}

	bag.close();
	vid.release();

	cout << "Success!" << endl;
	return 0;
}