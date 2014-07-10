#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <Eigen/Dense>

using namespace cv;
using namespace std;

ros::Publisher img_pub;
ros::Publisher bool_pub;
sensor_msgs::Image rosimage;
Point lastCenter;

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
		return;
	}

	GaussianBlur(cv_ptr->image,blurImg,Size(3,3), 0 ,0);
	{
		vector<Mat> channels;
		split(blurImg, channels);
		for(int i = 0; i < channels.size(); i++)
			equalizeHist(channels[i], channels[i]);
		merge(channels, blurImg);
	}
	cvtColor(blurImg,grayscaleImg,CV_BGR2GRAY);
	threshold(grayscaleImg, grayscaleImg, 254, 255, CV_THRESH_BINARY);
	
	Point center(0,0);
	int count = 0;
	for(int r = 0; r < grayscaleImg.rows; r++)
	{
		uchar* row = grayscaleImg.ptr<uchar>(r);
		for(int c = 0; c < grayscaleImg.cols; c++)
		{
			if(row[c])
			{
				center.x += c;
				center.y += r;
				count++;
			}
		}
	}

	center.x /= count;
	center.y /= count;

	int dx = center.x - lastCenter.x;
	int dy = center.y - lastCenter.y;
	
	if(dx < 10 && dy > 20)
	{
		std_msgs::Bool b;
		b.data = true;
		bool_pub.publish(b);
	}

	ROS_INFO("(%i, %i)\t%i", dx, dy, count);

	circlesImg=grayscaleImg.clone();
	
	lastCenter = center;

	cv_ptr->image=circlesImg;
	cv_ptr->encoding="mono8";
	cv_ptr->toImageMsg(rosimage);
	img_pub.publish(rosimage);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "iarrc_image_display");
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");

	std::string img_topic;
	nhp.param(std::string("img_topic"), img_topic, std::string("/image_raw"));

	ROS_INFO("Image topic:= %s", img_topic.c_str());

	// Subscribe to ROS topic with callback
	ros::Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageCB);
	img_pub = nh.advertise<sensor_msgs::Image>("/image_circles", 1);
	bool_pub = nh.advertise<std_msgs::Bool>("/light_change",1);


	ROS_INFO("IARRC stoplight watcher node ready.");
	ros::spin();
	ROS_INFO("Shutting down IARRC stoplight watcher node.");
	return 0;
}
