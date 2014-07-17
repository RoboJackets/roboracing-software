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
int last_diff = 0;

Mat lastFrame;

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

    cv_ptr->image.copyTo(circlesImg);

	GaussianBlur(cv_ptr->image,circlesImg,Size(3,3), 0 ,0);
	
	Point circCenter(497, 208);
	int circRadius = 13;
	
	//circle(circlesImg, circCenter, circRadius, Scalar(255,255,0));
	
	/*vector<Mat> channels;
	split(circlesImg, channels);
	for(int i = 0; i < 3; i++)
	{
		Rect ROI(i*circlesImg.cols/3,0,circlesImg.cols/3,circlesImg.rows);
		Size size(circlesImg.cols/3, circlesImg.rows);
		resize(channels[i], channels[i](ROI), size);
		ROI = Rect(((i+1)%3)*circlesImg.cols/3,0,circlesImg.cols/3,circlesImg.rows);
		channels[i](ROI) = Mat::zeros(size, CV_8UC1);
		ROI = Rect(((i+2)%3)*circlesImg.cols/3,0,circlesImg.cols/3,circlesImg.rows);
		channels[i](ROI) = Mat::zeros(size, CV_8UC1);
	}
	merge(channels, circlesImg);*/
	cvtColor(circlesImg, circlesImg, CV_BGR2GRAY);
	
	circlesImg(Rect(circlesImg.cols/3,0,circlesImg.cols/3,circlesImg.rows)) *= 0;
	
	if(!lastFrame.data)
		circlesImg.copyTo(lastFrame);
	
	Mat diff = Mat::ones(lastFrame.rows, lastFrame.cols, CV_8UC1);
	for(int r = 0; r < diff.rows; r++)
	{
		uchar* dRow = diff.ptr(r);
		uchar* cRow = circlesImg.ptr(r);
		uchar* lRow = lastFrame.ptr(r);
		for(int c = 0; c < diff.cols; c++) {
			dRow[c] = 127 + ( ( cRow[c] - lRow[c] ) / 2 );
		}
	}
	
	diff(Rect(0,diff.rows-30,diff.cols,30)) *= 0;
	
	for(int r = 0; r < diff.rows-30; r++)
	{
		uchar* row = diff.ptr(r);
		uchar* row2 = diff.ptr(r+30);
		for(int c = 0; c < diff.cols; c++) {
			if(row[c] < 100 && row2[c] > 154) {
				row[c] = 255;
			} else {
				row[c] = 0;
			}
		}
	}
	
	lastFrame = circlesImg;
	
	int diffSum = sum(diff)[0]/255;
	
	if(diffSum > 10 && diffSum + last_diff > 185) {
		ROS_INFO("Stoplight Change Detected");
		std_msgs::Bool b;
		b.data = true;
		bool_pub.publish(b);
	}
	last_diff = diffSum;

	cv_ptr->image=diff;
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
