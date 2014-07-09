#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <Eigen/Dense>

std::string img_file;
int low_Threshold=50;
int const max_lowThreshold=100;
int CannyThreshold;
int ratio=2;
int sigma=0;
int brightThreshold=80;
int curPos;
ros::Publisher img_pub;
ros::Publisher bool_pub;
sensor_msgs::Image rosimage;


using namespace cv;
using namespace std;

bool goodCircle(Vec3f &info,Mat &img);

// ROS image callback
void ImageCB(const sensor_msgs::Image::ConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	Mat blurImg;
	Mat downBlurImg;
	Mat grayscaleImg;
	Mat downGrayscaleImg;
	Mat edgeOp;
	Mat downEdgeOp;
	Mat circlesImg;
	Mat kernel=getGaussianKernel(15,sigma);

	// Convert ROS to OpenCV
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	} catch (cv_bridge::Exception& e) {
		return;
	}

	vector <Vec3f> circlesdata;
	GaussianBlur(cv_ptr->image,blurImg,Size(9,9),sigma,sigma);
	cvtColor(blurImg,grayscaleImg,CV_BGR2GRAY);
	Canny(grayscaleImg,edgeOp,low_Threshold,low_Threshold*ratio,3);
	circlesImg=blurImg.clone();
	HoughCircles( grayscaleImg, circlesdata, CV_HOUGH_GRADIENT, 1, grayscaleImg.rows/4, low_Threshold*2, 50, 0, 0 );

	for( size_t i = 0; i < circlesdata.size(); i++ )
	{	
		if (goodCircle(circlesdata [i],grayscaleImg))
		{ 
      			if  (abs(circlesdata[i][1]-curPos)>circlesdata[i][2])
      			{
      				std_msgs::Bool change;
      				change.data=true;
      				bool_pub.publish(change);
      			}
	      		Point center(cvRound(circlesdata[i][0]), cvRound(circlesdata[i][1]));
	      		int radius = cvRound(circlesdata[i][2]);
	      		circle( circlesImg, center, 3, Scalar(0,255,0), -1, 8, 0 );
	      		circle( circlesImg, center, radius, Scalar(127,0,127), 2, 8, 0 );
	      		ROS_INFO("x:%f\ty:%f\tradius:%f",circlesdata[i][0],circlesdata[i][1],circlesdata[i][2]);
	      		curPos=circlesdata[i][1];
		}
	}

	cv_ptr->image=circlesImg;
	cv_ptr->encoding="bgr8";
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



bool goodCircle(Vec3f &info,Mat &img)
{
	int centerx=info[0];
	int centery=info[1];
	float radius=info[2];
	//ROS_INFO("x:%d\ty:%d\tradius:%f",centerx,centery,radius);
	double goodPoints=0;
	double totalPoints=0;
	//ROS_INFO("Test Value:%d",img.at<uchar>(centery,centerx));
	
	for (int i = 0; i < img.cols; i++)
	{
		for (int j = 0; j < img.rows; j++)
		{
			double dist=pow((pow((centerx-i),2)+pow((centery-j),2)),(0.5));
			if (dist<=radius)
			{
				if (img.at<uchar>(j,i)>240)
				{	
					goodPoints++;
				}
			}
			totalPoints++;
		}
	}

	totalPoints=3.14*pow(radius,2);
	double percent=goodPoints/totalPoints*100;

	//ROS_INFO("Good:%f, Total:%f, Ratio:%f",goodPoints,totalPoints,percent);

	if (percent>=brightThreshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}
