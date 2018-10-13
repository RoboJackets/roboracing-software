#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

ros::Publisher pub;


int thresholdValue = 10;

cv::Mat calculateHistogram(cv::Mat image, int bins){
	bool uniform = true; bool accumulate = false;
	int histSize = bins; //Establish number of BINS

	float range[] = { 0, bins } ; //the upper boundary is exclusive. HUE from 0 - 360
	const float* histRange = { range };
	int channels [] = {0};

	cv::calcHist(&image,1,channels,cv::Mat(),histogram, 1 ,&histSize,&histRange,uniform,accumulate);
	return histogram;
}

void img_callback(const sensor_msgs::ImageConstPtr& msg) {
	//converty msg to mat
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	cv::Mat frame = cv_ptr->image;

	//convert from rgb to hsv
	cv::Mat frame_hsv;
	cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV );
	std::vector<cv::Mat> frame_hsv_planes;
	cv::split(frame_hsv, frame_hsv_planes);


	int rectangle_x = 0;
	int rectangle_y = 0;
	int rectangle_width = 300;
	int rectangle_height = 100;
	cv::Mat image_rectangle_hue = frame_hsv_planes[0](cv::Rect(rectangle_x,rectangle_y,rectangle_width, rectangle_height));

	cv::Mat road_histogram;
	int bins = 360;
	road_histogram = calculateHistogram(image_rectangle_hue, bins);

	cv::normalize(road_histogram, road_histogram, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());
	cv::Mat backProj;
	float hue_range[] = { 0, bins };
	const float* ranges = { hue_range }
	cv::calcBackProject( &frame_hsv_planes[0], 1, 0, road_histogram, backProj, &ranges, 1, true );

	cv::Mat output = backProj;
	//cv::Mat output;
	//cv::threshold(backproj, output, thresholdValue, 255, cv::THRESH_BINARY);

	//publish image
  sensor_msgs::Image outmsg;
  cv_ptr->image = output;
  cv_ptr->encoding = "mono8";
  cv_ptr->toImageMsg(outmsg);

  pub.publish(outmsg);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "road_detector");

	ros::NodeHandle nh;
	//ros::NodeHandle nhp("~");
	//nhp.param("road_threshold_bootstrap", road_threshold_bootstrap);//, 0.5f);//#Defaults are done at the top. This should be changed and below uncommented #TODO





  pub = nh.advertise<sensor_msgs::Image>("/histogram_detector", 1); //test publish of image
	auto img_sub = nh.subscribe("/camera/image_rect", 1, img_callback);

	ros::spin();
	return 0;

}
