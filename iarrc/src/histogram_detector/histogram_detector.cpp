#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

ros::Publisher pub;
ros::Publisher debug_pub;

int hsv_select = 0; //0 = hue, 1 = sat, 2 = value

int threshold = 10;

int threshold_only_value = 200;
bool threshold_only = false;

int rectangle_x = 500;
int rectangle_y = 520;
int rectangle_width = 200;
int rectangle_height = 100;

int bins = 255;//180; //360;

cv::Mat output;


cv::Mat calculateHistogram(cv::Mat image, int bins){
	bool uniform = true; bool accumulate = false;
	int histSize = bins; //Establish number of BINS

	float range[] = { 0, bins } ; //the upper boundary is exclusive. HUE from 0 - 360
	const float* histRange = { range };
	int channels [] = {0};
	cv::Mat histogram;
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

	//blurring to assist in removing line from road mask...eh
	//cv::GaussianBlur(frame_hsv_planes[2], frame_hsv_planes[2], cv::Size(9,9), 2.0);

	if (!threshold_only) {
		cv::Mat image_rectangle = frame_hsv_planes[hsv_select](cv::Rect(rectangle_x,rectangle_y,rectangle_width, rectangle_height));

		cv::Mat road_histogram;

		road_histogram = calculateHistogram(image_rectangle, bins);

		cv::normalize(road_histogram, road_histogram, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());
		cv::Mat backProj;
		float hue_range[] = { 0, bins };
		const float* ranges = { hue_range };
		cv::calcBackProject( &frame_hsv_planes[hsv_select], 1, 0, road_histogram, backProj, &ranges, 1, true );

		output = backProj;
		cv::threshold(backProj, output, threshold, 255, cv::THRESH_BINARY);


		//morphology to to close up holes
		auto kernel0 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(7,7));
		cv::morphologyEx(output,output,cv::MORPH_CLOSE,kernel0);
		//auto kernel1 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
		//cv::morphologyEx(output, output, cv::MORPH_OPEN,kernal1);

		//make white the lines
		cv::bitwise_not(output,output);

	} else {
		//straight up value based thresholding
		cv::threshold(frame_hsv_planes[2], output, threshold_only_value, 255, cv::THRESH_BINARY);
		auto kernel_small = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
		cv::morphologyEx(output, output, cv::MORPH_OPEN,kernel_small);
		//#TODO: will need some very small morphology openning to remove stray pixels, and proper body removal
	}


	//remove the body kinda
	cv::rectangle(output,
			cv::Point(390,650),
			cv::Point(400 + 410,output.rows),
			cv::Scalar(0,0,0),CV_FILLED);

	//remove sky kinda
	cv::rectangle(output,
			cv::Point(0,0),
			cv::Point(output.cols,output.rows / 3 + 100),
			cv::Scalar(0,0,0),CV_FILLED);



	//publish mask image
  sensor_msgs::Image outmsg;
  cv_ptr->image = output;
  cv_ptr->encoding = "mono8";
  cv_ptr->toImageMsg(outmsg);

  pub.publish(outmsg);



	//publish image with rectangle for debugging
	cv::rectangle(frame,
			cv::Point(rectangle_x,rectangle_y),
			cv::Point(rectangle_x + rectangle_width,rectangle_y+rectangle_height),
			cv::Scalar(255,0,0),5);


	sensor_msgs::Image outmsg2;
	cv_ptr->image = frame;
	cv_ptr->encoding = "bgr8";
	cv_ptr->toImageMsg(outmsg2);

	debug_pub.publish(outmsg2);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "histogram_detector");

	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	nhp.param("threshold", threshold, 10);
	nhp.param("hsv_select", hsv_select, 0);
	nhp.param("bins", bins, 255);

	nhp.param("rectangle_x", rectangle_x, 500);
	nhp.param("rectangle_y", rectangle_y, 520);
	nhp.param("rectangle_width", rectangle_width, 200);
	nhp.param("rectangle_height", rectangle_height, 100);

	nhp.param("threshold_only", threshold_only, false);
	nhp.param("threshold_only_value", threshold_only_value, 200);


  pub = nh.advertise<sensor_msgs::Image>("/histogram_detector", 1); //test publish of image
	debug_pub = nh.advertise<sensor_msgs::Image>("/histogram_debug", 1); //test publish of image
	auto img_sub = nh.subscribe("/camera/image_color_rect_flipped", 1, img_callback);

	ros::spin();
	return 0;

}
