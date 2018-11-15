#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"

#include <stdlib.h>
#include <stdio.h>

ros::Publisher pub;
ros::Publisher debug_pub;

int line_threshold;
int dilating_kernel;
double contour_rect_extent;

void img_callback(const sensor_msgs::ImageConstPtr& msg) {
	//convert msg to mat
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	cv::Mat frame = cv_ptr->image;

	cv::Mat blur_gray, blur, edges_img;

	//Blurs and turns Frame to Gray
	//If Running slow it is LIKELY this
	bilateralFilter (frame, blur, 4, 100*1, 200*1 );
	cv::cvtColor( blur, blur_gray, CV_BGR2GRAY );

	//More Blur in Grey helps.... I think...
	cv::blur( blur_gray, blur_gray, cv::Size(3,3) );

  /// Canny detector
  cv::Canny(blur_gray, edges_img,line_threshold,line_threshold*3, 3 );

	cv::Mat dst;
	dst.create(frame.size(), frame.type() );
	dst = cv::Scalar::all(0);
	frame.copyTo( dst, edges_img);

	// Make lines a bit bigger
	auto kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
 	cv::morphologyEx(edges_img, edges_img, cv::MORPH_CLOSE,kernel);

	// Truly get ALL the lines
	cv::cvtColor( dst, edges_img, CV_BGR2GRAY );
	Canny(edges_img, edges_img,300,200, 3);

	//Blow UP lines
	kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(dilating_kernel,dilating_kernel));
 	cv::morphologyEx(edges_img, edges_img, cv::MORPH_CLOSE,kernel);

	//Kill Noise
	kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4));
	cv::morphologyEx(edges_img, edges_img, cv::MORPH_OPEN,kernel);

	//Block Sky
	cv::rectangle(edges_img,
		cv::Point(0,0),
		cv::Point(edges_img.cols,edges_img.rows / 3 + 140),
		cv::Scalar(0,0,0),CV_FILLED);

	//Block Bottom
	cv::rectangle(edges_img,
		cv::Point(0,edges_img.rows),
		cv::Point(edges_img.cols,2*edges_img.rows / 3 + 40),
		cv::Scalar(0,0,0),CV_FILLED);

	//Block Car Front
	cv::rectangle(edges_img,
			cv::Point(edges_img.cols/3,edges_img.rows),
			cv::Point(2*edges_img.cols/3,2*edges_img.rows / 3.0-20),
			cv::Scalar(0,255,0),CV_FILLED);


	cv::Mat contours_img(edges_img.rows,edges_img.cols,CV_8UC1,cv::Scalar::all(0));

	std::vector<std::vector<cv::Point>> contours; // Vector for storing contours
	std::vector<cv::Vec4i> hierarchy;

	//All individual features must have a area larger area to be shown
	int minimum_area = 1500;
	cv::findContours(edges_img, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
	for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
	{
		float area = cv::contourArea( contours[i],false);  //  Find the area of contour
			if(area > minimum_area && area < 15000){  //Can't be too small or over whole frame
				auto rect = cv::boundingRect(cv::Mat(contours[i]));
				float extent =  area / (rect.height * rect.width);
				if(extent < contour_rect_extent) { //How similar is it to a normal "noise-square"
					cv::drawContours(contours_img, contours, i, cv::Scalar(255,255,0), CV_FILLED, 8, hierarchy );
			}
		}
	}


	//Make Lines Colored
	cv::Mat contours_img_BGR, colored_contours_img;
	cv::cvtColor(contours_img, contours_img_BGR, cv::COLOR_GRAY2BGR);
	cv::Scalar color( 0,255,0);  //GREEN!
	cv::Mat colorMask = cv::Mat(contours_img.rows, contours_img.cols, CV_8UC3, color);
	cv::bitwise_and(colorMask,contours_img_BGR, colored_contours_img);

	//Overlap Colored Line on Image
	cv::Mat result;
	cv::addWeighted(colored_contours_img, .3, frame, .7, 0.0, result);

	//publish Mask overlapped image
  sensor_msgs::Image outmsg;
  cv_ptr->image = result;
  cv_ptr->encoding = "bgr8";
  cv_ptr->toImageMsg(outmsg);

  pub.publish(outmsg);

	//Debuging just colored Mask
	sensor_msgs::Image outmsg_mask;
	cv_ptr->image = colored_contours_img;
	cv_ptr->encoding = "bgr8";
	cv_ptr->toImageMsg(outmsg_mask);

	debug_pub.publish(outmsg_mask);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "histogram_detector");

	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	nhp.param("line_threshold", line_threshold, 24);  //Increment to reduce Noise (MOST Influential)
	nhp.param("dilating_kernel", dilating_kernel, 15); //Diate Contours (Reduces Flickering)
 	nhp.param("contour_rect_extent", contour_rect_extent, 0.3);  //Reduces square-shaped noise

  pub = nh.advertise<sensor_msgs::Image>("/canny_overlapped_lines", 1); //test publish of image
	debug_pub = nh.advertise<sensor_msgs::Image>("/canny_colored_mask", 1); //test publish of image
	auto img_sub = nh.subscribe("/camera/image_color_rect_flipped", 1, img_callback);   //Newer Videos
	//auto img_sub = nh.subscribe("/camera/image_rect", 1, img_callback);               //Older Videos

	ros::spin();
	return 0;

}
