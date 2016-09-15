#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "avc/transform_image.h"
#include "avc/calibrate_image.h"

using namespace std;
using namespace cv;
using namespace ros;

Publisher img_pub;
Mat transform_matrix;
vector<Point2f> corners;
int radius = 10;

void ImageTransformCB(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	Mat frame;
	Mat resize_frame;
	Mat output;

	try {
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("CV-Bridge error: %s", e.what());
		return;
	}

	frame = cv_ptr->image;
	sensor_msgs::Image outmsg;
	cv_ptr->encoding = "bgr8";
	cv_ptr->toImageMsg(outmsg);
	img_pub.publish(outmsg);

	ROS_INFO("Find board");

	bool patternfound = findChessboardCorners(frame, Size(8, 6), corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
        + CALIB_CB_FAST_CHECK);

	ROS_INFO("Process corners");

	if(!patternfound){
		ROS_WARN("Pattern not found!");
		return;
	}

	Mat gray;
	cvtColor(frame, gray, CV_BGR2GRAY);
	cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	//drawChessboardCorners(frame, Size(8, 6), Mat(corners), patternfound);
	circle(frame, corners[0], radius, Scalar(0, 255, 0));
	circle(frame, corners[7], radius, Scalar(0, 255, 0));
	circle(frame, corners[40], radius, Scalar(0, 255, 0));
	circle(frame, corners[47], radius, Scalar(0, 255, 0));

	Point2f src[4] = {corners[0], corners[7], corners[40], corners[47]};
	Point2f dst[4] = {Point(800, 750), Point(1100, 750), Point(800, 950), Point(1100, 950)};
	transform_matrix = getPerspectiveTransform(src, dst);

	warpPerspective(frame, frame, transform_matrix, Size(1920, 1080));

	imshow("Image Window", frame); //display image in "Image Window"
	waitKey(10);

}

bool TransformImage(avc::transform_image::Request &request, avc::transform_image::Response &response){
	cv_bridge::CvImagePtr cv_ptr;
	Mat outimage;
	cv_ptr = cv_bridge::toCvCopy(request.image);
	const Mat& inimage = cv_ptr->image;

	warpPerspective(inimage, outimage, transform_matrix, Size(1920, 1080));

	cv_bridge::CvImage imageconverter;
	imageconverter.image = outimage;
	imageconverter.encoding = "bgr8";
	imageconverter.toImageMsg(response.image);

	return true;
}

bool CalibrateImage(avc::calibrate_image::Request &request, avc::calibrate_image::Response &response){
	cv_bridge::CvImagePtr cv_ptr;
	Mat outimage;
	cv_ptr = cv_bridge::toCvCopy(request.image);
	const Mat& inimage = cv_ptr->image;

	bool patternfound = findChessboardCorners(inimage, Size(8, 6), corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
        + CALIB_CB_FAST_CHECK);

	ROS_INFO("Process corners");

	if(!patternfound){
		ROS_WARN("Pattern not found!");
		return false;
	}

	return true;
}


int main(int argc, char** argv){

	namedWindow("Image Window", WINDOW_NORMAL);

	init(argc, argv, "image_transform");
	NodeHandle nh;

	Subscriber img_saver_sub = nh.subscribe("/camera/image_rect", 1, ImageTransformCB);

	img_pub = nh.advertise<sensor_msgs::Image>(string("/colors_img"), 1);

	ServiceServer transform_service = nh.advertiseService("transform_image", TransformImage);
	ServiceServer calibrate_service = nh.advertiseService("calibrate_image", CalibrateImage);

	spin();

	return 0;
}
