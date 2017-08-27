#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

using namespace std;

ros::Publisher pub;

//VAR Declaration
cv::Mat road_histogram_hue;
cv::Mat nonRoad_histogram_hue;
cv::Mat road_mask;

cv::Mat road_histogram_hue_normalized(361,1,5); //%
cv::Mat nonRoad_histogram_hue_normalized(361,1,5); //%

const int hist_bins_hue = 361; //0-360 +1 exclusive
const int hist_bins_saturation = 101; //0-100 +1 exclusive

//Histogram Calculation Rectangle
const int rectangle_x = 670; //TODO: set rectangle more in the middle? or find a better calibration point.
const int rectangle_y = 760;
const int rectangle_width = 580;
const int rectangle_height = 300;
const int rectangle_Area = rectangle_width * rectangle_height;



//###########################################################
cv::Mat calculateHistogram(cv::Mat image, cv::Mat histogram, int bins){
	bool uniform = true; bool accumulate = false;
	int histSize = bins; //Establish number of BINS

	float range[] = { 0, bins } ; //the upper boundary is exclusive. HUE from 0 - 360 //TODO: There is a compiler note.(and below) check...
	const float* histRange = { range };
	int channels [] = {0};

	cv::calcHist(&image,1,channels,cv::Mat(),histogram, 1 ,&histSize,&histRange,uniform,accumulate);
	return histogram;
}

cv::Mat train(cv::Mat hist_hue_road,int pixelTotal){
	//Normalize (sums to 1). Basically gives percentage of pixels of that color (0-1).
	for (int ubin = 0; ubin < hist_bins_hue; ubin++) {
		if(hist_hue_road.at<float>(ubin) > 0){
			hist_hue_road.at<float>(ubin) /= pixelTotal;
		}
	}
	return hist_hue_road;

}

cv::Mat createHueHistogram(cv::Mat image_hue){
	cv::Mat histogram_hue;
	cv::Mat image_rectangle_hue = image_hue(cv::Rect(rectangle_x,rectangle_y,rectangle_width, rectangle_height));

	histogram_hue = calculateHistogram(image_rectangle_hue,road_histogram_hue,hist_bins_hue);

	//Probability (sums to 1)
	train(histogram_hue,rectangle_Area);

	return histogram_hue;
}




void img_callback(const sensor_msgs::ImageConstPtr& msg) {
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		cv::Mat frame = cv_ptr->image;

		//frame to HSV colorspace
		cv::Mat frame_hsv;
		cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV );
		vector<cv::Mat> frame_hsv_planes;
		cv::split(frame_hsv, frame_hsv_planes);

		road_histogram_hue = createHueHistogram(frame_hsv_planes[0]);

		//average normalize maps
		road_histogram_hue_normalized = (road_histogram_hue_normalized + road_histogram_hue)/2;  //% TODOl should weight averages to give previous frames more weight


		//store histogram
		//per http://stackoverflow.com/questions/10277439/opencv-load-save-histogram-data
		cv::FileStorage fs("road_histogram_hue_normalized.yml", cv::FileStorage::WRITE);
		if (!fs.isOpened()) {ROS_FATAL_STREAM("unable to open file storage!"); return;}
		fs << "road_histogram_hue_normalized" << road_histogram_hue_normalized;
		fs.release();


		//draw area being calculated for #DEBUG
		cv::rectangle(frame,
			cv::Point(rectangle_x,rectangle_y),
			cv::Point(rectangle_x + rectangle_width,rectangle_y+rectangle_height),
			cv::Scalar(255,0,0),5);


    //publish image
    sensor_msgs::Image outmsg;
    cv_ptr->image = frame;
    cv_ptr->encoding = "bgr8";
    cv_ptr->toImageMsg(outmsg);

    pub.publish(outmsg);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "road_detector_histogramTrainer");

	ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::Image>("/road_detector_histogramTrainer", 1); //test publish of image
	auto img_sub = nh.subscribe("/camera/image_rect", 1, img_callback);

	ros::spin();
	return 0;

}
