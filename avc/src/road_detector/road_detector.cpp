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

const int hist_bins_hue = 361; //0-360 +1 exclusive
const int hist_bins_saturation = 101; //0-100 +1 exclusive


//###########################################################
cv::Mat calculateHistogram(cv::Mat image, cv::Mat histogram, int bins){
	bool uniform = true; bool accumulate = true;//false;
	int histSize = bins; //bins; //Establish number of BINS

	float range[] = { 0, bins } ; //the upper boundary is exclusive. HUE from 0 - 360 //TODO: There is a compiler note.(and below) check...
	const float* histRange = { range };
	int channels [] = {0};

	cv::calcHist(&image,1,channels,cv::Mat(),histogram, 1 ,&histSize,&histRange,uniform,accumulate);
	return histogram;
}

cv::Mat calculateHistogramMask(cv::Mat image, cv::Mat histogram, cv::Mat mask, int bins){
	bool uniform = true; bool accumulate = true;//false;
	int histSize = bins; //Establish number of BINS

	float range[] = { 0, bins } ; //the upper boundary is exclusive. HUE from 0 - 360 //TODO: There is a compiler note. (and above) check...
	const float* histRange = { range };
	int channels [] = {0};

	cv::calcHist(&image,1,channels,mask,histogram, 1, &histSize,&histRange,uniform,accumulate);
	return histogram;
}



cv::Mat bootstrap(cv::Mat image_hue){
	float threshold = 0.08; //TODO: Play with this number 0-1

	const int rectangle_x = 670; //TODO: set rectangle more in the middle? or find a better calibration point.
	const int rectangle_y = 760;
	const int rectangle_width = 580;
	const int rectangle_height = 300;

	cv::Mat mask(image_hue.rows, image_hue.cols, CV_8UC1); //mask. Same width/height of input image. Greyscale.
	cv::Mat histogram_hue;
	cv::Mat image_rectangle_hue = image_hue(cv::Rect(rectangle_x,rectangle_y,rectangle_width, rectangle_height));

	histogram_hue = calculateHistogram(image_rectangle_hue,road_histogram_hue,hist_bins_hue);

	//Normalize (sums to 1). Basically gives percentage of pixels of that color (0-1).
	int rectangle_pixels = (rectangle_width * rectangle_height);
	for (int ubin = 0; ubin < hist_bins_hue; ubin++) {
		if(histogram_hue.at<float>(ubin) > 0){
			histogram_hue.at<float>(ubin) /= rectangle_pixels;
		}else{
			histogram_hue.at<float>(ubin) = 0;
		}
	}

	//Mask making
	for(auto i = 0; i < image_hue.rows; i++){
		const unsigned char* row_hue = image_hue.ptr<unsigned char>(i); // HUE
		unsigned char* out_row = mask.ptr<unsigned char>(i); //output

		for(auto j = 0; j < image_hue.cols; j++){
			auto hue = row_hue[j]; //HUE of pixel
			auto histogram_hue_value = histogram_hue.at<float>(hue);

			if(histogram_hue_value >= threshold){
				out_row[j] = 255; //Road. White.
			}else{
				out_row[j] = 0; //nonRoad. Black.
			}

		}

	}

	return mask;
}


void train(cv::Mat hist_hue_road,cv::Mat hist_hue_nonRoad, cv::Mat mask){
	//Normalize (sums to 1). Basically gives percentage of pixels of that color (0-1).
	int road_pixels = cv::countNonZero(mask); int nonRoad_pixels = cv::countNonZero(~mask);
	for (int ubin = 0; ubin < hist_bins_hue; ubin++) {
		if(hist_hue_road.at<float>(ubin) > 0){
			hist_hue_road.at<float>(ubin) /= road_pixels;
		}
		if(hist_hue_nonRoad.at<float>(ubin) > 0){
			hist_hue_nonRoad.at<float>(ubin) /= nonRoad_pixels;
		}
	}
}


void predict(cv::Mat img_hue, cv::Mat hist_hue_road, cv::Mat hist_hue_nonRoad, cv::Mat mask){
	float threshold = 0.1; //TODO: Play with this value

	//Mask making based on threshold of road/nonRoad
	for(auto i = 0; i < img_hue.rows; i++){
		const unsigned char* row_hue = img_hue.ptr<unsigned char>(i); // HUE
		unsigned char* out_row = mask.ptr<unsigned char>(i); //output

		for(auto j = 0; j < img_hue.cols; j++){
			auto hue = row_hue[j]; //HUE of pixel
			auto hist_hue_road_val = hist_hue_road.at<float>(hue);
			auto hist_hue_nonRoad_val = hist_hue_nonRoad.at<float>(hue);

			if(hist_hue_nonRoad_val > 0){
				if((hist_hue_road_val / hist_hue_nonRoad_val) >= threshold){
					out_row[j] = 255; //Road. White.
				}else{
					out_row[j] = 0; //nonRoad. Black.
				}
			}else{
				out_row[j] = 0; //nonRoad. Black. //TODO:!!!! WHY THIS? why not check hist_hue_road_val??
			}

		}
	}

}



void img_callback(const sensor_msgs::ImageConstPtr& msg) {
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		cv::Mat frame = cv_ptr->image;

		//frame to HSV colorspace
		cv::Mat frame_hsv;
		cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV );
		vector<cv::Mat> frame_hsv_planes;
		cv::split(frame_hsv, frame_hsv_planes);

		road_mask = bootstrap(frame_hsv_planes[0]);//Guess of road

		road_histogram_hue = calculateHistogramMask(frame_hsv_planes[0],road_histogram_hue,road_mask,hist_bins_hue); //road
		nonRoad_histogram_hue = calculateHistogramMask(frame_hsv_planes[0],nonRoad_histogram_hue,~road_mask,hist_bins_hue); //nonRoad
		train(road_histogram_hue,nonRoad_histogram_hue,road_mask);
		predict(frame_hsv_planes[0],road_histogram_hue,nonRoad_histogram_hue,road_mask);

		/*TODO: RIGHT AWAY NEXT NOW LIKE NOW
			predict is having some problem. Not sure why but only kinda works if the input mask is inverted with ~.
			make bootstrap happen only once and accumulate needs to be turned on, so frames affect each other.
		*/

		/* THE PLAN:
		bootstrap mask (guess from rectangle)

		->
		calculate Histograms for road and non road;
		train those histograms (make them each add to 1)
		predict(create a new mask based on threshold compare of road and nonRoad)
		(repeat at -> x times)

		filter blur (something that checks if pixels around it are white, then makes it white or black based on that)
		large hole detection and filling
		*/


    //publish image
    sensor_msgs::Image outmsg;
    cv_ptr->image = road_mask;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(outmsg);

    pub.publish(outmsg);
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "road_detector");

	ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::Image>("/road_detector", 1); //test publish of image
	auto img_sub = nh.subscribe("/camera/image_rect", 1, img_callback);

	ros::spin();
	return 0;

}
