#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

using namespace std;

const int rectangle_x = 670;
const int rectangle_y = 760;
const int rectangle_width = 580;
const int rectangle_height = 300;

//Global for keeping mask image
cv::Mat road_mask_init;
//Global for 1st run or not. Decides if using small mask or training off past data. TODO: not this. Change to better method of masking.
int firstRun = true;


ros::Publisher pub;



void img_callback(const sensor_msgs::ImageConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    cv::Mat hsv_image; //create hsv_image matrix
    cvtColor( frame, hsv_image, cv::COLOR_BGR2HSV ); //convert to hsv

	//Split into H S V channel planes.
    vector<cv::Mat> hsv_planes;
    cv::split( hsv_image, hsv_planes );


	//args for HUE calcHist()
    bool uniform = true; bool accumulate = false;

    cv:Mat nonRoad_hue_histogram; //Mat of stored NON ROAD histogram for HUE
    cv::Mat road_hue_histogram; //Mat of stored road histogram for HUE
    int hue_histSize = 361; //Hue from 0 - 360. Establish number of BINS

    float hue_range[] = { 0, 361 } ; //the upper boundary is exclusive. HUE from 0 - 361
    const float* hue_histRange = { hue_range };
    int hue_channels [] = {0};


    //args for SATURATION calcHist()
    cv:Mat nonRoad_saturation_histogram; //Mat of stored NON ROAD histogram for SATURATION
    cv::Mat road_saturation_histogram; //Mat of stored ROAD histogram for SATURATION
    int saturation_histSize = 101; //Hue from 0 - 100. Establish number of BINS

    float saturation_range[] = { 0, 101 } ; //the upper boundary is exclusive. SATURATION from 0 - 100
    const float* saturation_histRange = { saturation_range };
    int saturation_channels [] = {0};



    //#### BOOTLEG APRROX ####

    //first run use small rectangle. TODO: Change this (see todo at firstrun variable)!
	firstRun = false; //For use when comnbining frame knowledge



    //get rectangle of road for comparison
    cv::Mat compare_image = hsv_image( cv::Rect(rectangle_x,rectangle_y,rectangle_width, rectangle_height) );

   	//Split into H S V channel planes.
    vector<cv::Mat> compare_image_planes;
    cv::split(compare_image, compare_image_planes );


    //CALCULATE HISTOGRAM of HUE Channel
    calcHist( &compare_image_planes[0], 1, hue_channels, cv::Mat(), road_hue_histogram, 1, &hue_histSize, &hue_histRange, uniform, accumulate );

    //CALCULATE HISTOGRAM of SATURATION Channel
    calcHist( &compare_image_planes[1], 1, saturation_channels, cv::Mat(), road_saturation_histogram, 1, &saturation_histSize, &saturation_histRange, uniform, accumulate );


    //##### CREATE MASK #####
    const float hue_threshold = 1900; //threshold to decide if there are enough in histogram to say it is probably road pixel
    const float saturation_threshold = 2200;

    auto img_channels = hsv_image.channels();
    for(auto i = 0; i < hsv_image.rows; i++) {
        const unsigned char* row_hue =  hsv_planes[0].ptr<unsigned char>(i); //HUE
        const unsigned char* row_saturation = hsv_planes[1].ptr<unsigned char>(i); //SATURATION
        unsigned char* out_row   = road_mask.ptr<unsigned char>(i)

        for(auto j = 0; j < hsv_image.cols; j++) {
        	auto image_hue = row_hue[j]; //get HUE channel of pixel
        	auto image_saturation = row_saturation[j]; //get SATURATION channel of pixel
        	auto histogram_hue = road_hue_histogram.at<float>(image_hue);
        	auto histogram_saturation = road_saturation_histogram.at<float>(image_saturation);

        	/* //guestimation of hues that finds road ok
        	if(row[j] > 0 && row[j] < 18){
        		out_row[j] = 255;
        	} else{
        		out_row[j] = 0;
        	}
			*/

        	//#TODO:Should the hue and saturation masks be seperately blurred (filtered before mixing?
        	//make road_mask by combining masks of HUE and SATURATION (only where both have white road spots)
        	if (histogram_hue >= hue_threshold && histogram_saturation >= saturation_threshold) {
        		out_row[j] = 255; //PAINT IT WHITE! WE GOT ROAD

        	} else{
        		out_row[j] = 0; //no road
        	}

        }


    }


    //###CALCULATE ROAD AND NON ROAD HISTOGRAMS ####

    //CALCULATE HISTOGRAM of HUE Channel
    calcHist( &hsv_planes[0], 1, hue_channels, road_mask_init, road_hue_histogram, 1, &hue_histSize, &hue_histRange, uniform, accumulate );

    //CALCULATE HISTOGRAM of SATURATION Channel
    calcHist( &hsv_planes[1], 1, saturation_channels, road_mask_init, road_saturation_histogram, 1, &saturation_histSize, &saturation_histRange, uniform, accumulate );


	//nonRoad histogram

	cv::Mat nonRoad_mask_init;
	cv::bitwise_not(road_mask_init,nonRoad_mask_init); //invert road_mask

	//CALCULATE HISTOGRAM of HUE Channel
    calcHist( &hsv_planes[0], 1, hue_channels, nonRoad_mask_init, nonRoad_hue_histogram, 1, &hue_histSize, &hue_histRange, uniform, accumulate );

    //CALCULATE HISTOGRAM of SATURATION Channel
    calcHist( &hsv_planes[1], 1, saturation_channels, nonRoad_mask_init, nonRoad_saturation_histogram, 1, &saturation_histSize, &saturation_histRange, uniform, accumulate );


    //Compare each pixel to its respective HUE and SATURATION channel bins.
    cv::Mat road_mask(hsv_image.rows, hsv_image.cols, CV_8UC1);


    //###TRAINING##### //#TODO: WHY IS THIS 250????? change or figure out why
    //normalize road(and non road) pixels
    float road_pixels = cv::countNonZero(road_mask);
    float nonRoad_pixels = cv::countNonZero(~road_mask);
    for (int ubin=0; ubin<250; ubin++){
    	for(int vbin=0; vbin<250; vbin++){
    		if (road_hue_histogram.at<float>(ubin,vbin)>0){
    			road_hue_histogram.at<float>(ubin,vbin) /= road_pixels;
    		}
    		if (nonRoad_hue_histogram.at<float>(ubin,vbin)>0){
    			nonRoad_hue_histogram.at<float>(ubin,vbin) /= nonRoad_pixels;
    		}
    		if (road_saturation_histogram.at<float>(ubin,vbin)>0){
    			road_saturation_histogram.at<float>(ubin,vbin) /= road_pixels;
    		}
    		if (nonRoad_saturation_histogram.at<float>(ubin,vbin)>0){
    			nonRoad_saturation_histogram.at<float>(ubin,vbin) /= nonRoad_pixels;
    		}

    	}
    }








    //some form of blur (filtering) to get rid of small holes/delete extraneous #TODO
    //cv::medianBlur(road_mask,road_mask,1);
    //cv::blur(road_mask,road_mask,cv::Size(4,4),cv::Point(-1,-1));



    //set global mask for next iteration equal to this iteration
    ::road_mask_init = road_mask;

    //publish image
    sensor_msgs::Image outmsg;
    cv_ptr->image = road_mask;//hsv_planes[2];//DEBUG CUTOUT base_compare_image
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
