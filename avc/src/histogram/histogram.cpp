#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

using namespace std;

const int rectangle_x = 500;
const int rectangle_y = 500;
const int rectangle_width = 1000;
const int rectangle_height = 500;

ros::Publisher pub;


cv::Mat hist_compare(const cv::Mat frame, const cv::Mat histogram) {
    const int square_size = 20;
    cv::Mat road_img; //return image
    cv::Mat compare_square; //spot to compare histogram
    cv::Mat compare_square_histogram;


    //Args for calc hist
    bool uniform = true; bool accumulate = false;
    int value_bins = 50;
    cv::Mat value_base_hist; //Mat of stored histogram for hsv_value
    int histSize = 101; //hsv_value from 0 - 100

    float range[] = { 0, 101 } ; //the upper boundary is exclusive. hsv_value from 0 - 100 //#TOD0: not sure what this does. Change?
    const float* histRange = { range };
    int channels [] = {0};

    /*
    //############## test histogram viewing
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
    cv::Mat hist( hist_h, hist_w, CV_8UC1, cv::Scalar( 0,0,0) );
    //#############
    */


    for(auto i = 0; i <= frame.size().width - square_size+1; i+= square_size+1){
        for(auto j = 0; j <= frame.size().height-square_size+1; j+=square_size+1){
            //create spot square:
            compare_square = frame(cv::Rect(i,j,square_size,square_size));

            //CALCULATE HISTOGRAM
            calcHist(&compare_square, 1, channels,cv::Mat(), compare_square_histogram, 1, &histSize, &histRange, uniform, accumulate);

            //compare hist of spot
            double compareValue = compareHist(histogram,compare_square_histogram,0);
            //color white the spot if it is a match else black
            if (compareValue > 0){
                //color white!
                cv::rectangle(road_img,cv::Point(i,j),cv::Point(i+square_size,j+square_size),cv::Scalar(255,255,255),-1);
            }
            else{
                cv::rectangle(road_img,cv::Point(i,j),cv::Point(i+square_size,j+square_size),cv::Scalar(0,0,255),-1);
            }

            /*
            //########################## test histogram viewing
                for( int i = 1; i < histSize; i++ ){
                cv::line( hist, cv::Point( bin_w*(i-1), hist_h - cvRound(compare_square_histogram.at<float>(i-1)) ) ,
                     cv::Point( bin_w*(i), hist_h - cvRound(compare_square_histogram.at<float>(i)) ),
                     cv::Scalar( 179, 0, 0), 2, 8, 0  ); 
                }
            //##########################
            */


        }
        
    }

    //return white spot for road. Black for other
    return road_img;//hist;


}

void img_callback(const sensor_msgs::ImageConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    //#TODO: DO we need to convert to hsv?
    cv::Mat hsv_image; //create hsv_image matrix for conver
    cvtColor( frame, hsv_image, cv::COLOR_BGR2HSV ); //convert to hsv

    //get rectangle of road for comparison //#TODO: MAY NEED TO .copyTo(<copy>)
    cv::Mat base_compare_image = hsv_image( cv::Rect(rectangle_x,rectangle_y,rectangle_width, rectangle_height));
    
    //Split into H S V channel planes. #TODO: do we need to split and use only value portion?
    vector<cv::Mat> hsv_planes;
    cv::split( hsv_image, hsv_planes );
   


    //args for calcHist()
    bool uniform = true; bool accumulate = false;
    int value_bins = 50;
    cv::Mat value_base_hist; //Mat of stored histogram for hsv_value
    int histSize = 101; //hsv_value from 0 - 100

    float range[] = { 0, 101 } ; //the upper boundary is exclusive. hsv_value from 0 - 100 //#TOD0: not sure what this does. Change?
    const float* histRange = { range };
    int channels [] = {0};

    //CALCULATE HISTOGRAM
    calcHist( &hsv_planes[2], 1, channels, cv::Mat(), value_base_hist, 1, &histSize, &histRange, uniform, accumulate );



    //REMOVE AFTER THIS. FOR VIEWING OF HISTOGRAM GRAPH TESTS ONLY
    /*
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
    cv::Mat histImage( hist_h, hist_w, CV_8UC1, cv::Scalar( 0,0,0) );

    for( int i = 1; i < histSize; i++ ){
    cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(value_base_hist.at<float>(i-1)) ) ,
                     cv::Point( bin_w*(i), hist_h - cvRound(value_base_hist.at<float>(i)) ),
                     cv::Scalar( 179, 0, 0), 2, 8, 0  ); 
    }
    */

    cv::Mat road_img_out = hist_compare(hsv_planes[2],value_base_hist);


    //publish image
    sensor_msgs::Image outmsg;
    cv_ptr->image = road_img_out;//hsv_planes[2];//DEBUG CUTOUT base_compare_image //histImage;// DEBUG HISTOGRAM
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(outmsg);

    pub.publish(outmsg);
    

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "histogram");

	ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::Image>("/histogram", 1); //test publish of image
	auto img_sub = nh.subscribe("/camera/image_rect", 1, img_callback);

	ros::spin();
	return 0;
}
