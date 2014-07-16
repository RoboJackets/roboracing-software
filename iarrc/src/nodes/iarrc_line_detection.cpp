#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdlib.h>
#include "image_utils.hpp"

std::string img_file;
int sigma=0;
ros::Publisher img_pub;
ros::Publisher debug_pub;
sensor_msgs::Image rosimage;
int erosion_size=2;
int erosion_type=2; //Ellipse



using namespace std;
using namespace cv;

sensor_msgs::Image CvMatToRosImage(cv::Mat& img, std::string encoding) {
	cv_bridge::CvImage cv_img;
	sensor_msgs::Image ros_img;
	cv_img.image=img;
    cv_img.encoding=encoding;
    cv_img.toImageMsg(ros_img);
    return ros_img;
}



// ROS image callback
void ImageSaverCB(const sensor_msgs::Image::ConstPtr& msg) {
	
    cv_bridge::CvImagePtr cv_ptr;
    int thresh = 245;

	// Convert ROS to OpenCV
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}


    int width = cv_ptr->image.cols;
	int height = cv_ptr->image.rows;
	
    // Crop input image
    // myRect is bottom one-third
    // store in tmp
    Rect myRect = Rect(0,height*2/3,width,height*1/3);
	Mat tmp;
	Mat(cv_ptr->image,myRect).copyTo(tmp);


    // First equalizeHist()
    // Then threshold to find the lines
    // Erode and Dilate
    Mat grayscaleImg;
    cvtColor(tmp, grayscaleImg, CV_BGR2GRAY);
    equalizeHist(grayscaleImg, grayscaleImg);
    threshold(grayscaleImg, grayscaleImg, thresh, 255,THRESH_BINARY);

    Mat element = getStructuringElement( erosion_type,Size( 2*erosion_size + 1, 2*erosion_size ),Point( erosion_size, 1 ) );
    erode(grayscaleImg,grayscaleImg,element);
    dilate(grayscaleImg,grayscaleImg,element);

    // Create zeros mat
    // Insert the grayscaleImg
    //And then we are done!
	Mat output = Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
    grayscaleImg.copyTo(output(myRect));

    image_utils::transform_perspective(output, output);

 	
 	cv_ptr->image=output;
    cv_ptr->encoding="mono8";
    cv_ptr->toImageMsg(rosimage);
    img_pub.publish(rosimage);
}

void help(std::ostream& ostr) {
	ostr << "Usage: iarrc_line_detection _img_topic:=<image-topic>" << std::endl;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "iarrc_line_detection");
	ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // FIXME: Not expected behavior
    if(argc >= 2) {
	    help(std::cerr);
	    exit(1);
    }

    std::string img_topic;
    nhp.param(std::string("img_topic"), img_topic, std::string("/ps3_eye/image_color"));
    nhp.param(std::string("img_file"), img_file, std::string("iarrc_image.png"));

    ROS_INFO("Image topic:= %s", img_topic.c_str());
    ROS_INFO("Image file:= %s", img_file.c_str());

    // Subscribe to ROS topic with callback
    ros::Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageSaverCB);
    img_pub = nh.advertise<sensor_msgs::Image>("/image_lines", 1);//image publisher

    // Debug publisher
    debug_pub = nh.advertise<sensor_msgs::Image>("/image_debug", 1);

    ROS_INFO("Hi");

	ROS_INFO("IARRC image saver node ready.");
	ros::spin();
	ROS_INFO("Shutting down IARRC image saver node.");
    return 0;
}
