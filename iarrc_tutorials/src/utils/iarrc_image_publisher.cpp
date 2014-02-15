#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>

std::string type2str(int type) {
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch ( depth ) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans+'0');

	return r;
}

void help(std::ostream& ostr) {
	ostr << "Usage: iarrc_image_publisher _img_topic:=<image-topic> _img_file:=<file-name>" << std::endl;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "iarrc_image_publisher");
	ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    if(argc >= 2) {
	    help(std::cerr);
	    return -1;
    }

    // Read in ROS parameters
    std::string img_topic, img_file;
    nhp.param(std::string("img_topic"), img_topic, std::string("/image_raw"));
    nhp.param(std::string("img_file"), img_file, std::string("default.jpg"));

    ROS_INFO("Image topic:= %s", img_topic.c_str());
    ROS_INFO("Image file:= %s", img_file.c_str());

    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>(img_topic, 1);

	ROS_INFO("IARRC image publisher node ready.");

	// Read in image using OpenCV
	cv_bridge::CvImage cv_img;
	cv_img.image = cv::imread(img_file, CV_LOAD_IMAGE_COLOR);
	if(!cv_img.image.data) {
		ROS_ERROR("Could not open or find the image: %s", img_file.c_str());
		return -1;
	}

	// Convert to ROS and publish
	sensor_msgs::Image image;
	cv_img.toImageMsg(image);

	std::string ty =  type2str( cv_img.image.type());
	ROS_INFO("Matrix: %s %dx%d \n", ty.c_str(), cv_img.image.cols, cv_img.image.rows);

	ROS_INFO("Publishing ROS Image");
	img_pub.publish(image);

	// Sleep so that ROS will finish publishing the image before shutdown
	ros::Rate rate(10);
	rate.sleep();

	ROS_INFO("Shutting down IARRC image saver node.");
	return 0;
}
