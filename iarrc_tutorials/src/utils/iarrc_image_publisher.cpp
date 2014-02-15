#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>

// Convert OpenCV Mat encoding to printable string
std::string CvToStr(int type) {
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

// Convert OpenCV Mat encoding to ROS encoding
// FIXME: Not all encodings have obvious conversions. Check
// sensor_msgs/image_encodings.h for a list of all the possible ROS
// encodings
std::string CvToRos(int type, std::string channels="bgr") {
	std::string encoding;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	encoding = channels;
	if(chans == 4)
		encoding += "a";

	switch (depth) {
	case CV_8U:
	case CV_8S:
		encoding += "8";
		break;
	case CV_16U: 
	case CV_16S:
		encoding += "16";
		break;
	case CV_32S:
	case CV_32F:
	case CV_64F:
	default:
		ROS_ERROR("No ROS encoding exists with bit-depth > 16: %s", CvToStr(type).c_str());
		return "";
	}

	return encoding;
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

    ROS_INFO("img_topic:= %s", img_topic.c_str());
    ROS_INFO("img_file:= %s", img_file.c_str());

    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>(img_topic, 1);

	ROS_INFO("IARRC image publisher node ready.");

	// Read in image using OpenCV
	cv_bridge::CvImage cv_img;
	cv_img.image = cv::imread(img_file, CV_LOAD_IMAGE_COLOR);
	if(!cv_img.image.data) {
		ROS_ERROR("Could not open or find the image: %s", img_file.c_str());
		return -1;
	}

	// Print out image information
	std::string type =  CvToStr(cv_img.image.type());
	ROS_INFO("Read in OpenCV image: %s %dx%d", type.c_str(), cv_img.image.cols, cv_img.image.rows);

	// Convert to ROS format
	sensor_msgs::Image image;
	cv_img.encoding = CvToRos(cv_img.image.type());
	cv_img.toImageMsg(image);
	ROS_INFO("As ROS image: %s %dx%d", image.encoding.c_str(), image.width, image.height);

	// Sleep so that ROS will finish publishing the image before shutdown
	ros::Rate rate(1);
	while(ros::ok()) {
		ROS_INFO("Publishing ROS Image");
		img_pub.publish(image);
		rate.sleep();
	}

	ROS_INFO("Shutting down IARRC image saver node.");
	return 0;
}
