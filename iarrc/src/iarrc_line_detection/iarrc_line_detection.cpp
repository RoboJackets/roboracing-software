#define LINE_WIDTH_PIXELS 10
#define THRESHOLD_VALUE 10

#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdlib.h>
#include <iarrc/image_utils.hpp>
#include <vector>

using namespace std;
using namespace cv;

std::string img_file;
ros::Publisher img_pub;
ros::Publisher debug_pub;
sensor_msgs::Image rosimage;

vector<Mat> kernal(8);
vector<Mat> kernalcompl(8);


vector<Mat> getGeometricMean(Mat& image);
void subtractOrthog(vector<Mat>& images);
Mat combine(vector<Mat>& images);

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

	// Convert ROS to OpenCV
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

    // Crop input image
    // ground is bottom 1/4 of blue channel
    Mat ground;
    Mat channel[3];
    split(cv_ptr->image, channel);
    Rect myRect = Rect(0,cv_ptr->image.rows*3/4,cv_ptr->image.cols,cv_ptr->image.rows/4);
	Mat(channel[0], myRect).copyTo(ground);

    // Make lines 3 pixels wide in image
    resize(ground, ground, Size(3*ground.cols/LINE_WIDTH_PIXELS, 3*ground.rows/LINE_WIDTH_PIXELS), 0, 0, INTER_LANCZOS4);

    vector<Mat> results = getGeometricMean(ground);
    cerr << "Subtracting Orthog" << endl;
    subtractOrthog(results);

    cerr << "Subtracting Orthog done combining results" << endl;
    Mat finImage = combine(results);

    cerr << "Combination complete, thresholding finImage" << endl;
    threshold(finImage, finImage, THRESHOLD_VALUE, 255, CV_THRESH_BINARY);

    cerr << "Thresholding done, preparing to publish" << endl;
//    // change perspective to overhead view
//    image_utils::transform_perspective(finImage, finImage);
    bitwise_not(finImage, finImage);

    resize(finImage, finImage, Size(cv_ptr->image.cols, cv_ptr->image.rows), 0, 0, INTER_LANCZOS4);

    cv_ptr->image=finImage;
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
    nhp.param(std::string("img_topic"), img_topic, std::string("/ps3_eye/image_raw"));
    nhp.param(std::string("img_file"), img_file, std::string("iarrc_image.png"));

    ROS_INFO("Image topic:= %s", img_topic.c_str());
    ROS_INFO("Image file:= %s", img_file.c_str());

    // Subscribe to ROS topic with callback
    ros::Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageSaverCB);
    img_pub = nh.advertise<sensor_msgs::Image>("/image_lines", 1);//image publisher

    // Debug publisher
    debug_pub = nh.advertise<sensor_msgs::Image>("/image_debug", 1);

	ROS_INFO("IARRC line detection node ready.");

// Setting up static variable kernal for callback function
    float karray[3][9][9] = {
            {
                    {-1, -1, -1, -1, -1, -1, -1, -1, -1},
                    {-1, -1, -1, -1, -1, -1, -1, -1, -1},
                    {-1, -1, -1, -1, -1, -1, -1, -1, -1},
                    { 1,  1,  1,  1,  1,  1,  1,  1,  1},
                    { 1,  1,  1,  1,  1,  1,  1,  1,  1},
                    { 1,  1,  1,  1,  1,  1,  1,  1,  1},
                    { 0,  0,  0,  0,  0,  0,  0,  0,  0},
                    { 0,  0,  0,  0,  0,  0,  0,  0,  0},
                    { 0,  0,  0,  0,  0,  0,  0,  0,  0}
            }, {
                    {-1, -1, -1, -1, -1, -1, -1, -1, -1},
                    {-1, -1, -1, -1, -1, -1, -1,  0,  1},
                    {-1, -1, -1, -1, -1,  0,  1,  1,  1},
                    {-1, -1, -1,  0,  1,  1,  1,  1,  1},
                    {-1,  0,  1,  1,  1,  1,  1, .5,  0},
                    { 1,  1,  1,  1,  1, .5,  0,  0,  0},
                    { 1,  1,  1, .5,  0,  0,  0,  0,  0},
                    { 1, .5,  0,  0,  0,  0,  0,  0,  0},
                    { 0,  0,  0,  0,  0,  0,  0,  0,  0}
            },  {
                    {-.89,-.89,-.89,-.89,-.89,-.89,-.89,   1,   1},
                    {-.89,-.89,-.89,-.89,-.89,-.89,   1,   1,   1},
                    {-.89,-.89,-.89,-.89,-.89,   1,   1,   1,   0},
                    {-.89,-.89,-.89,-.89,   1,   1,   1,   0,   0},
                    {-.89,-.89,-.89,   1,   1,   1,   0,   0,   0},
                    {-.89,-.89,   1,   1,   1,   0,   0,   0,   0},
                    {-.89,   1,   1,   1,   0,   0,   0,   0,   0},
                    {   1,   1,   1,   0,   0,   0,   0,   0,   0},
                    {   1,   1,   0,   0,   0,   0,   0,   0,   0}
            }
    };

    kernal[0] = Mat(9, 9, CV_32FC1, karray[0]) / 27;
    kernal[1] = Mat(9, 9, CV_32FC1, karray[1]) / 25;
    kernal[2] = Mat(9, 9, CV_32FC1, karray[2]) / 25;

    kernal[3] = kernal[1].t();
    kernal[4] = kernal[0].t();

    flip(kernal[3], kernal[5], 0);
    flip(kernal[2], kernal[6], 0);
    flip(kernal[1], kernal[7], 0);

    // kernalcompl are 180 degree rotations of kernal, looking for the other edge of the line
    for(int i = 0; i < kernal.size(); i++) {
        kernalcompl[i] = kernal[i].clone();
        flip(kernal[i], kernalcompl[i], -1);
    }

	ros::spin();
	ROS_INFO("Shutting down IARRC line detection node.");
    return 0;
}

vector<Mat> getGeometricMean(Mat& image) {
    Mat filtered, filteredcompl;
    vector<Mat> results;

    for(int i = 0; i < kernal.size(); i++) {
        filter2D(image, filtered, -1, kernal[i]);
        filter2D(image, filteredcompl, -1, kernalcompl[i]);

        filtered.convertTo(filtered, CV_16UC1, 1);
        filteredcompl.convertTo(filteredcompl, CV_16UC1, 1);

        results.push_back(filtered.mul(filteredcompl));
        results[i].convertTo(results[i], CV_8UC1, 1.0 / 256);
    }

    return results;
};

void subtractOrthog(vector<Mat>& images) {
    vector<Mat> imagesCopy;
    for(Mat& img : images)
        imagesCopy.push_back(img.clone());
    for(int i = 0; i < images.size(); i++) {
        images[i] -= imagesCopy[(i + images.size()/2) % images.size()];
    }
}

Mat combine(vector<Mat>& images) {
    Mat result = images[0].clone();
    for(int i = 1; i < images.size(); i++) {
        result = max(result, images[i]);
    }

    return result;
}
