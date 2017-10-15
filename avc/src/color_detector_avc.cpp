#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <rr_platform/transform_image.h>

using namespace std;
using namespace cv;
using namespace ros;

using uchar = unsigned char;

Publisher img_pub;
Mat mask;

vector<Scalar> lows;
vector<Scalar> highs;


Mat detectObstacleColor(const Mat& image, const Scalar &low, const Scalar &high) {
    Mat frame;
    image.copyTo(frame);
	Mat frameHSV;
	cvtColor(frame, frameHSV, CV_BGR2HSV);

	Mat blurredImage;
    GaussianBlur(frameHSV, blurredImage, Size{21, 21}, 15);

    Mat obstacleImg;
    inRange(blurredImage, low, high, obstacleImg);

    return obstacleImg;
}

void ImageRectCB(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cv_ptr;
    Mat frame;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV-Bridge error: %s", e.what());
        return;
    }
	frame = cv_ptr->image;
    Mat output(frame.rows, frame.cols, CV_8UC1, Scalar::all(0));

	for (int i = 0; i < lows.size(); i++) {
		Mat partial;
		partial = detectObstacleColor(frame, lows[i], highs[i]);
		bitwise_or(output, partial, output);
	}

    sensor_msgs::Image outmsg;
    cv_ptr->image = output;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(outmsg);
    img_pub.publish(outmsg);
}


int main(int argc, char** argv) {

    init(argc, argv, "color_detector_avc");

    //Doesn't proccess the top half of the picture
    vector<Mat> mask_segments = {
        Mat::zeros(640,240,CV_8UC3), 
		Mat(640,240,CV_8UC3, Scalar::all(1))
    };
    vconcat(mask_segments, mask);

    NodeHandle nh;

    img_pub = nh.advertise<sensor_msgs::Image>("/colors_img", 1);
    //auto img_sub = nh.subscribe("/camera/image_rect", 1, ImageRectCB);
	auto img_sub = nh.subscribe("/camera_wide/image_raw", 1, ImageRectCB);

    lows.push_back(Scalar(0,0,0));
    highs.push_back(Scalar(180,255,255));

    //lows.push_back(Scalar(0,0,0));
    //highs.push_back(Scalar(180,255,255));

    //lows.push_back(Scalar(0,0,0));
    //highs.push_back(Scalar(180,255,255));

    spin();

    return 0;
}
