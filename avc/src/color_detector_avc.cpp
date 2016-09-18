#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "avc/transform_image.h"

using namespace std;
using namespace cv;
using namespace ros;

using uchar = unsigned char;

//img size: 480 x 640 for camera

Publisher img_pub;
Publisher cloud_pub;
Mat mask;

ServiceClient transformClient;

/* DETECTS ROAD

Mat detectGrey(const Mat& image){
	Mat frame;
	image.copyTo(frame);

	Mat output(image.rows, image.cols, CV_8UC3);

		for(int r = 0; r < frame.rows; r++){
			uchar* row = frame.ptr<uchar>(r);
			uchar* out_row = output.ptr<uchar>(r);
			for(int c = 0; c < frame.cols * frame.channels(); c += frame.channels()){
				uchar B = row[c];
				uchar G = row[c + 1];
				uchar R = row[c + 2];

				if((B < 190 && B > 80) && (G < 190 && G > 80) && (R < 190 && R > 80)){
					out_row[c] = out_row[c + 1] = out_row[c + 2] = 0;
				} else {
					out_row[c] = out_row[c + 1] = out_row[c + 2] = 255;
				}
			}
	}
	return output;
}
*/

//DETECTS CURBS
Mat detectCurb(const Mat& image){
	Mat frame;
	image.copyTo(frame);

	Mat curb(image.rows, image.cols, CV_8UC3);

		for(int r = 0; r < frame.rows; r++){
			uchar* row = frame.ptr<uchar>(r);
			uchar* curb_row = curb.ptr<uchar>(r);
			for(int c = 0; c < frame.cols * frame.channels(); c += frame.channels()){
				uchar B = row[c];
				uchar G = row[c + 1];
				uchar R = row[c + 2];

				if(((B < 105 && B > 85) && (G < 105 && G > 85) && (R < 105 && R > 85))
					|| ((B < 235 && B > 215) && (G < 235 && G > 215) && (R < 235 && R > 215))
					|| ((B < 196 && B > 174) && (G < 196 && G > 174) && (R < 196 && R > 174))){
					curb_row[c] = curb_row[c + 1] = curb_row[c + 2] = 255;
				} else {
					curb_row[c] = curb_row[c + 1] = curb_row[c + 2] = 0;
				}
			}
		}

	auto kernel_size = 7;
    Mat erosion_kernel = getStructuringElement(MORPH_CROSS, Size(kernel_size, kernel_size));

    erode(curb, curb, erosion_kernel);

    curb = curb.mul(mask);

	return curb;
}

Mat detectHoop(const Mat& image){
	Mat frame;
	image.copyTo(frame);

	Mat hoop(image.rows, image.cols, CV_8UC3);

		for(int r = 0; r < frame.rows; r++){
			uchar* row = frame.ptr<uchar>(r);
			uchar* hoop_row = hoop.ptr<uchar>(r);
			for(int c = 0; c < frame.cols * frame.channels(); c += frame.channels()){
				uchar B = row[c];
				uchar G = row[c + 1];
				uchar R = row[c + 2];

				if((B < 135 && B > 95) && (G < 140 && G > 100) && (R < 100 && R > 74)){
					hoop_row[c] = hoop_row[c + 1] = hoop_row[c + 2] = 255;
				} else {
					hoop_row[c] = hoop_row[c + 1] = hoop_row[c + 2] = 0;
				}
			}
		}

		return hoop;
}

Mat detectRamp(const Mat& image){
	Mat frame;
	image.copyTo(frame);

	Mat ramp(image.rows, image.cols, CV_8UC3);

		for(int r = 0; r < frame.rows; r++){
			uchar* row = frame.ptr<uchar>(r);
			uchar* ramp_row = ramp.ptr<uchar>(r);
			for(int c = 0; c < frame.cols * frame.channels(); c += frame.channels()){
				uchar B = row[c];
				uchar G = row[c + 1];
				uchar R = row[c + 2];

				if((B < 180 && B > 145) && (G < 165 && G > 110) && (R < 150 && R > 85)){
					ramp_row[c] = ramp_row[c + 1] = ramp_row[c + 2] = 255;
				} else {
					ramp_row[c] = ramp_row[c + 1] = ramp_row[c + 2] = 0;
				}
			}
		}

		auto kernel_size = 5;
    	Mat erosion_kernel = getStructuringElement(MORPH_CROSS, Size(kernel_size, kernel_size));

    	erode(ramp, ramp, erosion_kernel);

		return ramp;
}

void ImageCB(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cv_ptr;
	Mat frame;
	Mat output;

	try {
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("CV-Bridge error: %s", e.what());
		return;
	}


	//applying detectCurb() function to image
	frame = cv_ptr->image;
    Mat curbs = detectCurb(frame);
    Mat hoop = detectHoop(frame);
    Mat ramp = detectRamp(frame);
    output = curbs + hoop + ramp;

    sensor_msgs::Image outmsg;

    cv_ptr->image = output;
	cv_ptr->encoding = "bgr8";
	cv_ptr->toImageMsg(outmsg);

    avc::transform_image srv;
    srv.request.image = outmsg;
    if(transformClient.call(srv)) {
        outmsg = srv.response.image;
        img_pub.publish(outmsg);
        float pxPerMeter = 100.0;
        try {
            cv_ptr = cv_bridge::toCvCopy(outmsg, "bgr8");
        } catch(cv_bridge::Exception& e) {
            ROS_ERROR("CV_Bridge error: %s", e.what());
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Mat transformed = cv_ptr->image;
        cvtColor(transformed, transformed, CV_BGR2GRAY);
        for(int r = 0; r < transformed.rows; r++) {
            uchar* row = transformed.ptr<uchar>(r);
            for(int c = 0; c < transformed.cols; c++) {
                if(row[c]) {
                    pcl::PointXYZ point;
                    point.y = -1 * (c - transformed.cols / 2.0f) / pxPerMeter;
                    point.x = (transformed.rows - r) / pxPerMeter;
                    point.z = 0.0;
                    cloud->push_back(point);
                }
            }
        }

        pcl::PCLPointCloud2 cloud_pc2;
        pcl::toPCLPointCloud2(*cloud, cloud_pc2);
        sensor_msgs::PointCloud2 cloud_msg;
        pcl_conversions::fromPCL(cloud_pc2, cloud_msg);
        cloud_msg.header.frame_id = "camera";
        cloud_msg.header.stamp = ros::Time::now();
        cloud_pub.publish(cloud_msg);
    } else {
        ROS_ERROR("Failed to call service transform_image");
    }
}

int main(int argc, char** argv){

	//create "Image Window"
	namedWindow("Image Window", WINDOW_NORMAL);

	init(argc, argv, "color_detector_avc");

    //Doesn't proccess the top half of the picture
    vector<Mat> mask_segments= {
            Mat(1080,960,CV_8UC3, Scalar::all(1)),
            Mat::zeros(1080,960,CV_8UC3)
    };

    hconcat(mask_segments, mask);

	NodeHandle nh;

    transformClient = nh.serviceClient<avc::transform_image>("transform_image");

	img_pub = nh.advertise<sensor_msgs::Image>(string("/colors_img"), 1);

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/colors_img/cloud", 1);

    Subscriber img_saver_sub = nh.subscribe("/camera/image_rect", 1, ImageCB);

    spin();

	return 0;
}
