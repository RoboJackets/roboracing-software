#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace ros;

using uchar = unsigned char;

//img size: 480 x 640 for camera

Publisher img_pub;
Mat mask;

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

				if(((B < 100 && B > 90) && (G < 100 && G > 90) && (R < 100 && R > 90))
					|| ((B < 215 && B > 200) && (G < 215 && G > 200) && (R < 215 && R > 200))){
					curb_row[c] = curb_row[c + 1] = curb_row[c + 2] = 0;
				} else {
					curb_row[c] = curb_row[c + 1] = curb_row[c + 2] = 255;
				}
			}
		}

	return curb;
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

	//applying mask and detectGrey() function to image
	frame = cv_ptr->image;
    frame = frame.mul(mask);
    Mat grey_img = detectCurb(frame);
    output = grey_img;

    sensor_msgs::Image outmsg;

    cv_ptr->image = output;
	cv_ptr->encoding = "bgr8";
	cv_ptr->toImageMsg(outmsg);
	img_pub.publish(outmsg);

	imshow("Image Window", output); //display image in "Image Window"
	waitKey(1);
}

int main(int argc, char** argv){

	//create "Image Window"
	namedWindow("Image Window", WINDOW_NORMAL);

	init(argc, argv, "color_detector_avc");

	NodeHandle nh;

	//Doesn't proccess the top half of the picture
	vector<Mat> mask_segments= {
	    Mat::zeros(360,1280,CV_8UC3),
	    Mat(360,1280,CV_8UC3, Scalar::all(1))
	};

	vconcat(mask_segments, mask);

	Subscriber img_saver_sub = nh.subscribe("/camera/image_raw", 1, ImageCB);

	img_pub = nh.advertise<sensor_msgs::Image>(string("/colors_img"), 1);

    spin();

	return 0;
}
