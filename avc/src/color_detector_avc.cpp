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
    output = curbs + hoop;

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
		Mat(1080,960,CV_8UC3, Scalar::all(1)),
	    Mat::zeros(1080,960,CV_8UC3)
	};

	hconcat(mask_segments, mask);

	Subscriber img_saver_sub = nh.subscribe("/camera/image_rect", 1, ImageCB);

	img_pub = nh.advertise<sensor_msgs::Image>(string("/colors_img"), 1);

    spin();

	return 0;
}
