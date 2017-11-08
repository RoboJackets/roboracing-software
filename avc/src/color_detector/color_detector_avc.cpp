#include "color_detector_avc.h"

using namespace std;
using namespace cv;
using namespace ros;

using uchar = unsigned char;


Mat detectObstacleColor(const Mat& image, const Scalar &low, const Scalar &high) {
    Mat frame;
    image.copyTo(frame);

    Mat blurredImage;
    GaussianBlur(frame, blurredImage, Size{blur_strength, blur_strength}, blur_strength);
	
    Mat frameHSV;
    cvtColor(blurredImage, frameHSV, CV_BGR2HSV);

    Mat obstacleImg;
    inRange(frameHSV, low, high, obstacleImg);

    Mat masked;
    obstacleImg.copyTo(masked, mask);

    return masked;
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

    NodeHandle nh;
    NodeHandle nhp("~");

    nhp.param(string("image_topic"), image_topic, string("/camera/image_rect"));
    nhp.param(string("pub_topic"), pub_topic, string("/colors_img"));
    nhp.param(string("image_width"), image_width, 640);
    nhp.param(string("image_height"), image_height, 480);
    nhp.param(string("mask_px_top"), mask_px_top, 320);
    nhp.param(string("mask_px_bottom"), mask_px_bottom, 1);
    nhp.param(string("blur_strength"), blur_strength, 7);

    img_pub = nh.advertise<sensor_msgs::Image>(pub_topic, 1);
    auto img_sub = nh.subscribe(image_topic, 1, ImageRectCB);

    int good_height = image_height - mask_px_bottom - mask_px_top;
    vector<Mat> mask_segments = {
            Mat::zeros(mask_px_top, image_width, CV_8UC1),
            Mat(good_height, image_width, CV_8UC1, Scalar::all(1)),
            Mat::zeros(mask_px_bottom, image_width, CV_8UC1)
    };
    vconcat(mask_segments, mask);

    lows.push_back(Scalar(0,50,0));
    highs.push_back(Scalar(180,255,255));

    lows.push_back(Scalar(0,0,0));
    highs.push_back(Scalar(180,255,85));

    spin();

    return 0;
}
