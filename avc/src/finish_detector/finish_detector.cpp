#include "finish_detector.h"
#include <climits>


using namespace std;
using namespace cv;
using namespace ros;

namespace avc {
using uchar = unsigned char;


void finish_detector::ImageCB(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
	Mat frame;
	Mat output;
	
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("CV-Bridge error: %s", e.what());
		return;
	}
	
	frame = cv_ptr->image;
	Mat frameHSV;
	cvtColor(frame, frameHSV, CV_BGR2HSV);

    Mat blurredImage;
    GaussianBlur(frameHSV, blurredImage, Size{7, 7}, 7);

    Mat redMask1 = Mat::zeros(blurredImage.rows, blurredImage.cols, CV_8U);
    Mat redMask2 = Mat::zeros(blurredImage.rows, blurredImage.cols, CV_8U);
    inRange(blurredImage, red_low1, red_high1, redMask1);
    inRange(blurredImage, red_low2, red_high2, redMask2);

    Mat mask;
	bitwise_or(redMask1, redMask2, mask);
	imshow("filtered", mask);
	waitKey(10);

    auto count = countNonZero(mask);

    if(state == LOW && count > 1000) {
        state = HIGH;
    } else if(state == HIGH && count < 1000) {
        // We crossed the line!
        state = LOW;
        if (ros::Time::now() > lastCross + ros::Duration(10)) {
            number_of_crosses++;
            lastCross = ros::Time::now();
        }
        NODELET_FATAL_STREAM("Finish line crossed - " << to_string(number_of_crosses));
    }

    sensor_msgs::Image outmsg;
    cv_ptr->image = frame;
	cv_ptr->encoding = "mono8";
	cv_ptr->toImageMsg(outmsg);
	debug_pub.publish(outmsg);
}

void finish_detector::onInit() {
    state = LOW;
    lastCross = ros::Time::now() - ros::Duration(100);
    number_of_crosses = 0;

    NodeHandle nh =  getNodeHandle();
    NodeHandle nhp = getPrivateNodeHandle();
    image_transport::ImageTransport it(nh);

    img_saver_sub = it.subscribe("/camera/image_raw", 1, &finish_detector::ImageCB, this);

    crosses_pub = nh.advertise<std_msgs::Int8>("finish_line_crosses", 1);
    debug_pub = nhp.advertise<sensor_msgs::Image>("finish_line_debug_img", 1);

    nhp.param("red_low_h", red_low_h, 160.f);
    nhp.param("red_low_s", red_low_s, 130.0f);
    nhp.param("red_low_v", red_low_v, 0.0f);
    red_low1 = Scalar{red_low_h, red_low_s, red_low_v};
    red_low2 = Scalar{0, red_low_s, red_low_v};

    nhp.param("red_high_h", red_high_h, 15.0f);
    nhp.param("red_high_s", red_high_s, 255.0f);
    nhp.param("red_high_v", red_high_v, 255.0f);
    red_high1 = Scalar{180, red_high_s, red_high_v};
    red_high2 = Scalar{red_high_h, red_high_s, red_high_v};

    Rate rate(30);
    while(ros::ok()) {
        std_msgs::Int8 intmsg;
        intmsg.data = number_of_crosses;
        crosses_pub.publish(intmsg);
   
        spinOnce();
        rate.sleep();
    }

}
}
PLUGINLIB_EXPORT_CLASS(avc::finish_detector, nodelet::Nodelet)
