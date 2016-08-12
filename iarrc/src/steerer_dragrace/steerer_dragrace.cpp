#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <climits>
#include <iarrc/constants.hpp>
#include <rr_platform_msgs/steering.h>
#include <rr_platform_msgs/speed.h>

using namespace std;
using namespace cv;
using namespace ros;

Publisher img_pub;
Publisher steer_pub;

int speed;

void SpeedCallback(const rr_platform_msgs::speed::ConstPtr& msg)
{
	speed = msg->speed;
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
	
	frame = cv_ptr->image;

    for(int r = 0; r < frame.rows; r++) {
        unsigned char* row = frame.ptr<unsigned char>(r);
        for(int c = 0; c < frame.cols * frame.channels(); c+= frame.channels()) {
            auto& blue = row[c];
            auto& green = row[c+1];
            auto& red = row[c+2];
            if(blue == 255 && green == 0 && red == 0) {
                blue = 0;
            } else if(blue != 0 || green != 0 || red != 0) {
                blue = green = red = 255;
            }
        }
    }
    cvtColor(frame, frame, CV_BGR2GRAY);
   
    //dragrace steerer uses limited steer directions and angles because track is straight
    auto leftRect = Rect(0,0,213,480);
    auto centerRect = Rect(213,0,214,480);
    auto rightRect = Rect(417,0,213,480);

    auto left = frame(leftRect);
    auto center = frame(centerRect);
    auto right = frame(rightRect);

    auto leftCount = countNonZero(left);
    auto centerCount = countNonZero(center);
    auto rightCount = countNonZero(right);

    // Bias towards going straight
    centerCount *= 0.90;

    vector<int> counts = {leftCount, centerCount, rightCount};

    auto min_index = distance(counts.begin(),min_element(counts.begin(),counts.end()));

    rr_platform_msgs::steering steering_msg;
    steering_msg.header.stamp = Time::now();
    
    //drag
    switch(min_index) {
    case 0: // left
        rectangle(frame, leftRect, Scalar(255), 3);
        steering_msg.angle = -10;
        break;
    case 1: // center
        rectangle(frame, centerRect, Scalar(255), 3);
        steering_msg.angle = 0;
        break;
    case 2: // right
        rectangle(frame, rightRect, Scalar(255), 3);
        steering_msg.angle = 10;
        break;
    }

    if (speed == 0) {
        steering_msg.angle = 0;
    }

    steer_pub.publish(steering_msg);

    sensor_msgs::Image outmsg;

    cv_ptr->image = frame;
	cv_ptr->encoding = "mono8";
	cv_ptr->toImageMsg(outmsg);
	img_pub.publish(outmsg);
}

int main(int argc, char** argv) {
    
    init(argc, argv, "steerer_dragrace");

    NodeHandle nh;

    ros::Subscriber img_saver_sub = nh.subscribe("/colors_img", 1, ImageCB);
    
    ros::Subscriber speed_sub = nh.subscribe("/speed",1,SpeedCallback);
	
	img_pub = nh.advertise<sensor_msgs::Image>(string("/steerer_debug_img"), 1);

    steer_pub = nh.advertise<rr_platform_msgs::steering>(string("/steering"), 1);
        
    spin();

    return 0;
}
