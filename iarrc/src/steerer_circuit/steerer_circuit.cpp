#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <climits>
#include <iarrc/constants.hpp>
#include <rr_platform/steering.h>
#include <rr_platform/speed.h>

using namespace std;
using namespace cv;
using namespace ros;

Publisher img_pub;
Publisher steer_pub;

int speed;

void SpeedCallback(const rr_platform::speed::ConstPtr& msg)
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

    
    //Circuit race steerer has more rectangles for sharper turns
    auto leftRect = Rect(0,0,128,480);
    auto leftCenterRect = Rect(128,0,128,480);
    auto centerRect = Rect(256,0,128,480);
    auto rightCenterRect = Rect(384,0,128,480);
    auto rightRect = Rect(512,0,128,480);

    auto left = frame(leftRect);
    auto leftCenter = frame(leftCenterRect);
    auto center = frame(centerRect);
    auto rightCenter = frame(rightCenterRect);
    auto right = frame(rightRect);

    auto leftCount = countNonZero(left);
    auto leftCenterCount = countNonZero(leftCenter);
    auto centerCount = countNonZero(center);
    auto rightCenterCount = countNonZero(rightCenter);
    auto rightCount = countNonZero(right);
    
    // Bias towards going straight
    centerCount *= 0.90;

    vector<int> counts = {leftCount, leftCenterCount, centerCount, rightCenterCount, rightCount};

    auto min_index = distance(counts.begin(),min_element(counts.begin(),counts.end()));

    rr_platform::steering steering_msg;
    steering_msg.header.stamp = Time::now();

//circuit
    switch(min_index) {
    case 0: // left
        rectangle(frame, leftRect, Scalar(255), 3);
        steering_msg.angle = -0.523599;
        break;
    case 1: // left center
        rectangle(frame, leftCenterRect, Scalar(255), 3);
        steering_msg.angle = -0.174533;
        break;
    case 2: // center
        rectangle(frame, centerRect, Scalar(255), 3);
        steering_msg.angle = 0;
        break;
    case 3: // right center
        rectangle(frame, rightCenterRect, Scalar(255), 3);
        steering_msg.angle = 0.174533;
        break;
    case 4: // right
        rectangle(frame, rightRect, Scalar(255), 3);
        steering_msg.angle = 0.523599;
        break;
    }

    if (speed == 0) {
        steering_msg.angle = 0;
    }
    //cout << "Speed: " << speed;

    steer_pub.publish(steering_msg);

    sensor_msgs::Image outmsg;

    cv_ptr->image = frame;
	cv_ptr->encoding = "mono8";
	cv_ptr->toImageMsg(outmsg);
	img_pub.publish(outmsg);
}

int main(int argc, char** argv) {
    
    init(argc, argv, "steerer_circuit");

    NodeHandle nh;

    ros::Subscriber img_saver_sub = nh.subscribe("/colors_img", 1, ImageCB);
    
    ros::Subscriber speed_sub = nh.subscribe("/speed",1,SpeedCallback);
	
	img_pub = nh.advertise<sensor_msgs::Image>(string("/steerer_debug_img"), 1);

    steer_pub = nh.advertise<rr_platform::steering>(string("/steering"), 1);
        
    spin();

    return 0;
}
