#include "finish_detector.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <climits>
#include <std_msgs/Int8.h>
#include <pluginlib/class_list_macros.h> 

using namespace std;
using namespace cv;
using namespace ros;

namespace rr_common {
using uchar = unsigned char;

Publisher debug_pub;

#define HIGH 1
#define LOW 0


int red_thresh = 0;
int blue_thresh = 0;

int state = LOW;

int number_of_crosses = 0;


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

    for(int r = 0; r < frame.rows; r++) {
        unsigned char* row = frame.ptr<unsigned char>(r);
        for(int c = 0; c < frame.cols * frame.channels(); c+= frame.channels()) {
            auto& blue = row[c];
            auto& green = row[c+1];
            auto& red = row[c+2];
            if(blue < 50 && green < 50 && red > 200) {
                blue = green = red = 255;
            } else {
                blue = green = red = 0;
            }
        }
    }
    cvtColor(frame, frame, CV_BGR2GRAY);

    Rect ROI(0,270,640,50);

    frame = frame(ROI);

    auto count = countNonZero(frame);

    if(state == LOW && count > 1000) {
        state = HIGH;
    } else if(state == HIGH && count < 1000) {
        // We crossed the line!
        state = LOW;
        number_of_crosses++;
	ROS_INFO_STREAM("Finish line crossed - " << to_string(number_of_crosses));
    }

    sensor_msgs::Image outmsg;
    cv_ptr->image = frame;
	cv_ptr->encoding = "mono8";
	cv_ptr->toImageMsg(outmsg);
	debug_pub.publish(outmsg);
}

void finish_detector::onInit() {
    
    NodeHandle nh =  getNodeHandle();
    NodeHandle nhp = getPrivateNodeHandle();
    image_transport::ImageTransport it(nh);

    img_saver_sub = it.subscribe("/camera_mono/image_raw", 1, &finish_detector::ImageCB, this);

    crosses_pub = nh.advertise<std_msgs::Int8>("finish_line_crosses", 1);
    debug_pub = nhp.advertise<sensor_msgs::Image>("finish_line_debug_img", 1);

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
PLUGINLIB_EXPORT_CLASS(rr_common::finish_detector, nodelet::Nodelet)
