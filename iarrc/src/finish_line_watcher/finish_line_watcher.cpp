/*
 * Modified version of old speed_controller node which now
 * publishes the number of times the robot has crossed the
 * finish line
 */

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <climits>
#include <rr_platform/speed.h>
#include <std_msgs/Int8.h>

using namespace std;
using namespace cv;
using namespace ros;

using uchar = unsigned char;

Publisher debug_pub;
Publisher crosses_pub;

#define HIGH 1
#define LOW 0

int state = LOW;

int number_of_crosses = 0;

int getWidth(const Mat& image) {

    int width = 0;

    bool in_line = false;
    int start;

    for(int r = 0; r < image.rows; r++) {
        auto row = image.ptr<uchar>(r);
        for(int c = 0; c < image.cols; c++) {
            if(!in_line && row[c]) {
                in_line = true;
                start = c;
            } else if(in_line && !row[c]) {
                in_line = false;
                auto my_width = c - start;
                width = max(width, my_width);
            }
        }
    }
    return width;
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
                blue = green = red = 255;
            } else if(blue != 0 || green != 0 || red != 0) {
                blue = green = red = 0;
            }
        }
    }
    cvtColor(frame, frame, CV_BGR2GRAY);

    Rect ROI(0,270,640,50);

    frame = frame(ROI);

    auto count = countNonZero(frame);

    auto width = getWidth(frame);

    if(state == LOW && count > 1000 && width > 240) {
        state = HIGH;
    } else if(state == HIGH && count < 1000) {
        // We crossed the line!
        state = LOW;
        number_of_crosses++;
	ROS_INFO("Finish line crossed - " + to_string(number_of_crosses));
    }

    sensor_msgs::Image outmsg;
    cv_ptr->image = frame;
	cv_ptr->encoding = "mono8";
	cv_ptr->toImageMsg(outmsg);
	debug_pub.publish(outmsg);
}

int main(int argc, char** argv) {
    
    init(argc, argv, "speed_controller");

    NodeHandle nh;
    NodeHandle nhp("~");

    string img_topic;
    nhp.getParam("img_topic", img_topic);
    ROS_INFO("Finish line watching %s", img_topic.c_str());

    Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageCB);

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

    return 0;
}
