#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <climits>
#include <iarrc/constants.hpp>
#include <rr_platform/speed.h>
#include <std_msgs/Bool.h>

using namespace std;
using namespace cv;
using namespace ros;

using uchar = unsigned char;

Publisher debug_pub;
Publisher speed_pub;

#define HIGH 1
#define LOW 0

int state = LOW;

bool stoplight_state = false;

int number_of_crosses = 0;

bool flag = false;

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

    //ROS_INFO_STREAM("Count: " << count);
    //ROS_INFO_STREAM("Width: " << width);

    if(state == LOW && count > 8000 && width > 300) {
        state = HIGH;
    } else if(state == HIGH && count < 1000) {
        // We crossed the line!
        state = LOW;
        number_of_crosses++;
    }

    //ROS_INFO_STREAM("State: " << state);

    sensor_msgs::Image outmsg;

    cv_ptr->image = frame;
	cv_ptr->encoding = "mono8";
	cv_ptr->toImageMsg(outmsg);
	debug_pub.publish(outmsg);
}

void StoplightCB(const std_msgs::BoolConstPtr& msg) {
    stoplight_state = msg->data;
}

int main(int argc, char** argv) {
    
    init(argc, argv, "speed_controller");

    NodeHandle nh;
    NodeHandle nhp("~");

    Subscriber img_saver_sub = nh.subscribe("/colors_img", 1, ImageCB);
    Subscriber stoplight_sub = nh.subscribe("/light_change", 1, StoplightCB);
	
	debug_pub = nh.advertise<sensor_msgs::Image>("/speed_debug_img", 1);
    speed_pub = nh.advertise<rr_platform::speed>("/speed", 1);

    int max_crosses;
    nhp.param("max_crosses", max_crosses, int(1));
    
    int go_speed;
    nhp.param("go_speed", go_speed, int(9));
    ROS_INFO_STREAM("Go Speed is: " << go_speed);

    bool prev_stoplight_state = false;

    bool should_be_moving = false;

    Rate rate(30);
    while(ros::ok()) {
        if(!prev_stoplight_state && stoplight_state) {
            should_be_moving = true;
        }
        prev_stoplight_state = stoplight_state;

        if(number_of_crosses >= max_crosses) {
            should_be_moving = false;
        }

        //ROS_INFO_STREAM("Crosses: " << number_of_crosses);

        rr_platform::speed speed_msg;
        speed_msg.header.stamp = Time::now();

        if(should_be_moving) {
            speed_msg.speed = go_speed;
        } else {
            speed_msg.speed = 0;
        }
        speed_pub.publish(speed_msg);
   
        spinOnce();

        rate.sleep();
    }

    return 0;
}
