#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <Eigen/Dense>

using namespace cv;
using namespace std;

ros::Publisher img_pub;
ros::Publisher bool_pub;
sensor_msgs::Image rosimage;

Mat lastFrameRed, lastFrameGreen;

bool red=true;

int lowS = 100;
int highS = 200;
int lowV =150;
int highV = 255;

int lowHR = 0;
int highHR = 40;

int lowHL = 30;
int highHL = 120;

// ROS image callback
void ImageCB(const sensor_msgs::Image::ConstPtr& msg) { 
	cv_bridge::CvImagePtr cv_ptr;
    Mat circlesImg, circlesImgRed, circlesImgGreen;
    CvScalar cvs;

    int diff;

	// Convert ROS to OpenCV
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("CV-Bridge error: %s", e.what());
		return;
	}

    //Crop out relevant area.
    int width = cv_ptr->image.cols;
    int height = cv_ptr->image.rows;
    Rect trafficRect(0,height*3/8, width, height/4);
    cv_ptr->image(trafficRect).copyTo(circlesImg);

    //convert to HSV and threshold values to find red and green lights.
    cvtColor(circlesImg, circlesImg, CV_BGR2HSV);
    inRange(circlesImg, Scalar(lowHR, lowS, lowV), Scalar(highHR, highS, highV), circlesImgRed);
    inRange(circlesImg, Scalar(lowHL, lowS, lowV), Scalar(highHL, highS, highV), circlesImgGreen);
	
    //Put the cropped area back into the correctly sized image.
    Mat finalImg = Mat::zeros(height, width, circlesImgRed.type());
    if (red)
        circlesImgRed.copyTo(finalImg(trafficRect));
    else
        circlesImgGreen.copyTo(finalImg(trafficRect));

    //If we don't have any previous data, just copy the new images over
    if (!lastFrameRed.data){
        circlesImgRed.copyTo(lastFrameRed);
        circlesImgGreen.copyTo(lastFrameGreen);
    }
    else{
        if (red){
            //find difference in red frame
            cvs = sum(lastFrameRed) - sum(circlesImgRed);
            lastFrameRed = circlesImgRed;
            diff = cvs.val[0];
            cout <<"red: " << diff <<endl;
            //if large enough, say that the light is no longer red.
            if (diff > 50000)
                red = false;
        }
        if (!red){
            //Check difference in green frames
            cvs = sum(circlesImgGreen) - sum(lastFrameGreen);
            diff = cvs.val[0];
            cout << "green: " << diff << endl;
            //if large enough, signal the car to go. 
            if (diff >50000){
                red = true;
                ROS_INFO("Stoplight Change Detected");
                std_msgs::Bool b;
                b.data = true;
                bool_pub.publish(b);
            }
        }
        lastFrameGreen = circlesImgGreen;

    }

   // cvtColor(finalImg, finalImg, CV_HSV2GRAY);

    cv_ptr->image=finalImg;
    cv_ptr->encoding="mono8";
	cv_ptr->toImageMsg(rosimage);
	img_pub.publish(rosimage);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "iarrc_image_display");
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");

	std::string img_topic;
	nhp.param(std::string("img_topic"), img_topic, std::string("/image_raw"));

	ROS_INFO("Image topic:= %s", img_topic.c_str());

	// Subscribe to ROS topic with callback
	ros::Subscriber img_saver_sub = nh.subscribe(img_topic, 1, ImageCB);
	img_pub = nh.advertise<sensor_msgs::Image>("/image_circles", 1);
	bool_pub = nh.advertise<std_msgs::Bool>("/light_change",1);


	ROS_INFO("IARRC stoplight watcher node ready.");
	ros::spin();
	ROS_INFO("Shutting down IARRC stoplight watcher node.");
	return 0;
}
