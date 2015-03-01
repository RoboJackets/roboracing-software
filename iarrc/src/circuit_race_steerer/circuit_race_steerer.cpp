#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <climits>
#include <iarrc/constants.hpp>
#include <iarrc_msgs/iarrc_steering.h>

using namespace std;
using namespace cv;

ros::Publisher steer_pub;

double penaltyForAngle(int angle) {
	int right_adjust;
	right_adjust = angle < 5 ? angle : angle - 5;
	double mu = 8;
 	return ( 1.0 - ( exp( - (right_adjust * right_adjust) / (2 * mu * mu) ) / ( sqrt( 2 * M_PI ) * mu ) ) ) * 10;
}

void frameCB(const sensor_msgs::Image::ConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	// Convert ROS to OpenCV
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("CV-Bridge error: %s", e.what());
		return;
	}
	
	int chosen_steer_angle = 0;
	int cost_of_chosen = INT_MAX;
	
	Mat frame = cv_ptr->image;
	Mat collisionsImg;
	Mat chosen_collisionsImg;
	
	for(int s = -20; s <= 20; s+=5) {
		double penalty = penaltyForAngle(s);
		collisionsImg = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
		if(s == 0) {
			line(collisionsImg, Point(frame.cols/2,0), Point(frame.cols/2,frame.rows), Scalar::all(1), constants::wheel_base * 100);
		} else {
			double radius = 1000 * ( 0.5 * constants::wheel_base + ( constants::chassis_length / tan(constants::steering_radians_per_pwm*abs(s)) ));
			radius *= s < 0 ? -1 : 1;
			double abs_radius = abs(radius);
			Point center(collisionsImg.cols / 2 - (int)radius, 0);
			double startAngle = radius > 0 ? 0 : 180;
			double endAngle = 90;
			ellipse(collisionsImg, center, Size(abs_radius, abs_radius), 0, startAngle, endAngle, Scalar::all(1), constants::wheel_base * 100);
		}
		multiply(collisionsImg, frame, collisionsImg);
		int cost = sum(collisionsImg)[0] + penalty;
		if(cost < cost_of_chosen) { 
			cost_of_chosen = cost;
			chosen_steer_angle = s;
			collisionsImg.copyTo(chosen_collisionsImg);
		}
		cout << cost << "\t";
	}
	cout << endl;
	
	iarrc_msgs::iarrc_steering pmsg;
	pmsg.angle = chosen_steer_angle - 3;
	//pmsg.angle = -1*chosen_steer_angle;
	steer_pub.publish(pmsg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "circuit_race_steerer");
	
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	
	string frame_topic;
	nhp.param(string("frame_topic"), frame_topic, string("/joint_frame"));
	ros::Subscriber frame_sub = nh.subscribe(frame_topic, 1, frameCB);
	
	string steer_topic;
	nhp.param(string("steer_topic"), steer_topic, string("/steering"));
	steer_pub = nh.advertise<iarrc_msgs::iarrc_steering>(steer_topic,1);
	
	ROS_INFO("IARRC ciruit race steerer node ready.");
	ros::spin();
	ROS_INFO("Shutting down IARRC ciruit race steerer node.");
	return 0;
}
