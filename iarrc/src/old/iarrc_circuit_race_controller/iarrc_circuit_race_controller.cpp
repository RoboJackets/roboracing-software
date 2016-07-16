#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <iarrc_msgs/iarrc_speed.h>
#include <iarrc_msgs/iarrc_steering.h>
#include <std_msgs/Bool.h>
#include <string>

#define MAX_SPEED 18.0 //16.0 for circuit
#define MIN_SPEED 16.0
#define MAX_ANGLE 49.0

ros::Publisher speed_publisher;

ros::NodeHandle *nh;
ros::NodeHandle *nhp;

int drive_speed;
std::string race_end_topic;
bool stoplightSeen = true;
bool raceOver = false;

ros::Subscriber stoplight_subscriber;
ros::Subscriber finishline_subscriber;
ros::Subscriber steering_subscriber;

void stoplightCB(const std_msgs::Bool::ConstPtr& msg);
void finishlineCB(const std_msgs::Bool::ConstPtr& msg);
void steeringCB(const iarrc_msgs::iarrc_steering::ConstPtr& msg);
void racecar_set_speed(int);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "iarrc_circuit_race_controller");

	nh = new ros::NodeHandle();
	nhp = new ros::NodeHandle("~");

	std::string stoplight_topic;
    	nhp->param(std::string("stoplight_topic"), stoplight_topic, std::string("/light_change"));
    	stoplight_subscriber = nh->subscribe(stoplight_topic, 1, stoplightCB);

	nhp->param(std::string("drive_speed"), drive_speed, 14);

	std::string speed_topic;
    	nhp->param(std::string("speed_topic"), speed_topic, std::string("/speed"));
    	speed_publisher = nh->advertise<iarrc_msgs::iarrc_speed>(speed_topic, 1);
	racecar_set_speed(0);

	std::string steering_topic;
    	nhp->param(std::string("steering_topic"), steering_topic, std::string("/steering"));
	steering_subscriber = nh->subscribe(steering_topic, 1, steeringCB);

	std::string finishline_topic;
    	nhp->param(std::string("finishline_topic"), finishline_topic, std::string("/finishline"));
    	finishline_subscriber = nh->subscribe(finishline_topic, 1, finishlineCB);

	ROS_INFO("IARRC circuit-race_controller node ready.");
	ros::spin();

	delete nh;
	delete nhp;
	ROS_INFO("Shutting down IARRC circuit_race_controller node.");

    	return 0;
}

void steeringCB(const iarrc_msgs::iarrc_steering::ConstPtr& msg)
{
    static float speedPerAngle = (MAX_SPEED - MIN_SPEED)/MAX_ANGLE;
//    iarrc_msgs::iarrc_steering newSteering = *msg;
    if(stoplightSeen && !raceOver) {
	racecar_set_speed(MAX_SPEED - (msg->angle < 0 ? -1 : 1) * msg->angle * speedPerAngle);
    } else {
	racecar_set_speed(0);
    }
}
void stoplightCB(const std_msgs::Bool::ConstPtr& msg)
{
  
  if (msg->data && !stoplightSeen) {
	ROS_INFO("Drag Race Controller got stoplight.");
	stoplight_subscriber.shutdown();
	stoplightSeen = true;
	system("rosnode kill stoplight_watcher\n");
  }
}

void finishlineCB(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) {
	raceOver = true;
	ROS_INFO("Finishline Detected");
	system("rosnode kill iarrc_line_detection\n");
    }
}

void racecar_set_speed(int speed)
{
	iarrc_msgs::iarrc_speed sp_cmd;
	sp_cmd.speed = speed;
	speed_publisher.publish(sp_cmd);
}
