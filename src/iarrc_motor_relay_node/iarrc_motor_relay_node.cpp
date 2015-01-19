#include <ros/ros.h>
#include <ros/publisher.h>
#include <iarrc_software/iarrc_speed.h>
#include <iarrc_software/iarrc_steering.h>
#include <boost/asio.hpp>
#include <string>

iarrc_software::iarrc_speed speed_cmd;
iarrc_software::iarrc_steering steering_cmd;
bool new_cmd = false;
int prevAngle = 0;
int prevSpeed = 0;

void SpeedCallback(const iarrc_software::iarrc_speed::ConstPtr& msg)
{
	speed_cmd = *msg;
	new_cmd = true;
}

void SteeringCallback(const iarrc_software::iarrc_steering::ConstPtr& msg)
{
	steering_cmd = *msg;
	new_cmd = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iarrc_motor_relay_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Subscribers
    std::string speed_topic_name;
    nhp.param(std::string("speed_topic"), speed_topic_name, std::string("/speed"));
    ros::Subscriber speed_sub = nh.subscribe(speed_topic_name, 1, SpeedCallback);

    std::string steering_topic_name;
    nhp.param(std::string("steering_topic"), steering_topic_name, std::string("/steering"));
    ros::Subscriber steering_sub = nh.subscribe(steering_topic_name, 1, SteeringCallback);

    ROS_INFO_STREAM("Listening for speed on " << speed_topic_name);
    ROS_INFO_STREAM("Listening for steer on " << steering_topic_name);

    // Serial port setup
    std::string serial_port_name;
    nhp.param(std::string("serial_port"), serial_port_name, std::string("/dev/ttyUSB0"));
    boost::asio::io_service io_service;
    boost::asio::serial_port serial(io_service, serial_port_name);

	ROS_INFO("IARRC motor relay node ready.");

	ros::Rate rate(10);         // 10 hz
	while(ros::ok() && serial.is_open()) {
		ros::spinOnce();

		// if(new_cmd) {
		if(true) {
			if(steering_cmd.angle != prevAngle || speed_cmd.speed != prevSpeed)
				ROS_INFO("Sending command: servo=%d, motor=%d", steering_cmd.angle, speed_cmd.speed);
			prevAngle = steering_cmd.angle;
			prevSpeed = speed_cmd.speed;

			char m[4];
			m[0] = 181;
			m[1] = (char)(speed_cmd.speed + 90);
			m[2] = (char)(steering_cmd.angle + 90);
			m[3] = 182;

			try {
				boost::asio::write(serial, boost::asio::buffer(m, 4));
			} catch (boost::system::system_error& err) {
				ROS_ERROR("%s", err.what());
			}
			new_cmd = false;
		}

		rate.sleep();
	}

	ROS_INFO("Shutting down IARRC motor relay node.");
    return 0;
}
