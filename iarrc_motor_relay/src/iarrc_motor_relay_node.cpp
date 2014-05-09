#include <ros/ros.h>
#include <ros/publisher.h>
#include <iarrc_msgs/DriveCommand.h>
#include <boost/asio.hpp>
#include <string>

iarrc_msgs::DriveCommand cmd;
bool new_cmd = false;

void DriveCommandCB(const iarrc_msgs::DriveCommand::ConstPtr& msg)
{
	cmd = *msg;
	new_cmd = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iarrc_motor_relay_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // 
    std::string drive_command_topic;
    nhp.param(std::string("drive_command_topic"), drive_command_topic, std::string("/drive_command_topic"));
    ros::Subscriber drive_command_sub = nh.subscribe(drive_command_topic, 1, DriveCommandCB);
    
    // 
    std::string serial_port_name;
    nhp.param(std::string("serial_port"), serial_port_name, std::string("/dev/ttyUSB0"));
    boost::asio::io_service io_service;
    boost::asio::serial_port serial(io_service, serial_port_name);

	ROS_INFO("IARRC motor relay node ready.");

	ros::Rate rate(10);         // 10 hz
	while(ros::ok() && serial.is_open()) {
		ros::spinOnce();

		if(!new_cmd) {
			std::stringstream ss;
			ss << (char)181 << (char)(cmd.servo_position + 90) << (char)cmd.motor_speed;
			try {
				std::string msg = ss.str();
				boost::asio::write(serial, boost::asio::buffer(msg.c_str(), msg.size()));
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
