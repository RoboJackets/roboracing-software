#include <ros/ros.h>
#include <ros/publisher.h>
#include <iarrc_msgs/DriveCommand.h>
#include <boost/asio.hpp>
#include <string>

iarrc_msgs::DriveCommand cmd;

void DriveCommandCB(const iarrc_msgs::DriveCommand::ConstPtr& msg)
{
	cmd = *msg;
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
    nhp.param(std::string("serial_port"), serial_port_name, std::string("/serial_port"));
    boost::asio::io_service io_service;
    boost::asio::serial_port serial(io_service, serial_port_name);

	ROS_INFO("IARRC motor relay node ready.");

	ros::Rate rate(60);         // 60 hz
	while(ros::ok() && serial.is_open()) {
		std::stringstream ss;
		ss << "s:" << cmd.servo_position << "m:" << cmd.motor_speed; // FIXME: Send less bytes? What if scientific notation? Convert to int? <-- Probably the best solution
		try {
			std::string msg = ss.str();
			boost::asio::write(serial, boost::asio::buffer(msg.c_str(), msg.size()));
		} catch (boost::system::system_error& err) {
			ROS_ERROR("%s", err.what());
		}
		rate.sleep();
	}

	ROS_INFO("Shutting down IARRC motor relay node.");
    return 0;
}
