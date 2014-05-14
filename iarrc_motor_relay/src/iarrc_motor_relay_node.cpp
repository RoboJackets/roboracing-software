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

    ROS_INFO_STREAM("Drive Command topic = " << drive_command_topic);

    // 
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

			std::stringstream ss;

			ROS_INFO("Sending command: servo=%d, motor=%d", cmd.servo_position, cmd.motor_speed);

			ss << (char)181 << (char)(cmd.motor_speed + 90) << (char)(cmd.servo_position + 90) << (char)182;

			char m[4];
			m[0] = 181;
			m[1] = (char)(cmd.motor_speed + 90);
			m[2] = (char)(cmd.servo_position + 90);
			m[3] = 182;

			std::string msg = ss.str();
			for(int i=0; i < 4; i++) {
				if(m[i] != msg[i])
					ROS_INFO("WTF");
			}

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
