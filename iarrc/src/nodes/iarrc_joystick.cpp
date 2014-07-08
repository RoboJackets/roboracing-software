#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/Joy.h>
#include <iarrc_msgs/MotorCommand.h>
#include <string>

ros::Publisher command_publisher; // Publishes the motor commands
int angle_max;
int speed_max;

void JoystickCB(const sensor_msgs::Joy::ConstPtr& msg) {
    if(!msg) {
        ROS_ERROR("Received void input pointer in JoystickCB.");
        return;
    }

    iarrc_msgs::MotorCommand command;
    command.angle = angle_max * msg->axes[0];
    command.speed = speed_max * msg->axes[3];

    // ROS_INFO_STREAM("Sending motor command:\n" << command);

    command_publisher.publish(command);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iarrc_joystick");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Subscribe to joystick topic
    std::string joystick_topic;
    nhp.param(std::string("joystick_topic"), joystick_topic, std::string("/joy"));
    ros::Subscriber float_command_sub = nh.subscribe(joystick_topic, 1, JoystickCB);

    // Convert joystick commands into motor commands on this topic
    std::string motor_topic;
    nhp.param(std::string("motor_topic"), motor_topic, std::string("/iarrc/motor_command"));
    command_publisher = nh.advertise<iarrc_msgs::MotorCommand>(motor_topic, 1);

    // Driving limits
    nhp.param(std::string("angle_max"), angle_max, 10);
    nhp.param(std::string("speed_max"), speed_max, 10);

	ROS_INFO("IARRC joystick node ready.");
	ros::spin();
	ROS_INFO("Shutting down IARRC joystick node.");

    return 0;
}
