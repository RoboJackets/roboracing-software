#include <ros/ros.h>
#include <ros/publisher.h>
#include <iarrc_msgs/iarrc_speed.h>
#include <iarrc_msgs/iarrc_steering.h>
#include <sensor_msgs/Joy.h>
#include <string>

ros::Publisher speed_publisher;
ros::Publisher steering_publisher;
int angle_max;
int speed_max;

void JoystickCB(const sensor_msgs::Joy::ConstPtr& msg) {
    if(!msg) {
        ROS_ERROR("Received void input pointer in JoystickCB.");
        return;
    }

    iarrc_msgs::iarrc_speed sp_cmd;
    iarrc_msgs::iarrc_steering st_cmd;
    sp_cmd.speed = speed_max * msg->axes[3];
    // -3 for steering offset
    st_cmd.angle = -(angle_max * 2 * msg->axes[0]);
    //st_cmd.angle = 20;//angle_max * msg->axes[0];

    speed_publisher.publish(sp_cmd);
    steering_publisher.publish(st_cmd);
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

    // Convert joystick commands into motor commands on these topics
    std::string speed_topic;
    nhp.param(std::string("speed_topic"), speed_topic, std::string("/speed"));
    speed_publisher = nh.advertise<iarrc_msgs::iarrc_speed>(speed_topic, 1);
    std::string steering_topic;
    nhp.param(std::string("steering_topic"), steering_topic, std::string("/steering"));
    steering_publisher = nh.advertise<iarrc_msgs::iarrc_steering>(steering_topic, 1);

    // Driving limits
    nhp.param(std::string("angle_max"), angle_max, 20);
    nhp.param(std::string("speed_max"), speed_max, 10);

	ROS_INFO("IARRC joystick node ready.");
	ros::spin();
	ROS_INFO("Shutting down IARRC joystick node.");

    return 0;
}
