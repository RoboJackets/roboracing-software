#include <ros/publisher.h>
#include <ros/ros.h>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>
#include <sensor_msgs/Joy.h>
#include <string>

ros::Publisher speed_publisher;
ros::Publisher steering_publisher;
double angle_max;
double speed_max;

void JoystickCB(const sensor_msgs::Joy::ConstPtr& msg) {
    rr_msgs::speed sp_cmd;
    rr_msgs::steering st_cmd;
    sp_cmd.speed = speed_max * ((-1 * msg->axes[5] + 1.) / 2.);
    st_cmd.angle = -(angle_max * msg->axes[0]);

    speed_publisher.publish(sp_cmd);
    steering_publisher.publish(st_cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rr_platform_joystick");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Subscribe to joystick_driver topic
    std::string joystick_topic;
    nhp.param(std::string("joystick_topic"), joystick_topic, std::string("/joy"));
    ros::Subscriber float_command_sub = nh.subscribe(joystick_topic, 1, JoystickCB);

    // Convert joystick_driver commands into motor commands on these topics
    std::string speed_topic;
    nhp.param(std::string("speed_topic"), speed_topic, std::string("/speed"));
    speed_publisher = nh.advertise<rr_msgs::speed>(speed_topic, 1);
    std::string steering_topic;
    nhp.param(std::string("steering_topic"), steering_topic, std::string("/steering"));
    steering_publisher = nh.advertise<rr_msgs::steering>(steering_topic, 1);

    // Driving limits
    nhp.param(std::string("angle_max"), angle_max, 0.366);
    nhp.param(std::string("speed_max"), speed_max, 0.2);

    ROS_INFO("joystick_driver node ready.");
    ros::spin();
    ROS_INFO("Shutting down joystick_driver node.");

    return 0;
}
