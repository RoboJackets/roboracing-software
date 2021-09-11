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
// ros::Time joy_time = ros::Time::now();

void JoystickCB(const sensor_msgs::Joy::ConstPtr& msg) {
    rr_msgs::speed sp_cmd;
    rr_msgs::steering st_cmd;

    static int dir = 1;
    static int last_button = 0;

    if (last_button == 0 && msg->buttons[0] == 1) {
        if (abs(sp_cmd.speed) == 0) {
            dir *= -1;
        } else {
            ROS_WARN("Speed should be zero!");
        }
    }

    last_button = msg->buttons[0];

    sp_cmd.speed = speed_max * (dir * msg->axes[4] > 0 ? msg->axes[4] : 0);
    st_cmd.angle = (angle_max * msg->axes[0]);

    speed_publisher.publish(sp_cmd);
    steering_publisher.publish(st_cmd);
    // joy_time = ros::Time::now();
}

// void timerCallback(const ros::TimerEvent& event_time) {
//     if (event_time->last_real - joy_time > event_time->profile.last_duration) {
//         sp_cmd.speed = st_cmd.angle =  0.0;
//     }
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "rr_platform_joystick");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Create timer
    // ros::Timer timer = nh.createTimer(ros::Duration(1), timerCallback);

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
