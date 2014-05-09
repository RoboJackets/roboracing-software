#include <ros/ros.h>
#include <ros/publisher.h>
#include <std_msgs/Float32.h>
#include <string>
#include <iarrc_tutorials/Empty.h>
#include <sensor_msgs/Joy.h>
#include <linux/joystick.h> // may not be necessary
#include <iarrc_msgs/DriveCommand.h>

// TODO: 
// + Access more realistic data structures with ROS headers and such
// 

// Global variables
ros::Publisher float_publisher;

void FloatCommandCB(const sensor_msgs::Joy::ConstPtr& scan_in) {
    // Always be careful!
    if(!scan_in) {
        ROS_ERROR("Received void input pointer in FloatCommandCB.");
    }
    //ROS_INFO("Axes 0: %f !", scan_in->axes[0]); // print axes[1]
    //ROS_INFO("Button 1: %d !", scan_in->buttons[1]); // print button[1]
    iarrc_msgs::DriveCommand StateVehicle; 
    if ((scan_in->buttons[1])>0){ //if the x button is pushed then accelerate to the speed.
        StateVehicle.motor_speed = 50;
    }else{
        StateVehicle.motor_speed = 0;
    }
    if((scan_in->axes[0])>0){ // if pressed left on d-pad
        StateVehicle.servo_position = -30;
    }else if ((scan_in->axes[0])<0){ //if pressed right on d-pad
        StateVehicle.servo_position = 30;
    }else{
        StateVehicle.servo_position = 0;
    }
    ROS_INFO("motor_speed: %f !", StateVehicle.motor_speed); // print axes[1]
    ROS_INFO("servo_position: %f !", StateVehicle.servo_position); // print button[1]
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iarrc_laser_scan");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
   
    std::string scan_topic;    

    nhp.param(std::string("joy"), scan_topic, std::string("/joy"));
   
    ros::Subscriber float_command_sub = nh.subscribe(scan_topic, 1, FloatCommandCB);
    
    ROS_INFO_STREAM("IARRC scan_topic: " << scan_topic);
   
	ROS_INFO("IARRC tutorial node ready.");
	ros::spin();
	ROS_INFO("Shutting down IARRC tutorial node.");

    return 0;
}
