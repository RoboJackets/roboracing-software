#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iarrc_tutorial_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

	ros::spin();
    return 0;
}
