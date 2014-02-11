#include <ros/ros.h>
#include <ros/publisher.h>
#include <std_msgs/Float32.h>
#include <string>
#include <iarrc_tutorials/Empty.h>


// TODO: 
// + Access more realistic data structures with ROS headers and such
// 

// Global variables
ros::Publisher float_publisher;

void FloatCommandCB(const std_msgs::Float32::ConstPtr& msg) {
    // Always be careful!
    if(!msg) {
        ROS_ERROR("Recieved void input pointer in FloatCommandCB.");
    }

    // Access the float data
    // Remember that the message data structures can be found in the corresponding 
    // .msg files. Try `locate Float32.msg` on your system or google it.
    ROS_INFO("Recieved a float msg of value: %f in callback function!", msg->data);

    // Republish the data onto another topic
    ROS_DEBUG("Publishing this float to the topic: %s, which has %d subscribers.", 
              float_publisher.getTopic().c_str(), float_publisher.getNumSubscribers());
    float_publisher.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iarrc_tutorial_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    std::string float_command_topic;
    std::string float_repub_topic;

    nhp.param(std::string("float_command_topic"), float_command_topic, std::string("/float_command_topic"));
    nhp.param(std::string("float_repub_topic"), float_repub_topic, std::string("/float_repub_topic"));

    ros::Subscriber float_command_sub = nh.subscribe(float_command_topic, 1, FloatCommandCB);
    float_publisher = nh.advertise<std_msgs::Float32>(float_repub_topic, 1);

    ROS_INFO_STREAM("IARRC float_command_topic: " << float_command_topic);
    ROS_INFO_STREAM("IARRC float_repub_topic: " << float_repub_topic);

	ROS_INFO("IARRC tutorial node ready.");
	ros::spin();
	ROS_INFO("Shutting down IARRC tutorial node.");

    return 0;
}
