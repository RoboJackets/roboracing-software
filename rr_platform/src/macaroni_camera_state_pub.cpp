#include <ros/ros.h>
#include <rr_platform/camera_pose.h>
#include <sensor_msgs/JointState.h>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <std_msgs/Float64.h>

using namespace std;

bool USING_GAZEBO;

ros::Publisher joint_state_pub;
ros::Publisher height_command_pub;
ros::Publisher tilt_command_pub;
string joint_state_path;

double cam_height = 0; //height of camera from chassis
double cam_tilt = 0;


void saveJointState() {
    ofstream out(joint_state_path);
    out << cam_height << "|" << cam_tilt;
}

void loadJointState(string &prevState) {
    if(prevState.length() < 3) return;
    vector<string> strs;
    boost::split(strs, prevState, boost::is_any_of(" |\n"));
    if(strs.size() < 2) return;
    cam_height = atof(strs[0].c_str());
    cam_tilt = atof(strs[1].c_str());

    ROS_INFO("Set camera tilt to %f", cam_tilt);
}

void camInfoCB(const rr_platform::camera_pose::ConstPtr &msg) {
    cam_height = msg->height;
    cam_tilt = msg->angle;

    saveJointState();

    ROS_INFO("Set camera tilt to %f", cam_tilt);
}

void publishJointState() {
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.header.frame_id = "base_footprint";
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.velocity.resize(2);
    joint_state.effort.resize(2);
    joint_state.name[0] = "cam_chassis_height";
    joint_state.position[0] = cam_height;
    joint_state.velocity[0] = 0.0;
    joint_state.effort[0] = 0.0;
    joint_state.name[1] = "camera_tilt";
    joint_state.position[1] = cam_tilt;
    joint_state.velocity[1] = 0.0;
    joint_state.effort[1] = 0.0;
    joint_state_pub.publish(joint_state);
}

void publishJointCommands() {
    std_msgs::Float64 tilt_msg;
    tilt_msg.data = cam_tilt;
    tilt_command_pub.publish(tilt_msg);

    std_msgs::Float64 height_msg;
    height_msg.data = cam_height;
    height_command_pub.publish(height_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "macaroni_joint_updater");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.getParam("joint_state_uri", joint_state_path);

    string prevStateStr;
    nh_private.getParam("loaded_joint_state", prevStateStr);
    loadJointState(prevStateStr);

    nh_private.getParam("use_simulator_control", USING_GAZEBO);

    ros::Subscriber geo_sub = nh.subscribe("/camera_pose", 1, camInfoCB);
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

    height_command_pub = nh.advertise<std_msgs::Float64>("/camera_height_position_controller/command", 1, true);

    tilt_command_pub = nh.advertise<std_msgs::Float64>("/camera_tilt_position_controller/command", 1, true);

    ros::Rate rate(30);
    while(ros::ok()) {
        ros::spinOnce();

        if(USING_GAZEBO) {
            publishJointCommands();
        } else {
            publishJointState();
        }

        rate.sleep();
    }

    return 0;
}

