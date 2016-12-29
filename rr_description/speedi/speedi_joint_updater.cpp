#include <ros/ros.h>
#include <rr_platform/camera_geometry.h>
#include <sensor_msgs/JointState.h>
#include <boost/algorithm/string.hpp>

using namespace std;

ros::Publisher joint_pub;
string joint_state_path;

double cam_height = 0;
double cam_tilt = 0;


//TODO make this function not janky
void saveJointState() {
    string state = to_string(cam_height) + "|" + to_string(cam_tilt);
    string syscmd = "echo '"+state+"' > "+joint_state_path;
    system(syscmd.c_str());
}

void loadJointState(string &prevState) {
    if(prevState.length() < 3) return;
    vector<string> strs;
    boost::split(strs, prevState, boost::is_any_of(" |\n"));
    if(strs.size() < 2) return;
    cam_height = atof(strs[0].c_str());
    cam_tilt = atof(strs[1].c_str());

    ROS_INFO("SJU set camera tilt to %f", cam_tilt);
}

void camInfoCB(const rr_platform::camera_geometry::ConstPtr &msg) {
    cam_height = msg->height;
    cam_tilt = msg->angle;

    saveJointState();

    ROS_INFO("SJU set camera tilt to %f", cam_tilt);
}

void publishJoints() {
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.header.frame_id = "ground";
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.velocity.resize(2);
    joint_state.effort.resize(2);
    joint_state.name[0] = "cam_ground_height";
    joint_state.position[0] = cam_height;
    joint_state.velocity[0] = 0.0;
    joint_state.effort[0] = 0.0;
    joint_state.name[1] = "camera_tilt";
    joint_state.position[1] = cam_tilt;
    joint_state.velocity[1] = 0.0;
    joint_state.effort[1] = 0.0;
    joint_pub.publish(joint_state);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "speedi_joint_updater");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Subscriber geo_sub = nh.subscribe("/camera_geometry", 1, camInfoCB);
    joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

    nh_private.getParam("joint_state_uri", joint_state_path);

    string prevStateStr;
    nh_private.getParam("loaded_joint_state", prevStateStr);
    loadJointState(prevStateStr);

    ros::Rate rate(30);
    while(ros::ok()) {
        ros::spinOnce();
        publishJoints();
        rate.sleep();
    }

    return 0;
}