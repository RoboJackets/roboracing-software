#include <ros/ros.h>
#include <rr_platform/camera_geometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

using namespace std;

ros::Publisher joint_pub;

double cam_height = 0;
double cam_tilt = 0;

void camInfoCB(const rr_platform::camera_geometry::ConstPtr &msg) {
    cam_height = msg->height;
    cam_tilt = msg->angle;
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
    ros::Subscriber geo_sub = nh.subscribe("/camera_geometry", 1, camInfoCB);
    joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

    ros::Rate rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        publishJoints();
        rate.sleep();
    }

    return 0;
}