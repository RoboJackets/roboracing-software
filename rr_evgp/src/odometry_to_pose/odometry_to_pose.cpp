#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher pose_publisher_;
ros::Subscriber pose_subscriber_;

void poseCallback(const nav_msgs::Odometry& odometry) {
    geometry_msgs::PoseWithCovarianceStamped message;
    message.header = odometry.header;
    message.pose = odometry.pose;
    pose_publisher_.publish(message);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_pose_type_converter");
    ros::NodeHandle nhp = ros::NodeHandle("~");
    pose_subscriber_ = nhp.subscribe("/odometry/filtered", 1, poseCallback);
    pose_publisher_ = nhp.advertise<geometry_msgs::PoseWithCovarianceStamped>("/odometry/pose_with_cov_stamped", 1);

    ros::spin();
}