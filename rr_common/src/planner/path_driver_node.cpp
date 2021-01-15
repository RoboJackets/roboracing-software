//
// Created by nico on 11/16/20.
//
#include <ros/ros.h>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>
#include <rr_msgs/drive_path.h>

ros::Publisher speed_pub;
ros::Publisher steer_pub;
ros::Subscriber control_sub;
std::vector<rr_msgs::control_vector> curr_drive_path;
std::mutex mtx;
ros::Time curr_path_time;

void drive_path_callback(const rr_msgs::drive_path::ConstPtr& msg) {
    mtx.lock();
    curr_path_time = ros::Time::now();
    curr_drive_path = msg->control_vectors; // this should make a copy
    mtx.unlock();
//    ROS_INFO("callback finished");
}

rr_msgs::control_vector get_path_point(ros::Time t) {
    //get the right control vector given the time that has passed.
    rr_msgs::control_vector prev_vec;
    rr_msgs::control_vector curr_vec;
    ros::Time path_time;
    mtx.lock();
    unsigned int path_size = curr_drive_path.size();
    mtx.unlock();
    // not sure how to handle if curr_drive_path changes mid loop
    for (unsigned int i = 0; i < path_size; i++) {
        prev_vec = curr_vec;
        mtx.lock();
        if (i < curr_drive_path.size()) {
            curr_vec = curr_drive_path[i];
        }
        path_time = curr_path_time;
        mtx.unlock();
        if (path_time + ros::Duration(curr_vec.time) > t) {
            return prev_vec;
        }
    }
    return curr_vec;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_driver");
    ros::NodeHandle nh;
    curr_path_time = ros::Time::now();
    speed_pub = nh.advertise<rr_msgs::speed>("speed", 1);
    steer_pub = nh.advertise<rr_msgs::steering>("steering", 1);
    control_sub = nh.subscribe<rr_msgs::drive_path>("plan/drive_path", 1, drive_path_callback);

    rr_msgs::speed curr_speed;
    rr_msgs::steering curr_angle;
    ros::Rate r(10); // 10hz
    while(ros::ok()) {
        auto now = ros::Time::now();
        ros::spinOnce();
        rr_msgs::control_vector vec = get_path_point(ros::Time::now());
        curr_speed.header.stamp = now;
        curr_angle.header.stamp = now;

        curr_speed.speed = vec.speed;
        curr_angle.angle = vec.angle;

        speed_pub.publish(curr_speed);
        steer_pub.publish(curr_angle);
        r.sleep();
    }

    return 0;
}