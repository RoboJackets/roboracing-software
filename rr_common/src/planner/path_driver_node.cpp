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
std::vector<rr_msgs::control_vector> cur_drive_path;
std::mutex mtx;
unsigned int cur_index = 0;
double cur_dt = 1; // do not use

void drive_path_callback(const rr_msgs::drive_path::ConstPtr& msg) {
    mtx.lock();
    cur_index = 0;
    cur_drive_path = msg->control_vectors; // this should make a copy
    mtx.unlock();

    ROS_INFO("callback finished");
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "path_driver");
    ros::NodeHandle nh;
    speed_pub = nh.advertise<rr_msgs::speed>("speed", 1);
    steer_pub = nh.advertise<rr_msgs::steering>("steering", 1);
    control_sub = nh.subscribe<rr_msgs::drive_path>("plan/drive_path", 1, drive_path_callback);
//    ros::spin();
//    ros::AsyncSpinner spinner(1); //Use 1 thread
//    spinner.start();
//    double curr_speed = 0, curr_angle = 0, curr_dt;
    rr_msgs::speed curr_speed;
    rr_msgs::steering curr_angle;
//    ros::Duration curr_dt;
    double curr_dt = 0;
    while(ros::ok()) {
        ros::spinOnce();
        ros::Rate r(1 / cur_dt); // todo based on the diff in times.
        mtx.lock(); // can only do reads and writes in the lock, no math no sleep no publishes
        if (cur_index < cur_drive_path.size()) {
            curr_speed.speed = cur_drive_path[cur_index].speed;
            curr_angle.angle = cur_drive_path[cur_index].angle;
            if (cur_index + 1 < cur_drive_path.size()) {
                curr_dt = cur_drive_path[cur_index + 1].time - cur_drive_path[cur_index].time;
            }
            cur_index++;
        }
        mtx.unlock();
        auto now = ros::Time::now();
        curr_speed.header.stamp = now;
        curr_angle.header.stamp = now;
        speed_pub.publish(curr_speed); // this needs to be the right type
        steer_pub.publish(curr_angle);
        r.sleep();


    }

    return 0;
}