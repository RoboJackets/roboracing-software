//
// Created by nico on 11/16/20.
//
#include <ros/ros.h>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>
#include <rr_msgs/drive_path.h>
ros::Publisher speed_pub, steer_pub;
ros::Subscriber control_sub;
std::vector<rr_msgs::control_vector> curr_drive_path;
std::mutex mtx;
ros::Time recv_drive_path;
unsigned int curr_index;
void drive_path_callback(const rr_msgs::drive_path::ConstPtr& msg) {
    if (!msg->control_vectors.empty()) {
        mtx.lock();
        recv_drive_path = ros::Time::now();
        curr_drive_path = msg->control_vectors;
        curr_index = 0;
        mtx.unlock();
    }
}
std::pair<double, double> get_path_point(ros::Time t) {
    mtx.lock();
    std::vector<rr_msgs::control_vector> local_drive_path = curr_drive_path;
    unsigned int local_index = curr_index;
    mtx.unlock();
//    auto send_control = local_drive_path[curr_index];
    rr_msgs::control_vector send_control;
    for (unsigned int i = local_index; i < local_drive_path.size(); i++) {
        if (i == local_drive_path.size() - 1 || t < recv_drive_path + ros::Duration(local_drive_path[i + 1].time)) {
            send_control = local_drive_path[i];
            //why no mtx for this current index?
            mtx.lock();
            curr_index = i;
            mtx.unlock();
            break;
        }
    }
    return std::make_pair(send_control.speed, send_control.angle);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "path_driver");
    ros::NodeHandle nh;
    recv_drive_path = ros::Time::now();
    speed_pub = nh.advertise<rr_msgs::speed>("speed", 1);
    steer_pub = nh.advertise<rr_msgs::steering>("steering", 1);
    control_sub = nh.subscribe<rr_msgs::drive_path>("plan/drive_path", 1, drive_path_callback);
    rr_msgs::speed curr_speed;
    rr_msgs::steering curr_angle;
    ros::Rate r(100); // 10hz
    while(ros::ok()) {
        ros::spinOnce();
        auto now = ros::Time::now();
        std::tie(curr_speed.speed, curr_angle.angle) = get_path_point(now);
        curr_speed.header.stamp = now;
        curr_angle.header.stamp = now;
        speed_pub.publish(curr_speed);
        steer_pub.publish(curr_angle);
        r.sleep();
    }
    return 0;
}