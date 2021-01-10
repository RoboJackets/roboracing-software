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
//boost::weak_ptr<rr_msgs::drive_path> cur_msg;
//rr_msgs::drive_path_<std::allocator<void>> *cur_msg;
//rr_msgs::drive_path_<std::allocator<void>>::_control_vectors_type cur_drive_path;
std::vector<rr_msgs::control_vector> cur_drive_path;
std::mutex mtx;
unsigned int cur_index = 0;
double cur_dt = 1;

void drive_path_callback(const rr_msgs::drive_path::ConstPtr& msg) {
    ROS_INFO("called back");
    //todo: use mutex to make sure the number not multithread garbage corrupt
    mtx.lock();
    cur_index = 0;
//    cur_msg = msg;
//    cur_msg = msg.get();
//    cur_drive_path = const_cast<rr_msgs::drive_path_<std::allocator<void>>::_control_vectors_type *>(msg->control_vectors);
    cur_drive_path = msg->control_vectors; // this should make a copy
    cur_dt = msg->dt;
//    *rate = ros::Rate(msg->dt); // I cant remember if this works as expected or not.
    mtx.unlock();
}

//void drive_path(const rr_msgs::drive_path::ConstPtr& msg) {
//    ros::Rate r(msg->dt); // must be same dt as bike method
//    for (const auto& control_msg : msg->control_vectors) {
//        speed_pub.publish(control_msg.speed);
//        steer_pub.publish(control_msg.angle);
//        r.sleep();
//    }
//}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_driver");
    ros::NodeHandle nh;
    speed_pub = nh.advertise<rr_msgs::speed>("plan/speed", 1);
    steer_pub = nh.advertise<rr_msgs::steering>("plan/steering", 1);
    control_sub = nh.subscribe<rr_msgs::drive_path>("plan/drive_path", 1, drive_path_callback);
    ros::AsyncSpinner spinner(1); //Use 1 thread
    spinner.start();
    while(ros::ok()) {
        ros::Rate r(1 / cur_dt);
        ROS_INFO("Current dt: %f", 1 / cur_dt);
        mtx.lock();
        if (cur_index < cur_drive_path.size()) {
            speed_pub.publish(cur_drive_path[cur_index].speed);
            steer_pub.publish(cur_drive_path[cur_index++].angle);
            ROS_INFO("Speed and angle published");
            r.sleep();
        }
        mtx.unlock();
    }

    return 0;
}