#include <ros/ros.h>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>
#include <rr_platform/udp_sock.h>

using std::string;

double cmd_speed = 0;
double cmd_steering = 0;
const string steeringAddress = "192.168.20.3";
const string speedAddress = "192.168.20.4";
const int port = 8888;

void speedCallback(const rr_msgs::speed::ConstPtr& msg) {
    cmd_speed = msg->speed;
}

void steerCallback(const rr_msgs::steering::ConstPtr& msg) {
    cmd_steering = msg->angle;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_pub_node");
    ros::NodeHandle nh;

    ros::Subscriber speedSub = nh.subscribe("/speed", 1, speedCallback);
    ros::Subscriber steerSub = nh.subscribe("/steering", 1, steerCallback);

    const std::unique_ptr<rr::udp_socket> steeringSocket = std::make_unique<rr::udp_socket>(steeringAddress, port);
    const std::unique_ptr<rr::udp_socket> speedSocket = std::make_unique<rr::udp_socket>(speedAddress, port);

    while (ros::ok()) {
        ros::spinOnce();

        // send speed socket
        string speed_msg = "v=" + std::to_string(cmd_speed);
        speedSocket->send(speed_msg);
        ROS_INFO_STREAM("Sent speed: " << speed_msg);
        // send steering socket
        string steer_msg = "A=" + std::to_string(cmd_steering);
        steeringSocket->send(steer_msg);
        ROS_INFO_STREAM("Sent steering: " << steer_msg);
        ros::Rate rate(10);
    }
}