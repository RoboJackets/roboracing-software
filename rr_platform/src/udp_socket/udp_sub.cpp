#include <rr_platform/udp_sock.h>
#include <ros/ros.h>

using std::string;

const string myArress = "192.168.20.2";
const int myPort = 8888;

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_sub_node");
    ros::NodeHandle nh;

    const std::unique_ptr<rr::udp_socket> socket = std::make_unique<rr::udp_socket>(myArress, myPort);
    socket->bind();

    ros::Rate rate(10);
    while (ros::ok) {
        string msg = socket->recv();
        ROS_INFO_STREAM("Received: " << msg);
        rate.sleep();
    }

}