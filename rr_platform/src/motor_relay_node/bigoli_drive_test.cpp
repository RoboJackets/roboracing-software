#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/chassis_state.h>
#include <boost/asio.hpp>

using namespace std;

double speed = 0;

void sendCommand(boost::asio::serial_port &port) {
    string message = "$" + std::to_string(speed) + "\n";
    try {
        boost::asio::write(port, boost::asio::buffer(message.c_str(), message.size()));
    } catch (boost::system::system_error &err) {
        ROS_ERROR("%s", err.what());
    }
}

void speedCallback(const rr_platform::speed::ConstPtr &msg) {
    speed = msg->speed;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bigoli_tester");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    auto speedSub = nh.subscribe("/speed", 1, speedCallback);

    // Serial port setup
    string serial_port_name;
    nhp.param(std::string("serial_port"), serial_port_name, std::string("/dev/ttyACM0"));
    boost::asio::io_service io_service;
    boost::asio::serial_port serial(io_service, serial_port_name);
    serial.set_option(boost::asio::serial_port_base::baud_rate(9600));

    ros::Rate rate(10);

    // wait for microcontroller to start
    ros::Duration(2.0).sleep();

    while(ros::ok() && serial.is_open()) {
        ros::spinOnce();
        ROS_INFO("sending command %f", speed);
        sendCommand(serial);
        rate.sleep();
    }

    serial.close();
    return 0;
}
