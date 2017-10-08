#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/chassis_state.h>
#include <boost/asio.hpp>

using namespace std;

double output = 0;
double maxAngleMsg;
const double maxOutput = 1.0;

void sendCommand(boost::asio::serial_port &port) {
    string message = "$" + std::to_string(output) + "\n";
    try {
        boost::asio::write(port, boost::asio::buffer(message.c_str(), message.size()));
    } catch (boost::system::system_error &err) {
        ROS_ERROR("%s", err.what());
    }
}

// copied from motor_relay_node
string readLine(boost::asio::serial_port &port) {
    std::string line = "";
    bool inLine = false;
    while (true) {
        char in;
        try {
            boost::asio::read(port, boost::asio::buffer(&in, 1));
        } catch (
                boost::exception_detail::clone_impl <boost::exception_detail::error_info_injector<boost::system::system_error>> &err) {
            ROS_ERROR("Error reading serial port.");
            ROS_ERROR_STREAM(err.what());
            return line;
        }
        if (!inLine && in == '$')
            inLine = true;
        if (inLine) {
            if (in == '\n') {
                return line;
            }
            if (in == '\r') {
                return line;
            }
            line += in;
        }
    }
}


void steerCallback(const rr_platform::steering::ConstPtr &msg) {
    output = msg->angle / maxAngleMsg * maxOutput;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bigoli_tester");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    auto speedSub = nh.subscribe("/steering", 1, steerCallback);

    maxAngleMsg = nhp.param("max_angle_msg_in", 1.0);

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
        sendCommand(serial);
        string response = readLine(serial);
        ROS_INFO_STREAM("steer relay sent " << output << ", received " << response);
        rate.sleep();
    }

    serial.close();
    return 0;
}
