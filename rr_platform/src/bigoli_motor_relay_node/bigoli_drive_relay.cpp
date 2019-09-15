#include <ros/ros.h>
#include <rr_platform/SerialPort.h>
#include <rr_platform/chassis_state.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>

using namespace std;

double output = 0;

void speedCallback(const rr_platform::speed::ConstPtr& msg) {
    output = msg->speed;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bigoli_drive_relay");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    string speedTopic = nhp.param(string("topic"), string("/speed"));
    auto speedSub = nh.subscribe(speedTopic, 1, speedCallback);

    float pid_p = nhp.param(string("pid_p"), 0.0);
    float pid_i = nhp.param(string("pid_i"), 0.0);
    float pid_d = nhp.param(string("pid_d"), 0.0);

    // Serial port setup
    string serial_port_name;
    nhp.param(string("serial_port"), serial_port_name, string("/dev/ttyACM0"));
    SerialPort serial_port;
    if (!serial_port.Open(serial_port_name, 9600)) {
        ROS_FATAL_STREAM("Unable to open serial port: " << serial_port_name);
        return 1;
    }

    // wait for microcontroller to start
    ros::Duration(2.0).sleep();

    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        stringstream ss;
        ss << "$" << output << "," << pid_p << "," << pid_i << "," << pid_d << "\n";
        string command;
        ss >> command;

        serial_port.Write(command);
        string response = serial_port.ReadLine();
        // ROS_INFO_STREAM("drive relay sent " << command << ", received " <<
        // response);
        rate.sleep();
    }

    return 0;
}
