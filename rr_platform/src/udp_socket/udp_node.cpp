#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rr_msgs/chassis_state.h>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>
#include <rr_platform/udp_socket.h>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
#include <thread>

using namespace boost::asio;
using ip::tcp;
using std::cout;
using std::endl;
using std::string;

/*@
    NOTE THIS CODE USES BOOST 1.58 as it is the version currently installed with
    ROS. If that changes, this code will need to be updated as such! So don't fear
    if it breaks, just fix the things by checking the links below to examples
    given.
*/

using namespace std;

ros::Publisher chassisStatePublisher, odometryPublisher;

double cmd_speed = 0;
double cmd_steering = 0;

std::unique_ptr<rr::udp_client> driveBoardSocket, steeringBoardSocket, manualBoardSocket, estopBoardSocket;

void speedCallback(const rr_msgs::speed::ConstPtr& msg) {
    cmd_speed = msg->speed;
}

void steerCallback(const rr_msgs::steering::ConstPtr& msg) {
    cmd_steering = msg->angle;
}

string messageToString(boost::array<char, 128> buf) {
    return string(buf.begin() + 1, buf.end() - 1);  // return useable string, removing start and end markers
}

double extractSpeed(string s) {
    // v=$float I=$float
    double speed = 0.0;

    // Get interval from 'v=$' up to the next space
    std::string speed_str(s.begin() + s.find('=') + 2, s.begin() + s.find(' '));
    if (!speed_str.empty()) {
        speed = stod(speed_str);
    } else {
        return -1;
    }
    return speed;
}

double extractSteering(string a) {
    // A=$float
    double angle = 0.0;
    std::string angle_str(a.begin() + a.find('=') + 2, a.end());
    if (!angle_str.empty()) {
        angle = stod(angle_str);
    }
    return angle;
}

bool extractEstop(string s) {
    if (s == "$G;") {
        return false;  // estop not on
    } else {
        return true;
    }
}

string formatManualMsg(double speed, double steering) {
    // v=$float,a=$float
    return "$v=" + to_string(speed) + ",a=" + to_string(steering) + ";";
}

string formatEstopMsg(int command) {
    // G, H
    // TODO when we actually have this data
    return "G";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ethernet_drive_relay");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Setup speed info
    string speedTopic = nhp.param(string("speed_topic"), string("/speed"));
    ros::Subscriber speedSub = nh.subscribe(speedTopic, 1, speedCallback);

    // Setup steering info
    string steerTopic = nhp.param(string("steering_topic"), string("/steering"));
    ros::Subscriber steerSub = nh.subscribe(steerTopic, 1, steerCallback);

    chassisStatePublisher = nh.advertise<rr_msgs::chassis_state>("/chassis_state", 1);
    odometryPublisher = nh.advertise<nav_msgs::Odometry>("/odometry/encoder", 1);

    // IP address and port
    int udpPort = nhp.param(string("udp_port"), 7);
    string estopBoardIP = nhp.param(string("estop_ip_address"), string("192.168.20.3"));
    string driveBoardIP = nhp.param(string("drive_ip_address"), string("192.168.20.4"));
    string steeringBoardIP = nhp.param(string("steering_ip_address"), string("192.168.20.5"));
    string manualBoardIP = nhp.param(string("manual_ip_address"), string("192.168.20.6"));
    string brakeIP = nhp.param(string("brake_ip_address"), string("192.168.20.7"));
    string batteryMonitorIP = nhp.param(string("battery_monitor_ip_address"), string("192.168.20.11"));

    ROS_INFO_STREAM("[Motor Relay] Connecting to UDP Drive Board at " + driveBoardIP +
                    " port: " + std::to_string(udpPort));
    
    driveBoardSocket = std::make_unique<rr::udp_client>(driveBoardIP, udpPort);

    ROS_INFO_STREAM("[Motor Relay] Connecting to UDP steering board at " + steeringBoardIP +
                    " port: " + std::to_string(udpPort));

    steeringBoardSocket = std::make_unique<rr::udp_client>(steeringBoardIP, udpPort);

    ROS_INFO_STREAM("[Motor Relay] Connecting to UDP E-Stop Board at " + estopBoardIP +
                    " port: " + std::to_string(udpPort));

    estopBoardSocket = std::make_unique<rr::udp_client>(estopBoardIP, udpPort);

    ROS_INFO_STREAM("[Motor Relay] Initialized connection to all UDP host devices");

    ros::Rate rate(10);

    // Send state to estop
    estopBoardSocket->send("$G;");
    string estop_response = estopBoardSocket->read();
    ROS_INFO_STREAM("Estop Receiving: " << estop_response);
    bool current_estop = extractEstop(estop_response);

    while (ros::ok()) {
        ros::spinOnce();

        // RR Ethernet standard v2.0
        // https://docs.google.com/document/d/1oRHm5xBQiod_YXESQ9omFy5joJqMfeQjY3NCvdzRTjk/edit#heading=h.81mhflsq1wxv

        // Get Current Speed
        // It should look something like "$v=5.0;";
        driveBoardSocket->send("$v=" + std::to_string(cmd_speed) + ";");  //
        string drive_response = driveBoardSocket->read();                 // Clear response "R" from buffet
        ROS_INFO_STREAM("Drive Receiving: " << drive_response);
        double current_speed = extractSpeed(drive_response);

        // Get Current Steering
        // It should look something like "$A=30;" (in radians)
        steeringBoardSocket->send("$A=" + std::to_string(cmd_steering) + ";");
        string steering_response = steeringBoardSocket->read();  // Clear response "R" from buffer
        ROS_INFO_STREAM("Steering Receiving: " << steering_response);
        double current_steer;
        if (steering_response != "TIME_OUT") {
            current_steer = extractSteering(steering_response);
        } else {
            ROS_ERROR("Steering Time Out");
        }

        rr_msgs::chassis_state chassisStateMsg;
        chassisStateMsg.header.stamp = ros::Time::now();
        chassisStateMsg.speed_mps = (float)current_speed;
        chassisStateMsg.steer_rad = (float)current_steer;
        chassisStateMsg.mux_autonomous = true;  //#TODO
        chassisStateMsg.estop_on = current_estop;
        chassisStatePublisher.publish(chassisStateMsg);

        // Pose and Twist Odometry Information for EKF localization
        geometry_msgs::PoseWithCovariance poseMsg;
        geometry_msgs::TwistWithCovariance twistMsg;

        nav_msgs::Odometry odometryMsg;
        odometryMsg.header.stamp = ros::Time::now();
        odometryMsg.header.frame_id = "odom";
        odometryMsg.child_frame_id = "base_footprint";
        odometryMsg.twist.twist.linear.x = chassisStateMsg.speed_mps;
        odometryMsg.twist.twist.linear.y = 0.0;  // can't move sideways instantaneously
                                                 // // #TODO: set twist covariance?
                                                 // // #TODO: if need be, use steering for extra data
        // // #see https://answers.ros.org/question/296112/odometry-message-for-ackerman-car/
        odometryPublisher.publish(odometryMsg);

        rate.sleep();
    }

    return 0;
}
