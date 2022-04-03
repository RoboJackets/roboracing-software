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
#include <iostream>
#include <thread>
#include <chrono>

using namespace boost::asio;
using ip::tcp;
using std::cout;
using std::endl;
using std::string;

/*@NOTE THIS CODE USES BOOST 1.58 as it is the version currently installed with
  ROS. If that changes, this code will need to be updated as such! So don't fear
  if it breaks, just fix the things by checking the links below to examples
  given.
*/

using namespace std;

struct PIDConst {
    float p;
    float i;
    float d;
};

struct PIDConst accelDrivePID, steeringPID;

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
    return string(buf.begin()+1, buf.end()-1);  // return useable string, removing start and end markers
}

double extractSpeed(string s) {
    // v=$float,I=$float
    double speed = 0.0;
    std::string speed_str(s.begin() + s.find('=') + 1, s.begin() + s.find(','));
    if (!speed_str.empty()) {
        speed = stod(speed_str);
    }
    return speed;
}

double extractSteering(string a) {
    // A=$float
    double angle = 0.0;
    std::string angle_str(a.begin() + a.find('=') + 1, a.end());
    if (!angle_str.empty()) {
        angle = stod(angle_str);
    }
    return angle;
}

bool extractEstop(string s) {
    if (s == "$G;") { // this could be just G idk
        return false; //estop not on
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
    string estopBoardIP = nhp.param(string("estop_ip_address"), string("192.168.00.3"));
    string driveBoardIP = nhp.param(string("drive_ip_address"), string("192.168.00.4"));
    string steeringBoardIP = nhp.param(string("steering_ip_address"), string("192.168.00.5"));
    // string manualBoardIP = nhp.param(string("manual_ip_address"), string("192.168.00.6"));

    ROS_INFO_STREAM("[Motor Relay] Trying to connect to UDP Motor Board at " + driveBoardIP +
                    " port: " + std::to_string(udpPort));
    try {
        driveBoardSocket = std::make_unique<rr::udp_client>(driveBoardIP, udpPort);
    } catch (...) {
        ROS_ERROR_STREAM("[Motor Relay] Exiting... Failed to connect to motor board at " + driveBoardIP +
                " port: " + std::to_string(udpPort));
        return 1;
    }

    ROS_INFO_STREAM("[Motor Relay] Exiting... Trying to connect to UDP steering board at " + steeringBoardIP +
            " port: " + std::to_string(udpPort));

    try {
        steeringBoardSocket = std::make_unique<rr::udp_client>(steeringBoardIP, udpPort);
    } catch (...) {
        ROS_ERROR_STREAM("[Motor Relay] Exiting... Failed to connect to UDP steering board at " + steeringBoardIP +
                " port: " + std::to_string(udpPort));
        return 1;
    }
    // ROS_INFO_STREAM("[Motor Relay] Trying to connect to TCP Manual Board at " + manualBoardIP +
    //                 " port: " + std::to_string(tcpPort));
    // manualBoardSocket = std::make_unique<rr::EthernetSocket>(manualBoardIP, tcpPort);
    ROS_INFO_STREAM("[Motor Relay] Trying to connect to UDP E-Stop Board at " + estopBoardIP +
                    " port: " + std::to_string(udpPort));

    try {
        estopBoardSocket = std::make_unique<rr::udp_client>(estopBoardIP, udpPort);
    } catch (...) {
        ROS_ERROR_STREAM("[Motor Relay] Exiting... Failed to connect to UDP E-Stop Board at " + steeringBoardIP +
                " port: " + std::to_string(udpPort));
        return 1;
    }

    ROS_INFO_STREAM("[Motor Relay] Connected to UDP host devices");

    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        // RR Ethernet standard v1.0
        // https://docs.google.com/document/d/10klaJG9QIRAsYD0eMPjk0ImYSaIPZpM_lFxHCxdVRNs/edit#

        // Get Current Speed
        // std::string speed_str = "$v=5.0;";
        // driveBoardSocket->send(speed_str);
        driveBoardSocket->send("$v=" + std::to_string(cmd_speed) + ";"); //
        string drive_response = driveBoardSocket->read_with_timeout();  // Clear response "R" from buffet
        ROS_INFO_STREAM("Drive Receiving: " << drive_response);
        double current_speed = extractSpeed(drive_response);

        // Get Current Steering
        steeringBoardSocket->send("$S=" + std::to_string(cmd_steering) + ";");
        string steering_response = steeringBoardSocket->read_with_timeout();  // Clear response "R" from buffet
        ROS_INFO_STREAM("Steering Receiving: " << steering_response);
        double current_steer;
        if (steering_response != "TIME_OUT") {
            current_steer = extractSteering(steering_response);
        } else {
            ROS_ERROR("Steering Time Out");
        }

        // Send command speed and Steering
//        string x = formatManualMsg(3,2);
//        string x = formatManualMsg(cmd_speed, cmd_steering);
//        manualBoardSocket->send(x);
//
//        string manual_response = manualBoardSocket->read_with_timeout();
//        ROS_INFO_STREAM("Manual Receiving: " << manual_response);

    //     Send state to estop
        estopBoardSocket->send("$G;"); //todo, dont always send G maybe?
        string estop_response = estopBoardSocket->read_with_timeout();
        ROS_INFO_STREAM("Estop Receiving: " << estop_response);
        bool current_estop = extractEstop(estop_response);


        rr_msgs::chassis_state chassisStateMsg;
        chassisStateMsg.header.stamp = ros::Time::now();
        chassisStateMsg.speed_mps = (float) current_speed;
        chassisStateMsg.steer_rad = (float) current_steer;
        chassisStateMsg.mux_autonomous = true; //#TODO
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
