#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <rr_msgs/chassis_state.h>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <rr_platform/EthernetSocket.h>
#include <iostream>

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

double cmd_speed;
double cmd_steering;

std::unique_ptr<rr::EthernetSocket> driveBoardSocket, steeringBoardSocket, manualBoardSocket, estopBoardSocket;

void speedCallback(const rr_msgs::speed::ConstPtr& msg) {
    cmd_speed = msg->speed;
}

void steerCallback(const rr_msgs::steering::ConstPtr& msg) {
    cmd_steering = msg->angle;
}

string messageToString(boost::array<char, 128> buf) {
    return string(buf.begin() + 1, buf.end() - 1); // return useable string, removing start and end markers
}

double extractSpeed(string s) {
    // v=$float,I=$float
    std::string speed_str(s.begin() + s.find('=') + 1, s.begin() + s.find(','));
    double speed = stod(s);
    return speed;
}

double extractSteering(string a) {
    // A=$float
    std::string angle_str(a.begin() + a.find('=') + 1, a.end());
    double angle = stod(angle_str);
    return angle;
}

string formatManualMsg(double speed, double steering) {
    // v=$float,a=$float
    return "v=" + to_string(speed) + ",a=" + to_string(steering);
}

string formatEstopMsg(int command) {
    // G, H
    // TODO when we actually have this data
    return "G";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rigatoni_ethernet_drive_relay");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // // accel PID
    // accelDrivePID.p = nhp.param(string("accel_pid_p"), 0.0);
    // accelDrivePID.i = nhp.param(string("accel_pid_i"), 0.0);
    // accelDrivePID.d = nhp.param(string("accel_pid_d"), 0.0);

    // // Steering PID
    // steeringPID.p = nhp.param(string("steering_pid_p"), 0.0);
    // steeringPID.i = nhp.param(string("steering_pid_i"), 0.0);
    // steeringPID.d = nhp.param(string("steering_pid_d"), 0.0);

    // Setup speed info
    string speedTopic = nhp.param(string("speed_topic"), string("/speed"));
    auto speedSub = nh.subscribe(speedTopic, 1, speedCallback);

    // Setup steering info
    string steerTopic = nhp.param(string("steering_topic"), string("/steering"));
    auto steerSub = nh.subscribe(steerTopic, 1, steerCallback);

    chassisStatePublisher = nh.advertise<rr_msgs::chassis_state>("/chassis_state", 1);
    odometryPublisher = nh.advertise<nav_msgs::Odometry>("/odometry/encoder", 1);

    // IP address and port
    int tcpPort = nhp.param(string("tcp_port"), 7);
    string estopBoardIP = nhp.param(string("estop_ip_address"), string("192.168.0.3"));
    string driveBoardIP = nhp.param(string("drive_ip_address"), string("192.168.0.4"));
    string steeringBoardIP = nhp.param(string("steering_ip_address"), string("192.168.0.5"));
    string manualBoardIP = nhp.param(string("manual_ip_address"), string("192.168.0.6"));

    ROS_INFO_STREAM("[Motor Relay] Trying to connect to TCP Motor Board at " + driveBoardIP + " port: " + std::to_string(tcpPort));
    driveBoardSocket = std::make_unique<rr::EthernetSocket>(driveBoardIP, tcpPort);
    ROS_INFO_STREAM("[Motor Relay] Trying to connect to TCP Steering Board at " + steeringBoardIP + " port: " + std::to_string(tcpPort));
    steeringBoardSocket = std::make_unique<rr::EthernetSocket>(steeringBoardIP, tcpPort);
    ROS_INFO_STREAM("[Motor Relay] Trying to connect to TCP Manual Board at " + manualBoardIP + " port: " + std::to_string(tcpPort));
    manualBoardSocket = std::make_unique<rr::EthernetSocket>(manualBoardIP, tcpPort);
    ROS_INFO_STREAM("[Motor Relay] Trying to connect to TCP E-Stop Board at " + estopBoardIP + " port: " + std::to_string(tcpPort));
    estopBoardSocket = std::make_unique<rr::EthernetSocket>(estopBoardIP, tcpPort);

    ROS_INFO_STREAM("[Motor Relay] Connected to TCP host devices");

    ros::Rate rate(10);
    boost::array<char, 128> driveBuffer, steeringBuffer, manualBuffer, estopBuffer;  // buffer to read response into

    while (ros::ok()) {
        ros::spinOnce();
        // RR Ethernet standard v1.0
        // https://docs.google.com/document/d/10klaJG9QIRAsYD0eMPjk0ImYSaIPZpM_lFxHCxdVRNs/edit#

        // Get Current Speed
        driveBoardSocket->sendMessage("S?");
        // n is the response from socket: 0 means connection closed, otherwise n = num bytes read
        size_t nDrive = driveBoardSocket->readMessage(driveBuffer);  // Check nDrive == 0?
        double currentSpeed = extractSpeed(messageToString(driveBuffer));

        // Get Current Steering
        steeringBoardSocket->sendMessage("A?");
        size_t nSteering = steeringBoardSocket->readMessage(steeringBuffer);  // TODO: Blocking? Maybe should be on difference threads
        double currentSteering = extractSteering(messageToString(steeringBuffer));

        // Send command speed and Steering
        manualBoardSocket->sendMessage(formatManualMsg(cmd_speed, cmd_steering));
        size_t nManual = manualBoardSocket->readMessage(manualBuffer);  // Clear response "R" from buffet

        // Send state to estop
        estopBoardSocket->sendMessage("G");            // RCS_command
        size_t nEstop = estopBoardSocket->readMessage(estopBuffer);  // Clear response "R" from buffet

        // if (nDrive == 0) { //|| nSteering == 0) {
        //     ROS_ERROR_STREAM("[Motor Relay] Connection closed by server");
        //     ros::shutdown();
        // }

        // rr_msgs::chassis_state chassisStateMsg;
        // chassisStateMsg.header.stamp = ros::Time::now();
        // chassisStateMsg.speed_mps = currentSpeed;
        // chassisStateMsg.mux_autonomous = true; //#TODO
        // chassisStateMsg.estop_on = false; //#TODO
        // chassisStatePublisher.publish(chassisStateMsg);

        // // Pose and Twist Odometry Information for EKF localization
        // geometry_msgs::PoseWithCovariance poseMsg;
        // geometry_msgs::TwistWithCovariance twistMsg;

        // nav_msgs::Odometry odometryMsg;
        // odometryMsg.header.stamp = ros::Time::now();
        // odometryMsg.header.frame_id = "odom";
        // odometryMsg.child_frame_id = "base_footprint";
        // odometryMsg.twist.twist.linear.x = chassisStateMsg.speed_mps;
        // odometryMsg.twist.twist.linear.y = 0.0;  // can't move sideways instantaneously
        // #TODO: set twist covariance?
        // #TODO: if need be, use steering for extra data
        // #see https://answers.ros.org/question/296112/odometry-message-for-ackerman-car/
        // odometryPublisher.publish(odometryMsg);


        //reset buffers
        boost::asio::buffer(driveBuffer, nDrive);
        boost::asio::buffer(steeringBuffer, nSteering);
        boost::asio::buffer(manualBuffer, nManual);
        boost::asio::buffer(estopBuffer, nEstop);

        rate.sleep();
    }

    return 0;
}
