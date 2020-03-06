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

ros::Publisher chassisStatePublisher;
ros::Publisher odometryPublisher;

double speed = 0.0;
double steeringAngle = 0.0;

std::unique_ptr<rr::EthernetSocket> driveBoardSocket;
std::unique_ptr<rr::EthernetSocket> steeringBoardSocket;

void speedCallback(const rr_msgs::speed::ConstPtr& msg) {
    speed = msg->speed;
}

void steerCallback(const rr_msgs::steering::ConstPtr& msg) {
    steeringAngle = msg->angle;
}


// Handles working with RJNet for arduino
std::string dataToMessage(double data) {
    return "$" + std::to_string(data) + ";";
}

std::string messageToData(boost::array<char, 128> buf) {
    return std::string(buf.begin() + 1, buf.end() - 1); // return useable string, removing start and end markers
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "rigatoni_ethernet_drive_relay");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Setup speed info
    string speedTopic = nhp.param(string("speed_topic"), string("/speed"));
    auto speedSub = nh.subscribe(speedTopic, 1, speedCallback);

    // accel PID
    accelDrivePID.p = nhp.param(string("accel_pid_p"), 0.0);
    accelDrivePID.i = nhp.param(string("accel_pid_i"), 0.0);
    accelDrivePID.d = nhp.param(string("accel_pid_d"), 0.0);

    // Steering PID
    steeringPID.p = nhp.param(string("steering_pid_p"), 0.0);
    steeringPID.i = nhp.param(string("steering_pid_i"), 0.0);
    steeringPID.d = nhp.param(string("steering_pid_d"), 0.0);

    // Setup steering info
    string steerTopic = nhp.param(string("steering_topic"), string("/steering"));
    auto steerSub = nh.subscribe(steerTopic, 1, steerCallback);

    chassisStatePublisher = nh.advertise<rr_msgs::chassis_state>("/chassis_state", 1);
    odometryPublisher = nh.advertise<nav_msgs::Odometry>("/odometry/encoder", 1);



    // IP address and port
    string driveBoardIP = nhp.param(string("drive_ip_address"), string("192.168.0.177"));
    int driveBoardPort = nhp.param(string("drive_tcp_port"), 7);
    string steeringBoardIP = nhp.param(string("steering_ip_address"), string("192.168.0.3"));
    int steeringBoardPort = nhp.param(string("steering_tcp_port"), 7);

    ROS_INFO_STREAM("[Motor Relay] Trying to connect to TCP Motor Board at " + driveBoardIP + " port: " + std::to_string(driveBoardPort));
    driveBoardSocket  = std::make_unique<rr::EthernetSocket>(driveBoardIP, driveBoardPort);
    //ROS_INFO_STREAM("[Motor Relay] Trying to connect to TCP Steering Board at " + steeringBoardIP + " port: " + std::to_string(steeringBoardPort));
    //steeringBoardSocket = std::make_unique<rr::EthernetSocket>(steeringBoardIP, steeringBoardPort);
    ROS_INFO_STREAM("[Motor Relay] Connected to TCP host devices");


    ros::Rate rate(30);  //#TODO set this value to a good rate time

    size_t nDrive;             // n is the response from socket: 0 means connection closed, otherwise n = num bytes read
    boost::array<char, 128> driveBuffer;  // buffer to read response into
    size_t nSteering;
    boost::array<char, 128> steeringBuffer;


    while (ros::ok()) {
        ros::spinOnce();

        driveBoardSocket->sendMessage(dataToMessage(speed));
        //steeringBoardSocket->sendMessage(dataToMessage(steeringAngle));

        nDrive = driveBoardSocket->readMessage(driveBuffer);
        //nSteering = steeringBoardSocket->readMessage(steeringBuffer);
        //ROS_INFO_STREAM(driveBuffer);

        if (nDrive == 0) { //|| nSteering == 0) {
            ROS_ERROR_STREAM("[Motor Relay] Connection closed by server");
            ros::shutdown();
        }

        double currentSpeed = std::stof(messageToData(driveBuffer));

        rr_msgs::chassis_state chassisStateMsg;
        chassisStateMsg.header.stamp = ros::Time::now();
        chassisStateMsg.speed_mps = currentSpeed;
        chassisStateMsg.mux_autonomous = true; //#TODO
        chassisStateMsg.estop_on = false; //#TODO
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
        // #TODO: set twist covariance?
        // #TODO: if need be, use steering for extra data
        // #see https://answers.ros.org/question/296112/odometry-message-for-ackerman-car/
        odometryPublisher.publish(odometryMsg);




        //reset buffers
        boost::asio::buffer(driveBuffer, nDrive);
        //boost::asio::buffer(steeringBuffer, nSteering);

        rate.sleep();
    }

    return 0;
}
