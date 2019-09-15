#include <ros/ros.h>
#include <rr_platform/chassis_state.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <iostream>

/*@NOTE THIS CODE USES BOOST 1.58 as it is the version currently installed with
  ROS. If that changes, this code will need to be updated as such! So don't fear
  if it breaks, just fix the things by checking the links below to examples
  given.
*/

using namespace std;
using boost::asio::ip::tcp;

struct PIDConst {
    float p;
    float i;
    float d;
};

struct PIDConst accelDrivePID, decelDrivePID, steeringPID;

double speed = 0.0;
double steeringAngle = 0.0;

double maxAngleMsg;
const double maxOutput = 1.0;

std::unique_ptr<tcp::socket> currentSocket;

void speedCallback(const rr_platform::speed::ConstPtr& msg) {
    speed = msg->speed;
}

void steerCallback(const rr_platform::steering::ConstPtr& msg) {
    steeringAngle = msg->angle / maxAngleMsg * maxOutput;  // Taken from old relay
}

string readMessage() {
    // read data from TCP connection
    boost::array<char, 128> buf;
    boost::system::error_code error;

    size_t len = currentSocket->read_some(boost::asio::buffer(buf), error);
    string reading(buf.begin(), buf.end());  // convert buffer into useable string

    if (error == boost::asio::error::eof) {
        // Connection closed cleanly by peer
        ROS_ERROR_STREAM("TCP Connection closed by peer: Disconnecting");
    } else if (error) {
        ROS_ERROR_STREAM("TCP ERROR: Disconnecting");
        throw boost::system::system_error(error);  // Some other error
    }

    return reading;
}

void sendMessage(string message) {
    boost::array<char, 128> buf;
    boost::system::error_code error;

    // write data to TCP connection
    boost::asio::write(*currentSocket, boost::asio::buffer(message),
                       error);  //#TODO: error check????
}

string buildPIDMessage(const PIDConst& pid) {
    // combines PID into useful message
    string command = to_string(pid.p) + " " + to_string(pid.i) + " " + to_string(pid.d);

    return command;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bigoli_ethernet_drive_relay");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Setup speed info
    string speedTopic = nhp.param(string("speedTopic"), string("/speed"));
    auto speedSub = nh.subscribe(speedTopic, 1, speedCallback);

    // accel PID
    accelDrivePID.p = nhp.param(string("accel_pid_p"), 0.0);
    accelDrivePID.i = nhp.param(string("accel_pid_i"), 0.0);
    accelDrivePID.d = nhp.param(string("accel_pid_d"), 0.0);
    // decel PID
    decelDrivePID.p = nhp.param(string("decel_pid_p"), 0.0);
    decelDrivePID.i = nhp.param(string("decel_pid_i"), 0.0);
    decelDrivePID.d = nhp.param(string("decel_pid_d"), 0.0);

    // Steering PID
    steeringPID.p = nhp.param(string("steering_pid_p"), 0.0);
    steeringPID.i = nhp.param(string("steering_pid_i"), 0.0);
    steeringPID.d = nhp.param(string("steering_pid_d"), 0.0);

    // Setup steering info
    string steerTopic = nhp.param(string("steeringTopic"), string("/steering"));
    auto steerSub = nh.subscribe(steerTopic, 1, steerCallback);

    maxAngleMsg = nhp.param(string("max_angle_msg_in"), 1.0);

    // IP address and port
    string serverName = nhp.param(string("ip_address"), string("192.168.2.2"));
    string serviceName = nhp.param(string("tcp_port"), string("7"));

    // wait for microcontroller to start
    //    ros::Duration(2.0).sleep(); //#TODO: do we need this anymore? Doesn't
    //    seem like it but may be safe

    // TCP client setup
    /*@Note
      https://www.boost.org/doc/libs/1_42_0/doc/html/boost_asio/tutorial/tutdaytime1.html
      this code changed based on the version of boost being used by ROS.
      Currently from 1.58 to 1.68 that looks like using io_context instead of
      io_service Look at the updated tutorial by following link and change as need
      be.
    */
    ROS_INFO_STREAM("Trying to connect to TCP Host at " + serverName + " port: " + serviceName);

    boost::asio::io_service io_service;  //@NOTE: in later versions of boost, this
                                         // may have a name change
    tcp::resolver resolver(io_service);
    tcp::resolver::query query(serverName, serviceName);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    currentSocket = make_unique<tcp::socket>(io_service);  // used to allow us to pass socket to functions
    boost::asio::connect(*currentSocket, endpoint_iterator);

    // CONNECTION NOW OPEN, READY TO JAM
    ROS_INFO_STREAM("Connected to TCP Host");

    // send initialization message (PIDs) in order: accel, deccel, steering
    // combines PID into useful message; # means init message; EX: # p i d p i d p
    // i d
    string pidMessage = "#" + buildPIDMessage(accelDrivePID) + " " + buildPIDMessage(decelDrivePID) + " " +
                        buildPIDMessage(steeringPID);

    sendMessage(pidMessage);
    ROS_INFO_STREAM("Sent PID: " + pidMessage);
    string response = readMessage();  // Should say PID Received //#TODO: have a
                                      // check for correct responses?

    ros::Rate rate(50);  //#TODO set this value to a good rate time (50hz seems
                         // good)/ do we need this?
    while (ros::ok()) {
        ros::spinOnce();

        if (response.at(0) == '@') {
            /*sometimes readMessage returns blank, unknown cause #TODO
             *This if keep us from sending unused data that backs up and causes delays
             *Using the @ symbols also gives us handy status check info too
             */
            ROS_INFO_STREAM("RESPONSE: " + response);

            // write data to MBED
            // Send Motor Command and Steering Command. EX: $speed steeringAngle
            string command =
                  "$" + to_string(speed) + " " + to_string(steeringAngle);  //#TODO: the firmware may need to
                                                                            // be clearing the buffer properly
            sendMessage(command);
            ROS_INFO_STREAM("SENT:" + command);
            response = "";  // clear for next check
        }

        // read data from MBED
        response = readMessage();  //#TODO: make this useful data

        rate.sleep();
    }

    return 0;
}
