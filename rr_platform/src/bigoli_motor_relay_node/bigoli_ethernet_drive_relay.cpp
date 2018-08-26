#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/chassis_state.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <iostream>

/*@NOTE THIS CODE USES BOOST 1.58 as it is the version currently installed with ROS.
  If that changes, this code will need to be updated as such!
  So don't fear if it breaks, just fix the things by checking the links to examples given.
*/


using namespace std;
using boost::asio::ip::tcp;

double speed = 0.0;
double steeringAngle = 0.0;

double maxAngleMsg;
const double maxOutput = 1.0;  //#TODO: magic # to launch param

void speedCallback(const rr_platform::speed::ConstPtr &msg) {
    speed = msg->speed;
}

void steerCallback(const rr_platform::steering::ConstPtr &msg) {
    steeringAngle = msg->angle / maxAngleMsg * maxOutput;
}

string readMessage(tcp::socket socket) {
  //read data from TCP connection
  boost::array<char, 128> buf;
  boost::system::error_code error;

  size_t len = socket.read_some(boost::asio::buffer(buf), error);
  string reading(buf.begin(), buf.end()); //convert buffer into useable string

  if (error == boost::asio::error::eof) { //#TODO THIS ERROR GET OUT OF HERE MAY NEED TO CHANGE as recieving nothing = dead
    // Connection closed cleanly by peer
    ROS_INFO_STREAM("TCP Connection closed by peer: Disconnecting"); //#TODO: not sure what to do or just leave it orr.
  } else if (error) {
    ROS_ERROR_STREAM("TCP ERROR: Disconnecting");
    throw boost::system::system_error(error); // Some other error
  }

  return reading;
}

void sendMessage(tcp::socket socket, string message) {
  boost::array<char, 128> buf;
  boost::system::error_code error;

  //write data to TCP connection
  boost::asio::write(socket, boost::asio::buffer(message), error); //#TODO: error check????
}

string buildMessage(double message, double pid_p, double pid_i, double pid_d) {
  //combines strings into one useful message #TODO: comma seperated or space or ?
  stringstream ss;
  ss << "$" << message << "," << pid_p << "," << pid_i << "," << pid_d;
  string command;
  ss >> command;

  return command;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "bigoli_ethernet_drive_relay");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    //Setup speed info
    string speedTopic = nhp.param(string("speedTopic"), string("/speed"));
    auto speedSub = nh.subscribe(speedTopic, 1, speedCallback);

    float speed_pid_p = nhp.param(string("speed_pid_p"), 0.0);
    float speed_pid_i = nhp.param(string("speed_pid_i"), 0.0);
    float speed_pid_d = nhp.param(string("speed_pid_d"), 0.0);

    //Setup steering info
    string steerTopic = nhp.param(string("steeringTopic"), string("/steering"));
    auto steerSub = nh.subscribe(steerTopic, 1, steerCallback);

    maxAngleMsg = nhp.param(string("max_angle_msg_in"), 1.0);

    float steering_pid_p = nhp.param(string("steering_pid_p"), 0.0);
    float steering_pid_i = nhp.param(string("steering_pid_i"), 0.0);
    float steering_pid_d = nhp.param(string("steering_pid_d"), 0.0);



    // wait for microcontroller to start
    ros::Duration(2.0).sleep();

    //TCP client setup
    /*@Note https://www.boost.org/doc/libs/1_42_0/doc/html/boost_asio/tutorial/tutdaytime1.html
      this code changed based on the version of boost being used by ROS.
      Currently from 1.58 to 1.68 that looks like using io_context instead of io_service
      Look at the updated tutorial by following link and change as need be.
    */
    ROS_INFO_STREAM("Trying to connect to TCP Host");

    string serverName = "192.168.2.2"; //ip #TODO ip .. is magic number
    string serviceName = "7"; //port #TODO port .. is magic
    boost::asio::io_service io_service;
    tcp::resolver resolver(io_service);
    tcp::resolver::query query(serverName, serviceName);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    tcp::socket socket(io_service);
    boost::asio::connect(socket, endpoint_iterator);


    //CONNECTION NOW OPEN, READY TO JAM
    ROS_INFO_STREAM("Connected to TCP Host");

    ros::Rate rate(10); //#TODO set this value to a good rate time
    while(ros::ok()) {
        ros::spinOnce();

        boost::array<char, 128> buf;
        boost::system::error_code error;

        //write data to MBED
        //Send Motor Command
        string speedCommand = buildMessage(speed, speed_pid_p, speed_pid_i, speed_pid_d);
        sendMessage(socket, speedCommand);
        //Send Steering Command
        string steeringCommand = buildMessage(steeringAngle, steering_pid_p, steering_pid_i, steering_pid_d);
        sendMessage(socket, steeringCommand);

        //read data from MBED
        //string response = readMessage(socket)


        rate.sleep();
    }

    return 0;
}
