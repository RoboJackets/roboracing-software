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
double steering = 0.0;

void speedCallback(const rr_platform::speed::ConstPtr &msg) {
    speed = msg->speed;
}

void steeringCallback(const rr_platform::steering::ConstPtr &msg) {
    steering = msg->angle;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bigoli_ethernet_drive_relay");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    string speedTopic = nhp.param(string("topic"), string("/speed"));
    auto speedSub = nh.subscribe(speedTopic, 1, speedCallback);

    //SUBSCRIBE HERE #TODO


    // wait for microcontroller to start #TODO need this?
    ros::Duration(2.0).sleep();

    //TCP client setup
    /*@Note https://www.boost.org/doc/libs/1_42_0/doc/html/boost_asio/tutorial/tutdaytime1.html */
    ROS_INFO_STREAM("Trying to connect to TCP Host");

    string serverName = "192.168.2.1"; //#TODO ip
    string serviceName = "7"; //#TODO port
    boost::asio::io_context io_context;
    tcp::resolver resolver(io_context);
    tcp::resolver::results_type endpoints = resolver.resolve(serverName, serviceName);
    tcp::socket socket(io_context);
    boost::asio::connect(socket, endpoints);

    //CONNECTION NOW OPEN, READY TO READ FROM
    ROS_INFO_STREAM("Connected to TCP Host!");

    ros::Rate rate(10); //#TODO set rate time
    while(ros::ok()) {
        ros::spinOnce();

        boost::array<char, 128> buf;
        boost::system::error_code error;

        //read data from MBED
        size_t len = socket.read_some(boost::asio::buffer(buf), error);
        if (error == boost::asio::error::eof) {
          break; // Connection closed cleanly by peer
        }
        else if (error) {
          throw boost::system::system_error(error); // Some other error
        }

        //write data to MBED
        string message = "Hello,world";
        boost::asio::write(socket, boost::asio::buffer(message), error); //#TODO: error check????


        rate.sleep();
    }

    return 0;
}
