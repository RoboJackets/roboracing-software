#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <boost/asio.hpp>

int desiredSpeed = 0;
int desiredSteer = 0;
int prevAngle = 0;
int prevSpeed = 0;

const boost::array<double, 9ul> unknown_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

/**
 * @note http://stackoverflow.com/a/27511119
 */
std::vector <std::string> split(const std::string &s, char delim) {
    std::stringstream ss(s);
    std::string item;
    std::vector <std::string> elems;
    while (std::getline(ss, item, delim)) {
        elems.push_back(std::move(item));
    }
    return elems;
}

int PWMFromSpeed(double metersPerSecond) {
    return static_cast<int>(metersPerSecond / 0.18333333);
}

int PWMFromAngle(double radians) {
    if(radians < 0) {
        return static_cast<int>(radians * 0.0436332);
    }
    if(radians > 0) {
        return static_cast<int>(radians * -0.0218166);
    }
    return 0;
}

void SpeedCallback(const rr_platform::speed::ConstPtr &msg) {
    desiredSpeed = PWMFromSpeed(msg->speed);
}

void SteeringCallback(const rr_platform::steering::ConstPtr &msg) {
    desiredSteer = PWMFromAngle(msg->angle);
    ROS_INFO_STREAM("Desired steer: " << desiredSteer);
}

void sendCommand(boost::asio::serial_port &port) {
    std::string message = "$" + std::to_string(static_cast<char>(desiredSpeed)) + ", " +
                          std::to_string(static_cast<char>(desiredSteer)) + "\n";
    
    try {
        boost::asio::write(port, boost::asio::buffer(message.c_str(), message.size()));
    } catch (boost::system::system_error &err) {
        ROS_ERROR("%s", err.what());
    }
}

std::string readLine(boost::asio::serial_port &port) {
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
        if(inLine) {
            if (in == '\n')
                return line;
            if (in == '\r')
                return line;
            line += in;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "iarrc_motor_relay_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Subscribers
    std::string speed_topic_name;
    nhp.param(std::string("speed_topic"), speed_topic_name, std::string("/speed"));
    ros::Subscriber speed_sub = nh.subscribe(speed_topic_name, 1, SpeedCallback);

    std::string steering_topic_name;
    nhp.param(std::string("steering_topic"), steering_topic_name, std::string("/steering"));
    ros::Subscriber steering_sub = nh.subscribe(steering_topic_name, 1, SteeringCallback);

    ROS_INFO_STREAM("Listening for speed on " << speed_topic_name);
    ROS_INFO_STREAM("Listening for steer on " << steering_topic_name);

    // Serial port setup
    std::string serial_port_name;
    nhp.param(std::string("serial_port"), serial_port_name, std::string("/dev/ttyACM0"));
    boost::asio::io_service io_service;
    boost::asio::serial_port serial(io_service, serial_port_name);
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));

    ROS_INFO("IARRC motor relay node ready.");

    float hz = 20;
    ros::Rate rate(hz);
    int count = 0;
    int countLimit = (int) (hz / 10); // Limit motor commands to 10hz regardless of loop rate
    int sequence = 0;
    while (ros::ok() && serial.is_open()) {
        ros::spinOnce();

        if (count == countLimit) {
            if (desiredSteer != prevAngle || desiredSpeed != prevSpeed) {
                ROS_INFO("Sending command: servo=%d, motor=%d", desiredSteer, desiredSpeed);
            }

            prevAngle = desiredSteer;
            prevSpeed = desiredSpeed;

            sendCommand(serial);
            count = 0;
        }
        count++;

        rate.sleep();
    }

    serial.close();

    ROS_INFO("Shutting down IARRC motor relay node.");
    return 0;
}
