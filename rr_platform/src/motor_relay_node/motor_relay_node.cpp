#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/chassis_state.h>
#include <boost/asio.hpp>

double desiredSpeed = 0;
double desiredSteer = 0;
double prevAngle = 0;
double prevSpeed = 0;

double kP = 0;
double kI = 0;
double kD = 0;

const boost::array<double, 9ul> unknown_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

constexpr float ticks_per_meter = 1725.0f;
constexpr float s_per_50ms = 0.05f;

ros::Publisher state_pub;

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

void SpeedCallback(const rr_platform::speed::ConstPtr &msg) {
    desiredSpeed = msg->speed * ticks_per_meter * s_per_50ms;
}

void SteeringCallback(const rr_platform::steering::ConstPtr &msg) {
    desiredSteer = msg->angle;
    ROS_INFO_STREAM("Desired steer: " << desiredSteer);
}

void sendCommand(boost::asio::serial_port &port) {
    std::string message = "$" + std::to_string(desiredSpeed) + ", " +
                          std::to_string(desiredSteer) + "," +
                          std::to_string(kP) + "," +
                          std::to_string(kI) + "," +
                          std::to_string(kD) + "\n";
    
    try {
        boost::asio::write(port, boost::asio::buffer(message.c_str(), message.size()));
    } catch (boost::system::system_error &err) {
        ROS_ERROR("%s", err.what());
    }
    //ROS_INFO_STREAM("sent: " + message);
}

void publishData(const std::string &line) {
    if (line.empty()) {
        return;
    }
    //ROS_INFO_STREAM(line);
    std::vector <std::string> data = split(line.substr(1), ',');
    rr_platform::chassis_state msg;
    msg.header.stamp = ros::Time::now();
    msg.speed_mps = std::atof(data[0].c_str()) / (s_per_50ms * ticks_per_meter);
    msg.mux_automatic = (data[1] == "1");
    msg.estop_on = (data[2] == "1");
    state_pub.publish(msg);
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
            if (in == '\n') {
	        ROS_INFO_STREAM(line);
                return line;
	    }
            if (in == '\r') {
	        ROS_INFO_STREAM(line);
                return line;
	    }
	    ROS_INFO_STREAM("adding char");
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

    nhp.param(std::string("kP"), kP, 1.0);
    nhp.param(std::string("kI"), kI, 0.0);
    nhp.param(std::string("kD"), kD, 0.1);

    state_pub = nh.advertise<rr_platform::chassis_state>("/chassis_state", 1);

    ROS_INFO_STREAM("Listening for speed on " << speed_topic_name);
    ROS_INFO_STREAM("Listening for steer on " << steering_topic_name);

    // Serial port setup
    std::string serial_port_name;
    nhp.param(std::string("serial_port"), serial_port_name, std::string("/dev/ttyACM0"));
    boost::asio::io_service io_service;
    boost::asio::serial_port serial(io_service, serial_port_name);
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));

    ROS_INFO("IARRC motor relay node is ready.");

    float hz = 20;
    ros::Rate rate(hz);
    int count = 0;
    int countLimit = (int) (hz / 10); // Limit motor commands to 10hz regardless of loop rate
    int sequence = 0;
    ros::Duration(1).sleep();
    while (ros::ok() && serial.is_open()) {
        ros::spinOnce();
        if (count == countLimit) {
            if (desiredSteer != prevAngle || desiredSpeed != prevSpeed) {
                ROS_INFO("Sending command: servo=%f, motor=%f", desiredSteer, desiredSpeed);
            }

            prevAngle = desiredSteer;
            prevSpeed = desiredSpeed;
            sendCommand(serial);
            publishData(readLine(serial));

            count = 0;
        }
        count++;

        rate.sleep();
    }

    serial.close();

    ROS_INFO("Shutting down IARRC motor relay node.");
    return 0;
}
