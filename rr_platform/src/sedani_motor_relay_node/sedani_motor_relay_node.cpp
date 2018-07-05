#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/chassis_state.h>
#include <boost/asio.hpp>

double desiredSpeed = 0;
double desiredSteer = 0;
double prevAngle = 0;
double prevSpeed = 0;

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
    desiredSpeed = msg->speed;
}

void SteeringCallback(const rr_platform::steering::ConstPtr &msg) {
    desiredSteer = msg->angle;
}

void sendCommand(boost::asio::serial_port &port) {
    std::string message = "$" + std::to_string(desiredSpeed) + ", " + std::to_string(desiredSteer) + "\n";
    try {
        boost::asio::write(port, boost::asio::buffer(message.c_str(), message.size()));
    } catch (boost::system::system_error &err) {
        ROS_ERROR("%s", err.what());
    }
}

void publishData(const std::string &line) {
    if (line.empty()) {
        return;
    }
    std::vector <std::string> data = split(line.substr(1), ',');
    rr_platform::chassis_state msg;
    msg.header.stamp = ros::Time::now();
    msg.speed_mps = std::stod(data[1]);
    msg.steer_rad = std::stod(data[2]);
    msg.state = data[0];
    msg.mux_automatic = (data[0] == "1.00" || data[0] == "3.00");
    msg.estop_on = (data[0] == "2.00");
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
                return line;
	    }
            if (in == '\r') {
                return line;
	    }
            line += in;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sedani_motor_relay_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Subscribers
    std::string speed_topic_name;
    nhp.param(std::string("speed_topic"), speed_topic_name, std::string("/speed"));
    ros::Subscriber speed_sub = nh.subscribe(speed_topic_name, 1, SpeedCallback);

    std::string steering_topic_name;
    nhp.param(std::string("steering_topic"), steering_topic_name, std::string("/steering"));
    ros::Subscriber steering_sub = nh.subscribe(steering_topic_name, 1, SteeringCallback);

    state_pub = nh.advertise<rr_platform::chassis_state>("/chassis_state", 1);

    ROS_INFO_STREAM("Listening for speed on " << speed_topic_name);
    ROS_INFO_STREAM("Listening for steer on " << steering_topic_name);

    // Serial port setup
    std::string serial_port_name;
    nhp.param(std::string("serial_port"), serial_port_name, std::string("/dev/ttyACM0"));
    boost::asio::io_service io_service;
    boost::asio::serial_port serial(io_service, serial_port_name);
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));

    ROS_INFO("Sedani motor relay node is ready.");

    float hz = 20;
    ros::Rate rate(hz);

    ros::Duration(2.0).sleep();

    while (ros::ok() && serial.is_open()) {
        ros::spinOnce();
        if (desiredSteer != prevAngle || desiredSpeed != prevSpeed) {
            ROS_INFO("Sending command: servo=%f, motor=%f", desiredSteer, desiredSpeed);
        }

        prevAngle = desiredSteer;
        prevSpeed = desiredSpeed;
        sendCommand(serial);
        publishData(readLine(serial));
        rate.sleep();
    }

    serial.close();

    ROS_INFO("Shutting down Sedani motor relay node.");
    return 0;
}
