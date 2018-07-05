#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/chassis_state.h>
#include <SerialPort.h>

double desiredSpeed = 0;
double desiredSteer = 0;
double prevAngle = 0;
double prevSpeed = 0;

double kP = 0;
double kI = 0;
double kD = 0;
int trim = 0;

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
}

void sendCommand(SerialPort &port) {
    std::string message = "$" + std::to_string(desiredSpeed) + ", " +
                          std::to_string(desiredSteer) + "," +
                          std::to_string(kP) + "," +
                          std::to_string(kI) + "," +
                          std::to_string(kD) + "," +
                          std::to_string(trim) + "\n";

    port.Write(message);
}

void publishData(const std::string &line) {
    if (line.empty()) {
        return;
    }
    std::vector <std::string> data = split(line.substr(1), ',');
    rr_platform::chassis_state msg;
    msg.header.stamp = ros::Time::now();
    msg.speed_mps = std::atof(data[0].c_str()) / (s_per_50ms * ticks_per_meter);
    msg.mux_automatic = (data[1] == "1");
    msg.estop_on = (data[2] == "1");
    state_pub.publish(msg);
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
    nhp.param(std::string("trim"), trim, 0);

    state_pub = nh.advertise<rr_platform::chassis_state>("/chassis_state", 1);

    ROS_INFO_STREAM("Listening for speed on " << speed_topic_name);
    ROS_INFO_STREAM("Listening for steer on " << steering_topic_name);

    // Serial port setup
    std::string serial_port_name;
    nhp.param(std::string("serial_port"), serial_port_name, std::string("/dev/ttyACM0"));

    SerialPort serial_port;
    if(!serial_port.Open(serial_port_name, 115200)) {
        ROS_FATAL_STREAM("Unable to open serial port: " << serial_port_name);
        return 1;
    }

    ROS_INFO("IARRC motor relay node is ready.");

    float hz = 20;
    ros::Rate rate(hz);
    int count = 0;
    int countLimit = (int) (hz / 10); // Limit motor commands to 10hz regardless of loop rate
    int sequence = 0;

    ros::Duration(2.0).sleep();

    while (ros::ok()) {
        ros::spinOnce();
        if (count == countLimit) {
            if (desiredSteer != prevAngle || desiredSpeed != prevSpeed) {
                ROS_INFO("Sending command: servo=%f, motor=%f", desiredSteer, desiredSpeed);
            }

            prevAngle = desiredSteer;
            prevSpeed = desiredSpeed;
            sendCommand(serial_port);
            publishData(serial_port.ReadLine());

            count = 0;
        }
        count++;

        rate.sleep();
    }

    ROS_INFO("Shutting down IARRC motor relay node.");
    return 0;
}
