#include <ros/ros.h>
#include <ros/publisher.h>
#include <rr_platform_msgs/speed.h>
#include <rr_platform_msgs/steering.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <sstream>

rr_platform_msgs::speed speed_cmd;
rr_platform_msgs::steering steering_cmd;
int prevAngle = 0;
int prevSpeed = 0;

const boost::array<double, 9ul> unknown_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

/**
 * @note http://stackoverflow.com/a/27511119
 */
std::vector<std::string> split(const std::string &s, char delim)
{
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> elems;
  while (std::getline(ss, item, delim)) {
    elems.push_back(std::move(item));
  }
  return elems;
}

void SpeedCallback(const rr_platform_msgs::speed::ConstPtr &msg)
{
  speed_cmd = *msg;
}

void SteeringCallback(const rr_platform_msgs::steering::ConstPtr &msg)
{
  steering_cmd = *msg;
}

void sendCommand(boost::asio::serial_port &port)
{
  std::string message = "$" + std::to_string(static_cast<char>(speed_cmd.speed+90)) + std::to_string(static_cast<char>(steering_cmd.angle + 90)) + "\n";

  try {
    boost::asio::write(port, boost::asio::buffer(message.c_str(), message.size()));
  } catch (boost::system::system_error &err) {
    ROS_ERROR("%s", err.what());
  }
}

std::string readLine(boost::asio::serial_port &port)
{
  std::string line;
  while (true) {
    char in;
    try {
      boost::asio::read(port, boost::asio::buffer(&in, 1));
    } catch (
        boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> > &err) {
      ROS_ERROR("Error reading serial port.");
      ROS_ERROR_STREAM(err.what());
      return line;
    }
    if (in == '\n')
      return line;
    if (in == '\r')
      return line;
    line += in;
  }
}

int main(int argc, char **argv)
{
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

  // Publishers
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 1);

  ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("/imu/mag", 1);

  ros::Publisher temp_pub = nh.advertise<sensor_msgs::Temperature>("/imu/temperature", 1);

  ROS_INFO_STREAM("Listening for speed on " << speed_topic_name);
  ROS_INFO_STREAM("Listening for steer on " << steering_topic_name);

  // Serial port setup
  std::string serial_port_name;
  nhp.param(std::string("serial_port"), serial_port_name, std::string("/dev/ttyACM0"));
  boost::asio::io_service io_service;
  boost::asio::serial_port serial(io_service, serial_port_name);

  ROS_INFO("IARRC motor relay node ready.");

  float hz = 20;
  ros::Rate rate(hz);
  int count = 0;
  int countLimit = (int) (hz / 10); // Limit motor commands to 10hz regardless of loop rate
  int sequence = 0;
  while (ros::ok() && serial.is_open()) {
    ros::spinOnce();

    if (count == countLimit) {
      if (steering_cmd.angle != prevAngle || speed_cmd.speed != prevSpeed)
        ROS_INFO("Sending command: servo=%d, motor=%d", steering_cmd.angle, speed_cmd.speed);

      prevAngle = steering_cmd.angle;
      prevSpeed = speed_cmd.speed;

      sendCommand(serial);
      count = 0;
    }
    count++;

    auto lineIn = readLine(serial);
    if (!lineIn.empty()) {

      auto tokens = split(lineIn.substr(1), ',');
      if (tokens.size() < 14) {
        ROS_WARN("Received invalid response.");
        continue;
      }

      sensor_msgs::Imu imuMsg;
      imuMsg.header.seq = sequence++;
      imuMsg.header.stamp = ros::Time::now();
      imuMsg.header.frame_id = "imu";
      imuMsg.linear_acceleration.x = std::stod(tokens[0]);
      imuMsg.linear_acceleration.y = std::stod(tokens[1]);
      imuMsg.linear_acceleration.z = std::stod(tokens[2]);
      imuMsg.angular_velocity.x = std::stod(tokens[3]);
      imuMsg.angular_velocity.y = std::stod(tokens[4]);
      imuMsg.angular_velocity.z = std::stod(tokens[5]);
      imuMsg.orientation.x = std::stod(tokens[6]);
      imuMsg.orientation.y = std::stod(tokens[7]);
      imuMsg.orientation.z = std::stod(tokens[8]);
      imuMsg.orientation.w = std::stod(tokens[9]);
      imuMsg.linear_acceleration_covariance = unknown_covariance;
      imuMsg.angular_velocity_covariance = unknown_covariance;
      imuMsg.orientation_covariance = unknown_covariance;
      imu_pub.publish(imuMsg);

      sensor_msgs::MagneticField magMsg;
      magMsg.header = imuMsg.header;
      magMsg.magnetic_field.x = std::stod(tokens[10]);
      magMsg.magnetic_field.y = std::stod(tokens[11]);
      magMsg.magnetic_field.z = std::stod(tokens[12]);
      magMsg.magnetic_field_covariance = unknown_covariance;
      mag_pub.publish(magMsg);

      sensor_msgs::Temperature tempMsg;
      tempMsg.header = imuMsg.header;
      tempMsg.temperature = std::stod(tokens[13]);
      tempMsg.variance = 0;
      temp_pub.publish(tempMsg);
    }

    rate.sleep();
  }

  serial.close();

  ROS_INFO("Shutting down IARRC motor relay node.");
  return 0;
}
