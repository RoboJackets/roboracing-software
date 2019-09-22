#include <ros/ros.h>
#include <rr_platform/SerialPort.h>
#include <rr_platform/axes.h>
#include <rr_platform/chassis_state.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <boost/asio.hpp>
#include <boost/regex.hpp>

ros::Publisher imu_pub;
ros::Publisher mag_pub;
ros::Publisher axes_pub;

/**
 * @note http://stackoverflow.com/a/27511119
 */

std::vector<std::string> split(const std::string &s, char delim) {
    std::stringstream ss(s);
    std::string item;
    std::vector<std::string> elems;
    while (std::getline(ss, item, delim)) {
        elems.push_back(std::move(item));
    }
    return elems;
}

bool set_imu(const std::string &line_in, sensor_msgs::Imu &imu_msg) {
    std::vector<std::string> data = split(line_in, ',');
    if (data[0] == "ax") {
        imu_msg.linear_acceleration.x = std::atof(data[1].c_str());
        imu_msg.linear_acceleration.y = std::atof(data[2].c_str());
        imu_msg.linear_acceleration.z = std::atof(data[3].c_str());
    } else if (data[0] == "gx") {
        imu_msg.angular_velocity.x = std::atof(data[1].c_str());
        imu_msg.angular_velocity.y = std::atof(data[2].c_str());
        imu_msg.angular_velocity.z = std::atof(data[3].c_str());
    } else if (data[0] == "q0") {
        imu_msg.orientation.x = std::atof(data[1].c_str());
        imu_msg.orientation.y = std::atof(data[2].c_str());
        imu_msg.orientation.z = std::atof(data[3].c_str());
        imu_msg.orientation.w = std::atof(data[4].c_str());
        return true;
    } else {
        return false;
    }
}

bool set_mag(const std::string &line_in, sensor_msgs::MagneticField &mag_msg) {
    std::vector<std::string> data = split(line_in, ',');
    if (data[0] == "mx") {
        mag_msg.magnetic_field.x = std::atof(data[1].c_str());
        mag_msg.magnetic_field.y = std::atof(data[2].c_str());
        mag_msg.magnetic_field.z = std::atof(data[3].c_str());
        return true;
    } else {
        return false;
    }
}

bool set_axes(const std::string &line_in, rr_platform::axes &axes_msg) {
    std::vector<std::string> data = split(line_in, ',');
    if (data[0] == "axes") {
        axes_msg.roll = std::atof(data[1].c_str());
        axes_msg.pitch = std::atof(data[2].c_str());
        axes_msg.yaw = std::atof(data[3].c_str());
        return true;
    } else {
        return false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu");

    ros::NodeHandle handle;
    ros::NodeHandle private_handle("~");

    imu_pub = handle.advertise<sensor_msgs::Imu>("/imu/data_raw", 1);
    mag_pub = handle.advertise<sensor_msgs::MagneticField>("/imu/mag", 1);
    axes_pub = handle.advertise<rr_platform::axes>("/axes", 1);

    // Serial port setup
    std::string serial_port_name;
    private_handle.param(std::string("serial_port"), serial_port_name, std::string("/dev/razor_imu"));
    SerialPort serial_port;

    if (!serial_port.Open(serial_port_name, 115200)) {
        ROS_FATAL_STREAM("Unable to open serial port: " << serial_port_name);
        return 1;
    }

    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;
    rr_platform::axes axes_msg;

    imu_msg.header.frame_id = "imu";
    mag_msg.header.frame_id = "mag";

    ROS_INFO("IMU Ready.");

    ros::Duration(2.0).sleep();

    while (ros::ok()) {
        ros::spinOnce();

        auto line_in = serial_port.ReadLine();
        if (!line_in.empty()) {
            if (set_imu(line_in, imu_msg)) {
                sensor_msgs::Imu publishable_copy = imu_msg;
                publishable_copy.header.stamp = ros::Time::now();
                imu_pub.publish(publishable_copy);
            }

            if (set_mag(line_in, mag_msg)) {
                sensor_msgs::MagneticField publishable_copy = mag_msg;
                publishable_copy.header.stamp = ros::Time::now();
                mag_pub.publish(publishable_copy);
            }

            if (set_axes(line_in, axes_msg)) {
                rr_platform::axes publishable_copy = axes_msg;
                publishable_copy.header.stamp = ros::Time::now();
                axes_pub.publish(publishable_copy);
            }
        }
    }

    return 0;
}
