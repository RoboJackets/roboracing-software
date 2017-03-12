#include <ros/ros.h>
#include <mraa.hpp>

using namespace std;

constexpr uint8_t MPU9250_ADDRESS = 0b1101000;
constexpr uint8_t MPU9250_WHO_AM_I_REGISTER = 117;
constexpr uint8_t MPU9250_WHO_AM_I_VALUE = 0x71;

int main(int argc, char **argv) {

    ros::init(argc, argv,  "mpu9250_for_joule");

    ros::NodeHandle handle;

//    mraa::I2c imu{0};
//
//    imu.address(MPU9250_ADDRESS);
//
//    auto whoAmIVal = imu.readReg(MPU9250_WHO_AM_I_REGISTER);
//
//    if(whoAmIVal != MPU9250_WHO_AM_I_VALUE) {
//        ROS_ERROR_STREAM("Recieved unexpected value (" << whoAmIVal << ") from IMU's \"Who Am I\" register.");
//        ros::shutdown();
//        return 1;
//    }

    mraa::Gpio led(100);

    auto response = led.dir(mraa::DIR_OUT);
    if(response != mraa::SUCCESS) {
        ROS_ERROR_STREAM(response);
        return 1;
    }

    ros::Rate rate{10}; // TODO set this higher if possible

    while(ros::ok()) {

        led.write(1);
        sleep(1);
        led.write(0);
        sleep(1);

        rate.sleep();
    }

    return 0;
}