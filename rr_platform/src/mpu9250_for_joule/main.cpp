#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include "mpu9250.h"
#include "quaternionFilters.h"

using namespace std;

// Degrees to Radians
constexpr double DEG_TO_RAD = M_PI/180.0;
// g's to m/s^2
constexpr double G_TO_MSS = 9.80665;
// milliGauss to Tesla
constexpr double mG_TO_T = 1e-7;

int main(int argc, char **argv) {

    ros::init(argc, argv,  "mpu9250_for_joule");

    ros::NodeHandle handle;

    MPU9250 imu;
    
    if(!imu.whoAmIPassed()) {
        ROS_ERROR("Failed to read Who-Am-I value from IMU.");
        return 1;
    }
    
    array<float, 6> selfTestResults;
    imu.runSelfTest(selfTestResults);
    
    if(any_of(selfTestResults.begin(), selfTestResults.end(),
           [](const auto &e){ return e > 14.0; })) {
        ROS_WARN("SelfTest results outside of acceptable range.");
        ROS_WARN_STREAM("SelfTest results: "
                        << selfTestResults[0] << "  "
                        << selfTestResults[1] << "  "
                        << selfTestResults[2] << "  "
                        << selfTestResults[3] << "  "
                        << selfTestResults[4] << "  "
                        << selfTestResults[5] << "  "
        );
    }

    imu.initialize();

    auto imu_publisher = handle.advertise<sensor_msgs::Imu>("/imu/data_raw", 1);
    auto mag_publisher = handle.advertise<sensor_msgs::MagneticField>("/imu/mag",1);
    auto temp_publisher = handle.advertise<sensor_msgs::Temperature>("/imu/temperature",1);

    ros::Rate rate(10/*Hz*/);

    string frame_id = "imu";

    auto lastStamp = ros::Time::now();

    while(ros::ok()) {

        auto stamp = ros::Time::now();
        auto deltaT = (stamp - lastStamp).toSec();
        lastStamp = stamp;

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = frame_id;
        imu.getGyro(imu_msg.angular_velocity.x,imu_msg.angular_velocity.y,imu_msg.angular_velocity.z);
        imu.getAccel(imu_msg.linear_acceleration.x,imu_msg.linear_acceleration.y,imu_msg.linear_acceleration.z);

        sensor_msgs::MagneticField mag_msg;
        mag_msg.header.stamp = stamp;
        mag_msg.header.frame_id = frame_id;
        imu.getMag(mag_msg.magnetic_field.x,mag_msg.magnetic_field.y,mag_msg.magnetic_field.z);

        sensor_msgs::Temperature temp_msg;
        temp_msg.header.stamp = stamp;
        temp_msg.header.frame_id = frame_id;
        imu.getTemp(temp_msg.temperature);

        MahonyQuaternionUpdate(static_cast<float>(imu_msg.linear_acceleration.x),
                               static_cast<float>(imu_msg.linear_acceleration.y),
                               static_cast<float>(imu_msg.linear_acceleration.z),
                               static_cast<float>(imu_msg.angular_velocity.x),
                               static_cast<float>(imu_msg.angular_velocity.y),
                               static_cast<float>(imu_msg.linear_acceleration.z),
                               static_cast<float>(mag_msg.magnetic_field.x),
                               static_cast<float>(mag_msg.magnetic_field.y),
                               static_cast<float>(mag_msg.magnetic_field.z),
                               deltaT);
        auto quat = getQ();

        imu_msg.orientation.x = quat[0];
        imu_msg.orientation.y = quat[1];
        imu_msg.orientation.z = quat[2];
        imu_msg.orientation.w = quat[3];

        imu_msg.angular_velocity.x *= DEG_TO_RAD;
        imu_msg.angular_velocity.y *= DEG_TO_RAD;
        imu_msg.angular_velocity.z *= DEG_TO_RAD;
        imu_msg.linear_acceleration.x *= G_TO_MSS;
        imu_msg.linear_acceleration.y *= G_TO_MSS;
        imu_msg.linear_acceleration.z *= G_TO_MSS;
        mag_msg.magnetic_field.x *= mG_TO_T;
        mag_msg.magnetic_field.y *= mG_TO_T;
        mag_msg.magnetic_field.z *= mG_TO_T;

        imu_publisher.publish(imu_msg);
        mag_publisher.publish(mag_msg);
        temp_publisher.publish(temp_msg);

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}

