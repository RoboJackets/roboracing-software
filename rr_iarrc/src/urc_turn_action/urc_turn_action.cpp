#include <math.h>
#include <ros/ros.h>
#include <angles/angles.h>
#include <rr_msgs/axes.h>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>
#include <rr_msgs/turning.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <stdlib.h> /* abs */

using namespace std;

const float PI = 3.14;

ros::Publisher steerPub;
ros::Publisher speedPub;
ros::ServiceServer service;

double speed;
double steering;

int imu_quadrant;
float imu_yaw;
float initial_yaw;
float target_yaw;

double left_offset;
double right_offset;
double turn_speed;
double left_turn_steering;
double right_turn_steering;

double forward_timeout;

std::string turn_direction;
ros::Time finishTime;

void publishSpeedandSteering() {
    rr_msgs::speed speedMsg;
    speedMsg.speed = speed;
    speedMsg.header.stamp = ros::Time::now();
    speedPub.publish(speedMsg);

    rr_msgs::steering steerMsg;
    steerMsg.angle = steering;
    steerMsg.header.stamp = ros::Time::now();
    steerPub.publish(steerMsg);
}

// Used to calculate the distance from the target IMU yaw to the current IMU yaw
// within range of [0 ,6.28]
bool comp(int a, int b) {
    return (a < b);
}

float angleDist(float target, float current) {
    double diff = static_cast<double>(target - current);
    float min = std::min({ std::fabs(diff), std::fabs(diff - 2 * M_PI), std::fabs(diff + 2 * M_PI) }, comp);
    return min;
}

void makeTurn() {
    ros::spinOnce();
    imu_quadrant = ceil((2 * imu_yaw) / PI);
    initial_yaw = imu_yaw;

    if (turn_direction.compare("LEFT") == 0) {
        if (imu_quadrant == 4) {
            target_yaw = initial_yaw - 1.5 * PI - left_offset;
        } else {
            target_yaw = initial_yaw + .5 * PI + left_offset;
        }

        while (ros::ok()) {
            if (angleDist(target_yaw, imu_yaw) <= 0.015) {
                break;
            }

            speed = turn_speed;
            steering = left_turn_steering;
            publishSpeedandSteering();
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }

    } else if (turn_direction.compare("RIGHT") == 0) {
        if (imu_quadrant == 1) {
            target_yaw = initial_yaw + 1.5 * PI + right_offset;
        } else {
            target_yaw = initial_yaw - .5 * PI + right_offset;
        }

        while (ros::ok()) {
            if (angleDist(target_yaw, imu_yaw) <= 0.015) {
                break;
            }

            speed = turn_speed;
            steering = right_turn_steering;
            publishSpeedandSteering();
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }
    } else if (turn_direction.compare("STRAIGHT") == 0) {
        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(forward_timeout);  // Timeout of 2 seconds
        while (ros::Time::now() < start_time + timeout && ros::ok()) {
            speed = turn_speed;
            steering = 0;
            publishSpeedandSteering();
            ros::Duration(0.01).sleep();
        }
    }

    ROS_INFO("Turn made");
    speed = 0.0;
    steering = 0.0;
    publishSpeedandSteering();
}

//Sets the yaw to a positive, normalized angle between [0, 2PI]
void imuCB(const rr_msgs::axesConstPtr& msg) {
    imu_yaw = angles::normalize_angle_positive(msg->yaw);
}

bool turnCallback(rr_msgs::turning::Request &req, rr_msgs::turning::Response &res) {
    ROS_INFO_STREAM(req.direction);
    return true;
}

int main(int argc, char** argv) {
    ROS_INFO("Starting turning test node");
    ros::init(argc, argv, "urc_turn_action");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    nhp.getParam("turn_speed", turn_speed);
    nhp.getParam("left_turn_steering", left_turn_steering);
    nhp.getParam("right_turn_steering", right_turn_steering);
    nhp.getParam("left_offset", left_offset);
    nhp.getParam("right_offset", right_offset);
    nhp.getParam("forward_timeout", forward_timeout);
    nhp.getParam("turn_direction", turn_direction);

    auto imuSub = nh.subscribe("/axes", 1, imuCB);
    speedPub = nh.advertise<rr_msgs::speed>("/speed", 1);
    steerPub = nh.advertise<rr_msgs::steering>("/steering", 1);

    service = nh.advertiseService("turn", turnCallback);

    ros::Rate loopRate(1.0);
    ros::spinOnce();
    loopRate.sleep();

    //makeTurn();

    return 0;
}