#include <ros/ros.h>
#include <rr_msgs/axes.h>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#include <stdlib.h> /* abs */

using namespace std;

const float PI = 3.14;

ros::Publisher steerPub;
ros::Publisher speedPub;

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

/*
// Checks to see in which quadrant the current yaw reading is in. Turning right
while in Q1, and turning left in Q4 are special cases.
*/
void checkQuadrant() {
    if (imu_yaw <= 2 * PI && imu_yaw >= 1.5 * PI) {
        imu_quadrant = 4;
    } else if (imu_yaw >= 0 && imu_yaw <= .5 * PI) {
        imu_quadrant = 1;
    }
}

/*
// As you turn counterclockwise, the yaw value received from the IMU ranges from
[0, 3.14] U [-3.14, 0]
// This function returns a new yaw value in the range of [0 ,6.28]
*/
double setInRange(double yaw_read) {
    double new_yaw;
    if (yaw_read < 0) {
        new_yaw = yaw_read + 2 * PI;
        return new_yaw;
    } else {
        return yaw_read;
    }
}

// Used to calculate the distance from the target IMU yaw to the current IMU yaw
// within range of [0 ,6.28]
bool comp(int a, int b) {
    return (a < b);
}

float angleDist(float target, float current) {
    float diff = target - current;
    float min = std::min({ std::abs(diff), std::abs(diff - 2 * PI), std::abs(diff + 2 * PI) }, comp);
    return min;
}

void makeTurn() {
    ros::spinOnce();
    checkQuadrant();
    initial_yaw = imu_yaw;

    if (turn_direction.compare("left") == 0) {
        if (imu_quadrant == 4) {
            target_yaw = initial_yaw - 1.5 * PI - left_offset;
        } else {
            target_yaw = initial_yaw + .5 * PI + left_offset;
        }

        while (1) {
            if (angleDist(target_yaw, imu_yaw) <= 0.015) {
                break;
            }

            speed = turn_speed;
            steering = left_turn_steering;
            publishSpeedandSteering();
            ros::spinOnce();
        }

    } else if (turn_direction.compare("right") == 0) {
        if (imu_quadrant == 1) {
            target_yaw = initial_yaw + 1.5 * PI + right_offset;
        } else {
            target_yaw = initial_yaw - .5 * PI + right_offset;
        }

        while (1) {
            if (angleDist(target_yaw, imu_yaw) <= 0.015) {
                break;
            }

            speed = turn_speed;
            steering = right_turn_steering;
            publishSpeedandSteering();
            ros::spinOnce();
        }
    } else if (turn_direction.compare("forward") == 0) {
        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(forward_timeout);  // Timeout of 2 seconds
        while (ros::Time::now() < start_time + timeout) {
            speed = turn_speed;
            steering = 0;
            publishSpeedandSteering();
        }
    }

    ROS_INFO("Turn made");
    speed = 0.0;
    steering = 0.0;
    publishSpeedandSteering();
}

void imuCB(const rr_msgs::axesConstPtr& msg) {
    imu_yaw = setInRange(msg->yaw);
}

int main(int argc, char** argv) {
    ROS_INFO("Starting turning test node");
    ros::init(argc, argv, "turning_test");

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

    ros::Rate loopRate(1.0);
    ros::spinOnce();
    loopRate.sleep();

    makeTurn();

    return 0;
}