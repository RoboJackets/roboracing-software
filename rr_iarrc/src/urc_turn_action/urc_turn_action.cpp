#include <math.h>
#include <ros/ros.h>
#include <angles/angles.h>
#include <rr_msgs/axes.h>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>
#include <rr_msgs/turning.h>
#include <stdlib.h>

using namespace std;

ros::Publisher steerPub,speedPub;
ros::ServiceServer service;

double imu_yaw;

double left_offset, right_offset;
double turn_speed;
double left_turn_steering, right_turn_steering;

double forward_timeout;
double angle_threshold;


void publishSpeedandSteering(double speed, double steering) {
    rr_msgs::speed speedMsg;
    speedMsg.speed = speed;
    speedMsg.header.stamp = ros::Time::now();
    speedPub.publish(speedMsg);

    rr_msgs::steering steerMsg;
    steerMsg.angle = steering;
    steerMsg.header.stamp = ros::Time::now();
    steerPub.publish(steerMsg);
}

void makeTurn(int turn_direction, double approach_angle) {
    ros::spinOnce();
    double initial_yaw = imu_yaw;

    if (turn_direction == rr_msgs::turning::Request::LEFT) {
        double target_yaw = initial_yaw + approach_angle + .5 * M_PI + left_offset;
        while (abs(angles::shortest_angular_distance(target_yaw, imu_yaw)) > angle_threshold) {
            publishSpeedandSteering(turn_speed, left_turn_steering);
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
    } else if (turn_direction == rr_msgs::turning::Request::RIGHT) {
        double target_yaw = initial_yaw + approach_angle - .5 * M_PI + right_offset;

        while (abs(angles::shortest_angular_distance(target_yaw, imu_yaw)) > angle_threshold) {
            publishSpeedandSteering(turn_speed, right_turn_steering);
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
    } else if (turn_direction == rr_msgs::turning::Request::STRAIGHT) {
        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(forward_timeout);  // Timeout of 2 seconds

        while (ros::Time::now() < start_time + timeout) {
            publishSpeedandSteering(turn_speed, 0);
            ros::Duration(0.01).sleep();
        }
    }

    ROS_INFO("Turn made");
    publishSpeedandSteering(0, 0);
}

// Sets the yaw to a positive, normalized angle between [0, 2PI]
void imuCB(const rr_msgs::axesConstPtr& msg) {
    imu_yaw = angles::normalize_angle_positive(msg->yaw);
}

bool turnCallback(rr_msgs::turning::Request &req, rr_msgs::turning::Response &res) {
    ROS_INFO_STREAM(req.direction);
    makeTurn(req.direction, req.approach_angle);
    res.success = true;
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
    nhp.getParam("angle_threshold", angle_threshold);

    auto imuSub = nh.subscribe("/axes", 1, imuCB);
    speedPub = nh.advertise<rr_msgs::speed>("/speed", 1);
    steerPub = nh.advertise<rr_msgs::steering>("/steering", 1);

    service = nh.advertiseService("/turn_action", turnCallback);

    ros::spin();

    return 0;
}