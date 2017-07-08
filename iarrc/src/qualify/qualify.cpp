#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>

ros::Publisher speedPub;
ros::Publisher steeringPub;

void publishSpeed(const float &desiredSpeed) {
    rr_platform::speed speedMsg;
    speedMsg.header.stamp = ros::Time::now();
    speedMsg.speed = desiredSpeed;
    speedPub.publish(speedMsg);
    ROS_INFO("published");
    rr_platform::steering steeringMsg;
    steeringMsg.header.stamp = ros::Time::now();
    steeringMsg.angle = 0.0;
    steeringPub.publish(steeringMsg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "qualify");
    ros::NodeHandle nh;
    speedPub = nh.advertise<rr_platform::speed>("/speed", 1);
    steeringPub = nh.advertise<rr_platform::steering>("/steering", 1);
    ros::Duration(1).sleep();
    publishSpeed(2.0);
    ros::Duration(5).sleep();
    publishSpeed(0.0);
    return 0;
}