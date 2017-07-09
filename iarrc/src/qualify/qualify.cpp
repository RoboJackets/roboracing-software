#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/chassis_state.h>

ros::Publisher speedPub;
ros::Publisher steeringPub;

bool go = false;

void chassisCB(const rr_platform::chassis_stateConstPtr& msg) {
    go = msg->mux_automatic;
}

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
    ros::Subscriber  chassis_sub = nh.subscribe("/chassis_state", 1, chassisCB);
    while (!go && ros::ok()) {
        ros::spinOnce();
    }
    publishSpeed(2.0);
    ros::Duration(5).sleep();
    publishSpeed(0.0);
    return 0;
}
