#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <std_msgs/Bool.h>

constexpr WAITING_FOR_START = 0;
constexpr RUNNING_PLANNER = 1;
constexpr FINISHED = 2;

ros::Publisher steerPub;
ros::Publisher speedPub;

int state;
double planSpeed, speed;
double planSteering, steering;
bool raceStarted;
bool raceEnded;

void updateState() {
    switch(state) {
        case WAITING_FOR_START:
            speed = 0.0;
            steering = 0.0;
            if(raceStarted) {
                state = RUNNING_PLANNER;
            }
            break;
        case RUNNING_PLANNER:
            speed = planSpeed;
            steering = planSteering;
            if(raceEnded) {
                state = FINISHED;
            }
            break;
        case FINISHED:
            speed = 0.0;
            steering = 0.0;
            break;
        default:
            ROS_WARN("State machine defaulted");
            state = WAITING_FOR_START;
    }
}

void planSpeedCB(const rr_platform::speed::ConstPtr &speed_msg) {
    planSpeed = speed_msg->speed;
}

void planSteerCB(const rr_platform::steering::ConstPtr &steer_msg) {
    planSteering = steer_msg->angle;
}

void startLightCB(const std_msgs::Bool::ConstPtr &bool_msg) {
    raceStarted = bool_msg->data;
}

void finishLineCB(const std_msgs::Bool::ConstPtr &bool_msg) {
    raceEnded = bool_msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "iarrc_navigation_controller");

    ros::NodeHandle nh;

    auto planSpeedSub = nh.subscribe("plan/speed", 1, planSpeedCB);
    auto planSteerSub = nh.subscribe("plan/steering", 1, planSteerCB);
    auto startLightSub = nh.subscribe("green_light", 1, startLightCB);

    speedPub = nh.advertise<rr_platform::speed>("speed", 1);
    steerPub = nh.advertise<rr_platform::steering>("steering", 1);

    state = WAITING_FOR_START;
    planSpeed = 0.0;
    planSteering = 0.0;

    ros::Rate rate(30.0);
    while(ros::ok()) {
        updateState();

        rr_platform::speed speedMsg;
        speedMsg.speed = speed;
        speedPub.publish(speedMsg);

        rr_platform::steering steerMsg;
        steerMsg.angle = steering;
        steerMsg.publish(steerMsg);

        rate.sleep();
    }

    return 0;
}
