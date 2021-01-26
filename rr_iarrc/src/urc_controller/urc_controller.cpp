#include <ros/ros.h>
#include <iostream>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

enum states {LANE_KEEPING, AT_STOP_BAR, TURNING, FINISHED} state;

double laneKeepingSpeed;
double laneKeepingSteering;
double turningSpeed;
double turningSteering;

std::string signDirection;
bool stopBarArrived;

double speed;
double steering;

ros::Publisher steerPub;
ros::Publisher speedPub;

std::string lane_keeper;
std::string turning_action;
std::string turn_detected;
std::string stop_bar_arrived;
std::string urc_turn_action;


void updateState() {
    switch (state) {
        case LANE_KEEPING:
            speed = laneKeepingSpeed;
            steering = laneKeepingSteering;
            if (stopBarArrived){
                state = AT_STOP_BAR;
            }
            break;
        case AT_STOP_BAR:
            speed = 0;
            steering = 0;

            state = TURNING;
            ros::ServiceClient client = nh.serviceClient<rr_iarrc::urc_turn_action>(urc_turn_action);
            rr_iarrc::urc_turn_action srv;
            srv.request.sign = signDirection;

            if (client.call(srv)){
                // Service call finished
                state = LANE_KEEPING;
            } else {
                ROS_ERROR("Failed to call service");
                return 1;
            }

            break;
        case TURNING:
            speed = turningSpeed;    
            steering = turningSteering;
            break;    
        default:
            ROS_WARN("State machine defaulted");
            state = WAITING_FOR_START;    
    }
}

void laneKeeperSpeedCB(const rr_msgs::speed::ConstPtr &speed_msg) {
    laneKeepingSpeed = speed_msg -> speed;
}

void laneKeeperSteeringCB(const rr_msgs::steering::ConstPtr &steer_msg) {
    laneKeepingSteering = steer_msg -> angle;
}

void turningActionSpeedCB(const rr_msgs::speed::ConstPtr &speed_msg) {
    turningSpeed = speed_msg -> speed;
}

void turningActionSteeringCB(const rr_msgs::steering::ConstPtr &steer_msg) {
    turningSteering = steer_msg -> angle;
}

void turnDetectedCB(const std_msgs::StringConstPtr &sign_msg) {
    signDirection = sign_msg -> data;
}

void stopBarArrivedCB(const std_msgs::BoolConstPtr &stop_msg) {
    stopBarArrived = stop_msg -> data;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "urc_controller");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    nhp.getParam("lane_keeper", lane_keeper);
    nhp.getParam("turning_action", turning_action);
    nhp.getParam("turn_detected", turn_detected);
    nhp.getParam("stop_bar_arrived", stop_bar_arrived);
    nhp.getParam("urc_turn_action", urc_turn_action);

    ros::Subscriber laneKeeperSpeedSub = nh.subscribe(lane_keeper + "/speed", 1, &laneKeeperSpeedCB);
    ros::Subscriber laneKeeperSteeringSub = nh.subscribe(lane_keeper + "/steering", 1, &laneKeeperSteeringCB);
    ros::Subscriber turningActionSpeedSub = nh.subscribe(turning_action + "/speed", 1, &turningActionSpeedCB);
    ros::Subscriber turningActionSteeringSub = nh.subscribe(turning_action + "/steering", 1, &turningActionSteeringCB);
    ros::Subscriber turnDetectedSub = nh.subscribe(turn_detected, 1, &turnDetectedCB);
    ros::Subscriber stopBarArrivedSum = nh.subscribe(stop_bar_arrived, 1, &stopBarArrivedCB);

    speedPub = nh.advertise<rr_msgs::speed>("/plan/speed", 1);
    steerPub = nh.advertise<rr_msgs::steering>("/plan/steering", 1);

    ros::Rate rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();
        updateState();

        rr_msgs::speed speedMsg;
        speedMsg.speed = speed;
        speedMsg.header.stamp = ros::Time::now();
        speedPub.publish(speedMsg);

        rr_msgs::steering steerMsg;
        steerMsg.angle = steering;
        steerMsg.header.stamp = ros::Time::now();
        steerPub.publish(steerMsg);

        ROS_INFO_STREAM("Current State: " + std::to_string(state));

        rate.sleep();
    }

    return 0;
}