#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

const int WAITING_FOR_START = 0;
const int RUNNING_PLANNER = 1;
const int FINISHED = 2;
int REQ_FINISH_LINE_CROSSES;
std::string startSignal;

ros::Publisher steerPub;
ros::Publisher speedPub;

int state;
double planSpeed, speed;
double planSteering, steering;
bool raceStarted;
int finishLineCrosses;

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
            if(finishLineCrosses >= REQ_FINISH_LINE_CROSSES) {
                state = FINISHED;
            }
            break;
        case FINISHED:
            //ROS_INFO("finished");
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

void finishLineCB(const std_msgs::Int8::ConstPtr &int_msg) {
    finishLineCrosses = int_msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_controller");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    state = WAITING_FOR_START;
    planSpeed = 0.0;
    planSteering = 0.0;
    finishLineCrosses = 0;

    nhp.getParam("req_finish_line_crosses", REQ_FINISH_LINE_CROSSES);
    nhp.getParam("startSignal", startSignal);
    ROS_INFO("req finish line crosses = %d", REQ_FINISH_LINE_CROSSES);

    auto planSpeedSub = nh.subscribe("plan/speed", 1, planSpeedCB);
    auto planSteerSub = nh.subscribe("plan/steering", 1, planSteerCB);
    auto startLightSub = nh.subscribe(startSignal, 1, startLightCB);
    auto finishLineSub = nh.subscribe("/finish_line_crosses", 1, finishLineCB);

    speedPub = nh.advertise<rr_platform::speed>("/speed", 1);
    steerPub = nh.advertise<rr_platform::steering>("/steering", 1);

    ros::Rate rate(30.0);
    while(ros::ok()) {
        ros::spinOnce();
        updateState();
        //ROS_INFO("Nav Mux = %d, crosses = %d", state, finishLineCrosses);

        rr_platform::speed speedMsg;
        speedMsg.speed = speed;
        speedMsg.header.stamp = ros::Time::now();
        speedPub.publish(speedMsg);

        rr_platform::steering steerMsg;
        steerMsg.angle = steering;
        steerMsg.header.stamp = ros::Time::now();
        steerPub.publish(steerMsg);

        rate.sleep();
    }

    return 0;
}
