#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/race_reset.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

const int WAITING_FOR_START = 0;
const int RUNNING_PLANNER = 1;
const int FINISHED = 2;
int REQ_FINISH_LINE_CROSSES;
std::string startSignal;
std::string resetSignal;
std::string finishLineCrossesSignal;


ros::Publisher steerPub;
ros::Publisher speedPub;

int state;
double planSpeed, speed;
double planSteering, steering;
bool raceStarted;
int finishLineCrosses;

ros::Time finishTime;
double steeringAfterFinishTime;

ros::Time startTime;
ros::Duration slowSpeedStartTime;
double slowSpeedFactor;

void updateState() {
    switch(state) {
        case WAITING_FOR_START:
            speed = 0.0;
            steering = 0.0;
            if(raceStarted) {
                state = RUNNING_PLANNER;
                startTime = ros::Time::now();
            }
            break;
        case RUNNING_PLANNER:
            if(finishLineCrosses >= REQ_FINISH_LINE_CROSSES) {
                state = FINISHED;
                finishTime = ros::Time::now();
            } else {
                if(startTime + slowSpeedStartTime < ros::Time::now()) {
                    speed = planSpeed * slowSpeedFactor;
                } else {
                    speed = planSpeed;
                }
                steering = planSteering;
            }
            break;
        case FINISHED:
            speed = 0.0; //@note: -1 brakes, 0 coasts
            //Continue steering until we have stopped moving.
            if (ros::Time::now() - finishTime > ros::Duration(steeringAfterFinishTime)) {
                steering = 0.0;
            } else {
                steering = planSteering;
            }
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

void resetCB(const rr_platform::race_reset &reset_msg) {
    state = WAITING_FOR_START;
    raceStarted = false;
    updateState();
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
    nhp.getParam("resetSignal", resetSignal);
    nhp.getParam("finishLineCrossesSignal", finishLineCrossesSignal);
    nhp.getParam("steeringAfterFinishTime", steeringAfterFinishTime);
    nhp.param("slowSpeedFactor", slowSpeedFactor, 1.0);
    double slowSpeedDuration;
    nhp.param("slowSpeedStartTime", slowSpeedDuration, 10000.0);
    slowSpeedStartTime = ros::Duration(slowSpeedDuration);
    ROS_INFO("required finish line crosses = %d", REQ_FINISH_LINE_CROSSES);

    auto planSpeedSub = nh.subscribe("plan/speed", 1, planSpeedCB);
    auto planSteerSub = nh.subscribe("plan/steering", 1, planSteerCB);
    auto startLightSub = nh.subscribe(startSignal, 1, startLightCB);
    auto finishLineSub = nh.subscribe(finishLineCrossesSignal, 1, finishLineCB);
    auto resetSub = nh.subscribe(resetSignal, 1, resetCB);

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
        ROS_INFO("Current state: %d", state);

        rate.sleep();
    }

    return 0;
}
