#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/axes.h>
#include <rr_platform/race_reset.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

const int WAITING_FOR_START = 0;
const int RUNNING_PLANNER = 1;
const int FINISHED = 2;
const int TURNING = 3; 

std::string startSignal;
std::string resetSignal;


ros::Publisher steerPub;
ros::Publisher speedPub;

int state;
double planSpeed, speed;
double planSteering, steering;

bool raceStarted;
bool turnDetected = false;
bool completed; 

double imu_yaw;
double initial_yaw; 
double target_yaw;

double left_offset; 
double right_offset; 
double turn_speed; 
double turn_steering; 

double forward_timeout;

std::string turn_direction = "right";
ros::Time finishTime;

void publishSpeedandSteering(){
        rr_platform::speed speedMsg;
        speedMsg.speed = speed;
        speedMsg.header.stamp = ros::Time::now();
        speedPub.publish(speedMsg);

        rr_platform::steering steerMsg;
        steerMsg.angle = steering;
        steerMsg.header.stamp = ros::Time::now();
        steerPub.publish(steerMsg);
}

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

            if(turnDetected){
                state = TURNING;
            } else if(completed) {
                state = FINISHED;
                finishTime = ros::Time::now();
            } 
            break;


        case TURNING: 
            initial_yaw = imu_yaw; 

            if(turn_direction.compare("left")){
                target_yaw = initial_yaw - 1.57 + left_offset;
                while(imu_yaw > target_yaw){
                    speed = turn_speed;
                    steering = turn_steering;
                    publishSpeedandSteering();
                }
                state = RUNNING_PLANNER;


            }else if(turn_direction.compare("right")){
                target_yaw = initial_yaw + 1.57 + right_offset;
                while(imu_yaw < target_yaw){
                    speed = turn_speed;
                    steering = turn_steering;
                    publishSpeedandSteering();

                }
                state = RUNNING_PLANNER;

            }else if(turn_direction.compare("forward")){
                ros::Time start_time = ros::Time::now();
                ros::Duration timeout(forward_timeout); // Timeout of 2 seconds
                while(ros::Time::now() - start_time < timeout) {
                    speed = planSpeed;
                    steering = planSteering;
                    publishSpeedandSteering();
                }
                state = RUNNING_PLANNER;
            }


            break; 
        case FINISHED:
            ROS_INFO("Finished");
            if( (ros::Time::now() - finishTime) > ros::Duration(0.5)) {
                speed = 0.0;
                steering = 0.0;
            }
            break;


        default:
            ROS_WARN("URC state machine defaulted");
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


void resetCB(const rr_platform::race_reset &reset_msg) {
    state = WAITING_FOR_START;
    raceStarted = false;
    updateState();
}

void imuCB(const rr_platform::axesConstPtr& msg){
     imu_yaw = msg->yaw;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_controller");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    state = WAITING_FOR_START;
    planSpeed = 0.0;
    planSteering = 0.0;
    completed = false;

    nhp.getParam("startSignal", startSignal);
    nhp.getParam("resetSignal", resetSignal);
    nhp.getParam("turn_speed", turn_speed);
    nhp.getParam("turn_steering", turn_steering);
    nhp.getParam("left_offset", left_offset);
    nhp.getParam("right_offset", right_offset);
    nhp.getParam("forward_timeout", right_offset);

    auto planSpeedSub = nh.subscribe("plan/speed", 1, planSpeedCB);
    auto planSteerSub = nh.subscribe("plan/steering", 1, planSteerCB);
    auto startLightSub = nh.subscribe(startSignal, 1, startLightCB);
    auto imuSub = nh.subscribe("/axes", 1, imuCB);

    auto resetSub = nh.subscribe(resetSignal, 1, resetCB);

    speedPub = nh.advertise<rr_platform::speed>("/speed", 1);
    steerPub = nh.advertise<rr_platform::steering>("/steering", 1);


    ros::Rate rate(30.0);
    while(ros::ok()) {
        ros::spinOnce();
        updateState();
        publishSpeedandSteering();
        rate.sleep();
    }

    return 0;
}