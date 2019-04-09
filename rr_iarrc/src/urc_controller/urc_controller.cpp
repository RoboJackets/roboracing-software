#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/axes.h>
#include <rr_platform/race_reset.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#include <stdlib.h>

const int WAITING_FOR_START = 0;
const int RUNNING_PLANNER = 1;
const int FINISHED = 2;
const int TURNING = 3; 
const float PI = 3.14; 

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

int imu_quadrant; 
float imu_yaw, initial_yaw, target_yaw;

double left_offset; 
double right_offset; 
double turn_speed;
double left_turn_steering; 
double right_turn_steering; 

double forward_timeout;

std::string turn_direction = "right";

/*
// As you turn counterclockwise, the yaw value received from the IMU ranges from [0, 3.14] U [-3.14, 0]
// This function returns a new yaw value in the range of [0 ,6.28]
*/
double setInRange(double yaw_read){
    double new_yaw; 
    if(yaw_read < 0){
        new_yaw = yaw_read + 2*PI; 
        return new_yaw;
    }else{
        return yaw_read;
    }
}

/*
// Checks to see in which quadrant the current yaw reading is in. Turning right while in Q1, and turning left in Q4 are special cases.
*/
void checkQuadrant(){
    if(imu_yaw <= 2*PI && imu_yaw >= 1.5*PI ){
        imu_quadrant = 4;
    }else if(imu_yaw >= 0 && imu_yaw <= .5*PI){
        imu_quadrant = 1;
    }
}

// Used to calculate the distance from the target IMU yaw to the current IMU yaw within range of [0 ,6.28]
bool comp(int a, int b) { 
    return (a < b); 
}

float angleDist(float target, float current){
    float diff = target - current;  
    float min = std::min({std::abs(diff), std::abs(diff - 2*PI),std::abs(diff + 2*PI)}, comp); 
    return min; 
}

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
            } 
            break;

        case TURNING: 
            ros::spinOnce();
            checkQuadrant(); 
            initial_yaw = imu_yaw;

            if(turn_direction.compare("left") == 0){

                if(imu_quadrant == 4){
                    target_yaw = initial_yaw - 1.5*PI - left_offset;
                }else{
                    target_yaw = initial_yaw + .5*PI + left_offset;
                }

                while(1){
                    if(angleDist(target_yaw, imu_yaw) <= 0.015){
                        break;
                    }

                    speed = turn_speed;
                    steering = left_turn_steering;
                    publishSpeedandSteering();
                    ros::spinOnce();
                }

            }else if(turn_direction.compare("right") == 0){

                if(imu_quadrant == 1){
                    target_yaw = initial_yaw + 1.5*PI + right_offset;
                }else{
                    target_yaw = initial_yaw - .5*PI + right_offset;
                }

                while(1){
                    if(angleDist(target_yaw, imu_yaw) <= 0.015){
                        break;
                    }

                    speed = turn_speed;
                    steering = right_turn_steering;
                    publishSpeedandSteering();
                    ros::spinOnce();
                }

            }else if(turn_direction.compare("forward") == 0){
                ros::Time start_time = ros::Time::now();
                ros::Duration timeout(forward_timeout); // Timeout of 2 seconds
                while(ros::Time::now() < start_time + timeout) {
                    speed = turn_speed;
                    steering = 0;
                    publishSpeedandSteering();
                }
            }
            state = RUNNING_PLANNER;
            break; 

        case FINISHED:
            ROS_INFO("Finished");
            speed = 0.0;
            steering = 0.0;
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
     imu_yaw = setInRange(msg->yaw);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "urc_controller");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    state = WAITING_FOR_START;
    planSpeed = 0.0;
    planSteering = 0.0;
    completed = false;

    nhp.getParam("startSignal", startSignal);
    nhp.getParam("resetSignal", resetSignal);
    nhp.getParam("turn_speed", turn_speed);
    nhp.getParam("left_turn_steering", left_turn_steering);
    nhp.getParam("right_turn_steering", right_turn_steering);
    nhp.getParam("left_offset", left_offset);
    nhp.getParam("right_offset", right_offset);
    nhp.getParam("forward_timeout", forward_timeout);

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