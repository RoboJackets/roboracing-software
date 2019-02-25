#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/axes.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

ros::Publisher steerPub;
ros::Publisher speedPub;

double speed;
double steering;

double imu_yaw;
double initial_yaw; 
double target_yaw;

double left_offset; 
double right_offset; 
double turn_speed; 
double left_turn_steering; 
double right_turn_steering; 


double forward_timeout;

std::string turn_direction;
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

void makeTurn(){
    initial_yaw = imu_yaw; 
    if(turn_direction.compare("left")){
        target_yaw = initial_yaw + 1.57 + left_offset;
        while(imu_yaw > target_yaw){
            speed = turn_speed;
            steering = left_turn_steering;
            publishSpeedandSteering();
        }
        speed = 0.0; 
        steering = 0.0;

    }else if(turn_direction.compare("right")){
        target_yaw = initial_yaw - 1.57 + right_offset;
        while(imu_yaw < target_yaw){
            speed = turn_speed;
            steering = right_turn_steering;
            publishSpeedandSteering();
        }
        speed = 0.0; 
        steering = 0.0;

    }else if(turn_direction.compare("forward")){
        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(forward_timeout); // Timeout of 2 seconds
        while(ros::Time::now() - start_time < timeout){
            speed = turn_speed;
            steering = 0;
            publishSpeedandSteering();
        }   
    }
}

void imuCB(const rr_platform::axesConstPtr& msg){
     imu_yaw = msg->yaw;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_controller");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    nhp.getParam("turn_speed", turn_speed);
    nhp.getParam("left_turn_steering", left_turn_steering);
    nhp.getParam("right_turn_steering", right_turn_steering);
    nhp.getParam("left_offset", left_offset);
    nhp.getParam("right_offset", right_offset);
    nhp.getParam("forward_timeout", right_offset);
    nhp.getParam("turn_direction", turn_direction);

    auto imuSub = nh.subscribe("/axes", 1, imuCB);
    
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(forward_timeout); 
    while(ros::Time::now() - start_time < timeout){
        speed = 0.0;
        steering = 0.0;
        publishSpeedandSteering();
    }   
    makeTurn(); 
    return 0;
}