#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/axes.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

const double PI = 3.14;

ros::Publisher steerPub;
ros::Publisher speedPub;

double speed;
double steering;

int imu_quadrant; 
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

void makeTurn(){
    initial_yaw = imu_yaw; 
    checkQuadrant(); 

    if(turn_direction.compare("left")){

        if(imu_quadrant == 4){
            target_yaw = initial_yaw - 1.5*PI - left_offset;
        }else{
            target_yaw = initial_yaw + .5*PI + left_offset;
        }
        
        while(imu_yaw != target_yaw){
            speed = turn_speed;
            steering = left_turn_steering;
            publishSpeedandSteering();
        }

    }else if(turn_direction.compare("right")){

        if(imu_quadrant == 1){
            target_yaw = initial_yaw + 1.5*PI + right_offset;
        }else{
            target_yaw = initial_yaw - .5*PI + right_offset;
        }

        while(imu_yaw != target_yaw){
            speed = turn_speed;
            steering = right_turn_steering;
            publishSpeedandSteering();
        }

    }else if(turn_direction.compare("forward")){
        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(forward_timeout); // Timeout of 2 seconds
        while(ros::Time::now() - start_time < timeout) {
            speed = turn_speed;
            steering = 0;
            publishSpeedandSteering();
        }
    }
}

void imuCB(const rr_platform::axesConstPtr& msg){
     imu_yaw = setInRange(msg->yaw);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turning_test");

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
        speed = 1.0;
        steering = 0.0;
        publishSpeedandSteering();
    }   

    makeTurn(); 
    return 0;
}