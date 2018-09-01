#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/chassis_state.h>
#include <rr_platform/axes.h>

#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <math.h>

ros::Publisher speedPub;
ros::Publisher steeringPub;

double straightYaw; 
double currentYaw = 0;


const double kP = 0;
const double kI = 0;
const double kD = 0;

double error = 0; 
double error_prior = 0;

double time_prior = 0;  // used to calculate integration interval
ros::Time time_now; 
double dt = 0; 
double integral = 0;
double derivative; 

double control_output = 0; 
 

bool go = false;
bool straightYawSet = false;


void imuCallback(const rr_platform::axesConstPtr& msg) {
   
    /*
    auto orientation = msg->orientation;
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(orientation, tf_quat);
    auto currentYaw = tf::getYaw(tf_quat);
    */


    if(!straightYawSet){
        straightYaw = msg->yaw;
        straightYawSet = true; 
    }

    currentYaw = msg->yaw;

    time_now = msg->header.stamp;

    dt = time_now.toSec() - time_prior;

    time_prior = time_now.toSec();
    
    

    
    error = straightYaw - currentYaw;

    integral = integral + error*dt; 
    derivative = (error - error_prior)/dt; 
    control_output = kP*error + kI*integral + kD*derivative;  

    error_prior = error; 


    ROS_INFO_STREAM(currentYaw);
}

void chassisCB(const rr_platform::chassis_stateConstPtr& msg) {
    go = msg->mux_automatic;
}

void publishSpeed(const float &desiredSpeed) {
   
    rr_platform::speed speedMsg;
    speedMsg.header.stamp = ros::Time::now();
    speedMsg.speed = desiredSpeed;
    speedPub.publish(speedMsg);
    //ROS_INFO("published");

  

    rr_platform::steering steeringMsg;
    steeringMsg.header.stamp = ros::Time::now();
    steeringMsg.angle = control_output;
    steeringPub.publish(steeringMsg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "qualify");
    ros::NodeHandle nh;

    speedPub = nh.advertise<rr_platform::speed>("/speed", 1);
    steeringPub = nh.advertise<rr_platform::steering>("/steering", 1);

    ros::Subscriber chassisSub = nh.subscribe("/chassis_state", 1, chassisCB);
    ros::Subscriber imuSub = nh.subscribe("/axes", 1, imuCallback);

    while (!go && ros::ok()) {
        ros::spinOnce();


        publishSpeed(1.0);
    }

   // publishSpeed(1.0);
    //ros::Duration(5).sleep();
    //publishSpeed(0.0);


    return 0;
}
