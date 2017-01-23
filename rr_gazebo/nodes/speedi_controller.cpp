#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

double speed_set_point = 0.0;
double speed_measured = 0.0;
double speed_last_error = 0.0;
double speed_P = 1.0;
double speed_D = 0.05;

double steer_set_point = 0.0;

constexpr double wheel_circumference = 2.0 * M_PI * 0.036;

void speedCallback(const rr_platform::speedConstPtr &msg) {
    speed_set_point = msg->speed;
}

void steeringCallback(const rr_platform::steeringConstPtr &msg) {
    steer_set_point = msg->angle;
}

void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg) {

    auto iter = std::find(msg->name.begin(), msg->name.end(), std::string{"chassis_to_back_axle"});

    if(iter != msg->name.end()) {
        auto index = std::distance(msg->name.begin(),iter);

        speed_measured = (-msg->velocity[index]) * ( wheel_circumference / ( 2 * M_PI ) );
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "speedi_controller");

    ros::NodeHandle handle;

    ros::Publisher backAxleEffortPublisher = handle.advertise<std_msgs::Float64>("/roboracing/axle_effort_controller/command", 1);

    ros::Publisher leftSteeringPublisher = handle.advertise<std_msgs::Float64>("/roboracing/left_steer_position_controller/command", 1);

    ros::Publisher rightSteeringPublisher = handle.advertise<std_msgs::Float64>("/roboracing/right_steer_position_controller/command", 1);

    auto speedSub = handle.subscribe("/speed", 1, speedCallback);

    auto steerSub = handle.subscribe("/steering", 1, steeringCallback);

    auto stateSub = handle.subscribe("/roboracing/joint_states", 1, jointStateCallback);

    ros::Rate rate{30};
    while(ros::ok()) {
        ros::spinOnce();

        auto error = speed_measured - speed_set_point;
        auto dError = error - speed_last_error;

        auto effort = speed_P * error - speed_D * dError;

        std_msgs::Float64 axleMsg;
        axleMsg.data = effort;
        backAxleEffortPublisher.publish(axleMsg);

        std_msgs::Float64 leftSteerMsg;
        leftSteerMsg.data = steer_set_point;
        leftSteeringPublisher.publish(leftSteerMsg);

        std_msgs::Float64 rightSteerMsg;
        rightSteerMsg.data = steer_set_point;
        rightSteeringPublisher.publish(rightSteerMsg);

        rate.sleep();
    }

    return 0;
}