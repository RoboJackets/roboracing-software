#pragma once

#include <nav_msgs/Odometry.h>
#include <parameter_assertions/assertions.h>
#include <ros/ros.h>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>
#include <rr_msgs/chassis_state.h>

namespace rr {

class EffectorTracker {
  public:
    EffectorTracker(ros::NodeHandle nh, rr_msgs::speedPtr& speed_message, rr_msgs::steeringPtr& steering_message);

    double getSpeed();
    double getAngle();

  private:
    void extractSpeedFromSpeedMsg(const rr_msgs::speedConstPtr& speed_msg);
    void extractSpeedFromChassisMsg(const rr_msgs::chassis_stateConstPtr& chassis_state_msg);
    void extractSpeedFromOdometryMsg(const nav_msgs::OdometryConstPtr& odometry_msg);
    void extractAngleFromSteeringMsg(const rr_msgs::steeringConstPtr& steering_msg);
    void extractAngleFromChassisMsg(const rr_msgs::chassis_stateConstPtr& chassis_state_msg);

    ros::Subscriber speed_sub;
    ros::Subscriber steering_sub;
    rr_msgs::speedPtr last_speed_msg;
    rr_msgs::steeringPtr last_steering_msg;
    bool speed_guessing_between_updates;
    bool steering_guessing_between_updates;
    double speed;
    double angle;
    bool speed_updated;
    bool steering_updated;
};

}  // namespace rr
