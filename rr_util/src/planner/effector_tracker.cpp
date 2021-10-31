/**
 * Helps the planner keep track of the vehicle's current speed and steering angle.
 *
 * Note: Pointers to the planner's speed and steering messages are stored
 * as global variables and get updated within the planner node (outside this class).
 * This means the allowing for 'guessing between updates' is equivalent to just keeping
 * track of the planner's last speed and steering message.
 *
 * @author Daniel Martin
 */

#include "rr_common/planning/effector_tracker.h"

namespace rr {

EffectorTracker::EffectorTracker(ros::NodeHandle nh, rr_msgs::speedPtr& speed_msg, rr_msgs::steeringPtr& steering_msg)
      : last_speed_msg(speed_msg), last_steering_msg(steering_msg), speed_updated(false), steering_updated(false) {
    std::string speed_topic, steering_topic, speed_type, steering_type;
    assertions::getParam(nh, "speed/message_topic", speed_topic);
    assertions::getParam(nh, "speed/message_type", speed_type);
    assertions::getParam(nh, "speed/guessing_between_updates", speed_guessing_between_updates);

    assertions::getParam(nh, "steering/message_topic", steering_topic);
    assertions::getParam(nh, "steering/message_type", steering_type);
    assertions::getParam(nh, "steering/guessing_between_updates", steering_guessing_between_updates);

    if (speed_type == "speed") {
        speed_sub = nh.subscribe(speed_topic, 1, &EffectorTracker::extractSpeedFromSpeedMsg, this);
    } else if (speed_type == "chassis") {
        speed_sub = nh.subscribe(speed_topic, 1, &EffectorTracker::extractSpeedFromChassisMsg, this);
    } else if (speed_type == "odometry") {
        speed_sub = nh.subscribe(speed_topic, 1, &EffectorTracker::extractSpeedFromOdometryMsg, this);
    } else {
        ROS_ERROR_STREAM("[EffectorTracker] Error: unknown speed message type \"" << speed_type << "\"");
        ROS_ERROR_STREAM("[EffectorTracker] Using planner's last published speed message");
        speed_sub = nh.subscribe(speed_topic, 1, &EffectorTracker::extractSpeedFromSpeedMsg, this);
    }

    if (steering_type == "steering") {
        steering_sub = nh.subscribe(steering_topic, 1, &EffectorTracker::extractAngleFromSteeringMsg, this);
    } else if (steering_type == "chassis") {
        steering_sub = nh.subscribe(steering_topic, 1, &EffectorTracker::extractAngleFromChassisMsg, this);
    } else {
        ROS_ERROR_STREAM("[EffectorTracker] Error: unknown steering message type \"" << speed_type << "\"");
        ROS_ERROR_STREAM("[EffectorTracker] Using planner's last published steering message");
        steering_sub = nh.subscribe(steering_topic, 1, &EffectorTracker::extractAngleFromSteeringMsg, this);
    }
}

void EffectorTracker::extractSpeedFromSpeedMsg(const rr_msgs::speedConstPtr& speed_msg) {
    speed = speed_msg->speed;
    speed_updated = true;
}

void EffectorTracker::extractSpeedFromChassisMsg(const rr_msgs::chassis_stateConstPtr& chassis_state_msg) {
    speed = chassis_state_msg->speed_mps;
    speed_updated = true;
}

void EffectorTracker::extractSpeedFromOdometryMsg(const nav_msgs::OdometryConstPtr& odometry_msg) {
    speed = odometry_msg->twist.twist.linear.x;
    speed_updated = true;
}

void EffectorTracker::extractAngleFromSteeringMsg(const rr_msgs::steeringConstPtr& steering_msg) {
    angle = steering_msg->angle;
    steering_updated = true;
}

void EffectorTracker::extractAngleFromChassisMsg(const rr_msgs::chassis_stateConstPtr& chassis_state_msg) {
    angle = chassis_state_msg->steer_rad;
    steering_updated = true;
}

double EffectorTracker::getSpeed() {
    if (speed_guessing_between_updates && !speed_updated) {
        return last_speed_msg->speed;
    }
    speed_updated = false;
    return speed;
}

double EffectorTracker::getAngle() {
    if (steering_guessing_between_updates && !steering_updated) {
        return last_steering_msg->angle;
    }
    steering_updated = false;
    return angle;
}

}  // namespace rr
