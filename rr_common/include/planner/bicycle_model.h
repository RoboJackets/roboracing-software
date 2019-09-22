#pragma once

#include <tuple>

#include <ros/time.h>

#include "planner_types.hpp"

namespace rr {

class BicycleModel {
  public:
    /**
     * Constructor. Units are metric standard: m, m/s, m/s^2, rad/s
     * @param wheel_base Distance from front axle to back axle
     * @param max_lateral_accel Maximum acceptable centripetal acceleration
     * @param distance_increment Distance between consecutive path points in
     * rollout
     * @param max_speed Straight-line desired speed
     * @param max_steer_rate Rate at which the platform can change its steering
     * angle
     * @param segment_sections Vector of sizes of
     */
    BicycleModel(double wheel_base, double max_lateral_accel, double distance_increment, double max_speed,
                 double max_steer_rate, const std::vector<int>& segment_sections);

    BicycleModel() = default;

    /**
     * roll out a trajectory through the world
     * @param control Vector of steering angles
     * @param path_points Output parameter into which points are placed
     */
    void RollOutPath(const std::vector<double>& control, std::vector<PathPoint>& path_points);

    /**
     * move the tracked steering angle of the car towards the value that
     * the car has been commanded to steer. This tracks the ROS time it is called,
     * so it should be used just before RollOutPath
     * @param commanded_angle Most recent steering angle commanded to platform
     */
    void UpdateSteeringAngle(double commanded_angle);

    /**
     * Estimate the current steering angle based on past commands
     * @return the angle we think the front wheels are currently at
     */
    double GetCurrentSteeringAngle();

  private:
    /**
     * Calculate a desired speed from a steering angle based on
     * a maximum acceptable lateral acceleration. See
     * https://www.desmos.com/calculator/7nh83g8afj
     * @param steer_angle Proposed steering value to publish
     * @return Maximum acceptable speed given the turning angle
     */
    double SteeringToSpeed(double steer_angle);

    /**
     * Calculate one distance-step (not timestep) of "simulated" vehicle motion.
     * Bicycle-model forward kinematics happens here.
     * @param last_pose Initial (x,y,theta) for the distance step
     * @param steer_angle Constant steering angle for the distance step
     * @param pose Out param for the resulting pose
     */
    void StepKinematics(const Pose& last_pose, double steer_angle, Pose& pose);

    /**
     * Trivial linear model supporting UpdateSteeringAngle
     * @param current Current estimate of the steering angle
     * @param target Steering setpoint commanded to platform
     * @param dt Time elapsed since current estimate was updated
     * @return Estimated steering angle of platform after dt has elapsed
     */
    double IncrementSteering(double current, double target, double dt);

    const double wheel_base;
    const double max_lateral_accel;
    const std::vector<int> segment_sections;
    const double distance_increment;
    const double max_speed;
    const double max_steer_rate;

    double current_steering_;
    ros::Time last_steering_update_;
};

}  // namespace rr
