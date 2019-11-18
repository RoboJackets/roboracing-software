#pragma once

#include <tuple>

#include <ros/time.h>

#include <rr_common/linear_tracking_filter.hpp>
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
    explicit BicycleModel(const ros::NodeHandle& nh, const std::shared_ptr<rr::LinearTrackingFilter>& steer_model_ptr);

    /**
     * roll out a trajectory through the world
     * @param control Vector of steering angles
     * @param path_points Output parameter into which points are placed
     */
    void RollOutPath(const std::vector<double>& control, std::vector<PathPoint>& path_points) const;

  private:
    /**
     * Calculate a desired speed from a steering angle based on
     * a maximum acceptable lateral acceleration. See
     * https://www.desmos.com/calculator/7nh83g8afj
     * @param steer_angle Proposed steering value to publish
     * @return Maximum acceptable speed given the turning angle
     */
    double SteeringToSpeed(double steer_angle) const;

    /**
     * Calculate one distance-step (not timestep) of "simulated" vehicle motion.
     * Bicycle-model forward kinematics happens here.
     * @param last_pose Initial (x,y,theta) for the distance step
     * @param steer_angle Constant steering angle for the distance step
     * @param pose Out param for the resulting pose
     */
    void StepKinematics(const Pose& last_pose, double steer_angle, Pose& pose) const;

    double wheel_base_;
    double max_lateral_accel_;
    int n_segments_;
    int segment_size_;
    double distance_increment_;
    double max_speed_;

    std::shared_ptr<rr::LinearTrackingFilter> steering_model_;
};

}  // namespace rr
