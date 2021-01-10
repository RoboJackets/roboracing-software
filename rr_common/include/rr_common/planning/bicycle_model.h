#pragma once

#include <rr_common/linear_tracking_filter.hpp>
#include <tuple>

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
    explicit BicycleModel(const ros::NodeHandle& nh, const std::shared_ptr<rr::LinearTrackingFilter>& steer_model_ptr,
                          const std::shared_ptr<rr::LinearTrackingFilter>& speed_model_ptr);

    /**
     * roll out a trajectory through the world
     * @param control Control vector, interpretation depends on planning scenario
     * @param path_points Output parameter into which points are placed
     */
    void RollOutPath(const Controls<1>& controls, TrajectoryRollout& rollout) const;

    //    void RollOutPath(const Controls<2>& controls, std::vector<PathPoint>& path_points) const;
    double getDt() const;
  private:
    /**
     * Calculate a desired speed from a steering angle based on
     * a maximum acceptable lateral acceleration. See
     * https://www.desmos.com/calculator/7nh83g8afj
     * @param steer_angle Proposed steering value to publish
     * @return Maximum acceptable speed given the turning angle
     */
    [[nodiscard]] double SteeringToSpeed(double steer_angle) const;

    /**
     * Calculate one distance-step (not timestep) of "simulated" vehicle motion.
     * Bicycle-model forward kinematics happens here.
     * @param last_pose Initial (x,y,theta) for the distance step
     * @param steer_angle Constant steering angle for the distance step
     * @param pose Out param for the resulting pose
     */
    void StepKinematics(const PathPoint& prev, Pose& next) const;

    double wheel_base_;
    double max_lateral_accel_;
    int segment_size_;
    double dt_;

    std::shared_ptr<rr::LinearTrackingFilter> steering_model_;
    std::shared_ptr<rr::LinearTrackingFilter> speed_model_;
};

}  // namespace rr
