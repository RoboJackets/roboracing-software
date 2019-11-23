#include <rr_common/planning/bicycle_model.h>

namespace rr {

BicycleModel::BicycleModel(const ros::NodeHandle& nh, const std::shared_ptr<rr::LinearTrackingFilter>& steer_model_ptr,
                           const std::shared_ptr<rr::LinearTrackingFilter>& speed_model_ptr) {
    assertions::getParam(nh, "segment_size", segment_size_);
    assertions::getParam(nh, "dt", dt_);
    assertions::getParam(nh, "wheel_base", wheel_base_);
    assertions::getParam(nh, "lateral_accel", max_lateral_accel_);

    steering_model_ = steer_model_ptr;
    speed_model_ = speed_model_ptr;
}

void BicycleModel::RollOutPath(const Controls<1>& controls, TrajectoryRollout& rollout) const {
    const size_t path_size = 1 + (segment_size_ * controls.cols());
    if (rollout.path.size() != path_size) {
        rollout.path.resize(static_cast<size_t>(path_size));
    }

    rollout.path[0].pose.x = 0;
    rollout.path[0].pose.y = 0;
    rollout.path[0].pose.theta = 0;
    rollout.path[0].speed = speed_model_->GetValue();
    rollout.path[0].steer = steering_model_->GetValue();
    rollout.path[0].time = 0;

    rollout.apply_steering = controls(0);

    rr::LinearTrackingFilter steering_model_temp = *steering_model_;  // copy
    rr::LinearTrackingFilter speed_model_temp = *speed_model_;

    int i = 1;
    for (int segment = 0; segment < controls.cols(); segment++) {
        steering_model_temp.SetTarget(controls(segment));

        for (auto j = i; j < i + segment_size_; j++) {
            const PathPoint& last_path_point = rollout.path[j - 1];
            PathPoint& path_point = rollout.path[j];

            StepKinematics(last_path_point, path_point.pose);

            steering_model_temp.UpdateRawDT(dt_);

            path_point.steer = steering_model_temp.GetValue();
            path_point.speed = SteeringToSpeed(path_point.steer);
            path_point.time = last_path_point.time + dt_;
        }

        i += segment_size_;
    }

    speed_model_temp.Reset(rollout.path.back().speed, 0);
    for (int i = path_size - 1; i >= 1; --i) {
        speed_model_temp.SetTarget(rollout.path[i].speed);
        speed_model_temp.UpdateRawDT(-dt_);
        rollout.path[i - 1].speed = std::min(rollout.path[i - 1].speed, speed_model_temp.GetValue());
    }
    rollout.apply_speed = speed_model_temp.GetValue();
}

void BicycleModel::StepKinematics(const PathPoint& prev, Pose& next) const {
    double deltaX, deltaY, deltaTheta;
    double distance_increment = prev.speed * dt_;

    if (std::abs(prev.steer) < 1e-7) {
        deltaX = distance_increment;
        deltaY = 0;
        deltaTheta = 0;
    } else {
        double turn_radius = wheel_base_ / std::tan(std::abs(prev.steer));
        double tempTheta = distance_increment / turn_radius;
        deltaX = turn_radius * std::cos(M_PI / 2 - tempTheta);
        if (prev.steer < 0) {
            deltaY = turn_radius - turn_radius * std::sin(M_PI / 2 - tempTheta);
        } else {
            deltaY = -(turn_radius - turn_radius * std::sin(M_PI / 2 - tempTheta));
        }
        deltaTheta = distance_increment / wheel_base_ * std::sin(-prev.steer);
    }

    next.x = prev.pose.x + deltaX * std::cos(prev.pose.theta) - deltaY * std::sin(prev.pose.theta);
    next.y = prev.pose.y + deltaX * std::sin(prev.pose.theta) + deltaY * std::cos(prev.pose.theta);
    next.theta = prev.pose.theta + deltaTheta;
}

double BicycleModel::SteeringToSpeed(double steer_angle) const {
    steer_angle = std::abs(steer_angle);

    double out;
    if (steer_angle < 1e-3) {
        out = speed_model_->GetValMax();
    } else {
        double vRaw = std::sqrt(max_lateral_accel_ * wheel_base_ / std::sin(steer_angle));
        out = std::min(vRaw, speed_model_->GetValMax());
    }
    return out;
}

}  // namespace rr
