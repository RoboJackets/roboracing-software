#include <rr_common/bicycle_model.h>

namespace rr {

BicycleModel::BicycleModel(const ros::NodeHandle& nh,
                           const std::shared_ptr<rr::LinearTrackingFilter>& steer_model_ptr) {
    assertions::getParam(nh, "n_segments", n_segments_);
    assertions::getParam(nh, "segment_size", segment_size_);
    assertions::getParam(nh, "distance_increment", distance_increment_);
    assertions::getParam(nh, "wheel_base", wheel_base_);
    assertions::getParam(nh, "lateral_accel", max_lateral_accel_);
    assertions::getParam(nh, "max_speed", max_speed_);

    steering_model_ = steer_model_ptr;
}

void BicycleModel::RollOutPath(const std::vector<double>& control, std::vector<PathPoint>& path_points) const {
    const size_t path_size = 1 + segment_size_ * n_segments_;
    if (path_points.size() != path_size) {
        path_points.resize(static_cast<size_t>(path_size));
    }

    path_points[0].pose.x = 0;
    path_points[0].pose.y = 0;
    path_points[0].pose.theta = 0;
    path_points[0].steer = control[0];
    path_points[0].speed = SteeringToSpeed(control[0]);

    rr::LinearTrackingFilter steering_model_temp = *steering_model_;  // copy

    int i = 1;
    for (int segment = 0; segment < n_segments_; segment++) {
        double speed = SteeringToSpeed(steering_model_temp.GetValue());
        double dt = distance_increment_ / speed;
        steering_model_temp.SetTarget(control[segment]);

        for (auto j = i; j < i + segment_size_; j++) {
            const Pose& last_pose = path_points[j - 1].pose;
            PathPoint& path_point = path_points[j];

            steering_model_temp.Update(steering_model_temp.GetLastUpdateTime() + dt);

            StepKinematics(last_pose, steering_model_temp.GetValue(), path_point.pose);
            path_point.steer = control[segment];
            path_point.speed = speed;
        }

        i += segment_size_;
    }
}

void BicycleModel::StepKinematics(const Pose& last_pose, double steer_angle, Pose& pose) const {
    double deltaX, deltaY, deltaTheta;

    if (std::abs(steer_angle) < 1e-3) {
        deltaX = distance_increment_;
        deltaY = 0;
        deltaTheta = 0;
    } else {
        double turn_radius = wheel_base_ / std::sin(std::abs(steer_angle));
        double tempTheta = distance_increment_ / turn_radius;
        deltaX = turn_radius * std::cos(M_PI / 2 - tempTheta);
        if (steer_angle < 0) {
            deltaY = turn_radius - turn_radius * std::sin(M_PI / 2 - tempTheta);
        } else {
            deltaY = -(turn_radius - turn_radius * std::sin(M_PI / 2 - tempTheta));
        }
        deltaTheta = distance_increment_ / wheel_base_ * std::sin(-steer_angle);
    }

    pose.x = last_pose.x + deltaX * std::cos(last_pose.theta) - deltaY * std::sin(last_pose.theta);
    pose.y = last_pose.y + deltaX * std::sin(last_pose.theta) + deltaY * std::cos(last_pose.theta);
    pose.theta = last_pose.theta + deltaTheta;
}

double BicycleModel::SteeringToSpeed(double steer_angle) const {
    steer_angle = std::abs(steer_angle);

    double out;
    if (steer_angle < 1e-3) {
        out = max_speed_;
    } else {
        double vRaw = std::sqrt(max_lateral_accel_ * wheel_base_ / std::sin(steer_angle));
        out = std::min(vRaw, max_speed_);
    }
    return out;
}

}  // namespace rr
