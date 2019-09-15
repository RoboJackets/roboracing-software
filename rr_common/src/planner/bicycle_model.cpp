#include "planner/bicycle_model.h"

#include <numeric>

namespace rr {

BicycleModel::BicycleModel(double wheel_base, double max_lateral_accel, double distance_increment, double max_speed,
                           double max_steer_rate, const std::vector<int>& segment_sections)
  : wheel_base(wheel_base)
  , max_lateral_accel(max_lateral_accel)
  , distance_increment(distance_increment)
  , max_steer_rate(max_steer_rate)
  , segment_sections(segment_sections)
  , max_speed(max_speed)
  , current_steering_(0) {
    last_steering_update_ = ros::Time::now();
}

void BicycleModel::RollOutPath(const std::vector<double>& control, std::vector<PathPoint>& path_points) {
    const size_t n_path_segments = control.size();

    const int n_path_points = std::accumulate(segment_sections.begin(), segment_sections.end(), 1);
    if (path_points.size() != n_path_points) {
        path_points.resize(static_cast<size_t>(n_path_points));
    }

    path_points[0].pose.x = 0;
    path_points[0].pose.y = 0;
    path_points[0].pose.theta = 0;
    path_points[0].steer = control[0];
    path_points[0].speed = SteeringToSpeed(control[0]);

    double real_steer = GetCurrentSteeringAngle();

    int i = 1;
    for (int segment = 0; segment < n_path_segments; segment++) {
        double ideal_steer = control[segment];
        double speed = SteeringToSpeed(real_steer);
        const int n_sections = segment_sections[segment];

        double dt = distance_increment / speed;

        for (auto j = i; j < i + n_sections; j++) {
            const Pose& last_pose = path_points[j - 1].pose;
            PathPoint& path_point = path_points[j];

            real_steer = IncrementSteering(real_steer, ideal_steer, dt);

            StepKinematics(last_pose, real_steer, path_point.pose);
            path_point.steer = ideal_steer;
            path_point.speed = speed;
        }

        i += n_sections;
    }
}

void BicycleModel::StepKinematics(const Pose& last_pose, double steer_angle, Pose& pose) {
    double deltaX, deltaY, deltaTheta;

    if (std::abs(steer_angle) < 1e-3) {
        deltaX = distance_increment;
        deltaY = 0;
        deltaTheta = 0;
    } else {
        double turn_radius = wheel_base / std::sin(std::abs(steer_angle));
        double tempTheta = distance_increment / turn_radius;
        deltaX = turn_radius * std::cos(M_PI / 2 - tempTheta);
        if (steer_angle < 0) {
            deltaY = turn_radius - turn_radius * std::sin(M_PI / 2 - tempTheta);
        } else {
            deltaY = -(turn_radius - turn_radius * std::sin(M_PI / 2 - tempTheta));
        }
        deltaTheta = distance_increment / wheel_base * std::sin(-steer_angle);
    }

    pose.x = last_pose.x + deltaX * std::cos(last_pose.theta) - deltaY * std::sin(last_pose.theta);
    pose.y = last_pose.y + deltaX * std::sin(last_pose.theta) + deltaY * std::cos(last_pose.theta);
    pose.theta = last_pose.theta + deltaTheta;
}

double BicycleModel::SteeringToSpeed(double steer_angle) {
    steer_angle = std::abs(steer_angle);

    double out;
    if (steer_angle < 1e-3) {
        out = max_speed;
    } else {
        double vRaw = std::sqrt(max_lateral_accel * wheel_base / std::sin(steer_angle));
        out = std::min(vRaw, max_speed);
    }
    return out;
}

double BicycleModel::IncrementSteering(double current, double target, double dt) {
    double max_steer_change = max_steer_rate * dt;
    double ideal_steer_update = target - current;

    return current + std::min(std::max(-max_steer_change, ideal_steer_update), max_steer_change);
}

void BicycleModel::UpdateSteeringAngle(double commanded_angle) {
    auto now = ros::Time::now();
    double dt = (now - last_steering_update_).toSec();

    if (dt > 0) {
        // if-statement covers time changes in sim or bag files
        current_steering_ = IncrementSteering(current_steering_, commanded_angle, dt);
    }

    last_steering_update_ = now;
}

double BicycleModel::GetCurrentSteeringAngle() {
    return current_steering_;
}

}  // namespace rr
