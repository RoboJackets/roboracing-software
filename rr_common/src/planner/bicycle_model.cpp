#include "bicycle_model.h"

#include <numeric>


namespace rr {

BicycleModel::BicycleModel(double wheel_base, double max_lateral_accel,
             double distance_increment, double max_speed, double max_steer_rate,
             const std::vector<int>& segment_sections)
    : wheel_base_(wheel_base), max_lateral_accel_(max_lateral_accel),
      distance_increment_(distance_increment), max_steer_rate_(max_steer_rate),
      segment_sections_(segment_sections), max_speed_(max_speed) {}


void BicycleModel::RollOutPath(std::vector<PathPoint>& path_points, const std::vector<double>& control) {
  const size_t n_path_segments = control.size();

  const int n_path_points = std::accumulate(segment_sections_.begin(), segment_sections_.end(), 1);
  if (path_points.size() != n_path_points) {
    path_points.resize(static_cast<size_t>(n_path_points));
  }

  path_points[0].pose.x = 0;
  path_points[0].pose.y = 0;
  path_points[0].pose.theta = 0;
  path_points[0].steer = control[0];
  path_points[0].speed = SteeringToSpeed(control[0]);

  double real_steer = 0;

  int i = 1;
  for (int segment = 0; segment < n_path_segments; segment++) {
    double ideal_steer = control[segment];
    double speed = SteeringToSpeed(real_steer);
    const int n_sections = segment_sections_[segment];

    double dt = distance_increment_ / speed;
    double max_steer_change = max_steer_rate_ * dt;

    for (auto j = i; j < i + n_sections; j++) {
      const Pose& last_pose = path_points[j - 1].pose;
      PathPoint& path_point = path_points[j];

      double ideal_steer_update = ideal_steer - real_steer;
      real_steer += std::min(std::max(-max_steer_change, ideal_steer_update), max_steer_change);

      StepKinematics(path_point.pose, last_pose, real_steer);
      path_point.steer = ideal_steer;
      path_point.speed = speed;
    }

    i += n_sections;
  }
}

void BicycleModel::StepKinematics(Pose& pose, const Pose& last_pose, double steer_angle) {
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

double BicycleModel::SteeringToSpeed(double steer_angle) {
  steer_angle = std::abs(steer_angle);

  double out;
  if (steer_angle < 1e-3) {
    out = max_speed_;
  } else {
    double vRaw = std::sqrt(max_lateral_accel_ * wheel_base_
                            / std::sin(steer_angle));
    out = std::min(vRaw, max_speed_);
  }
  return out;
}

}  // namespace rr
