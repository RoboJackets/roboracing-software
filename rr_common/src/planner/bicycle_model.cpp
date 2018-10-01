#include "bicycle_model.h"

namespace rr {

BicycleModel::BicycleModel(double wheel_base, double max_lateral_accel,
             double distance_increment, double max_speed,
             const std::vector<double>& segment_distances)
    : wheel_base_(wheel_base), max_lateral_accel_(max_lateral_accel),
      distance_increment_(distance_increment),
      segment_distances_(segment_distances), max_speed_(max_speed) {}

std::vector<PathPoint> BicycleModel::RollOutPath(
    const std::vector<double>& control) {
  auto n_path_segments = control.size();

  std::vector<PathPoint> path_points;
  double begin_speed = SteeringToSpeed(control[0]);
  path_points.emplace_back(Pose{0, 0, 0}, control[0], begin_speed);

  for (int segment = 0; segment < n_path_segments; segment++) {
    double steer = control[segment];
    double speed = SteeringToSpeed(steer);
    const double& seg_dist = segment_distances_[segment];
    for (double dist = 0.0; dist < seg_dist; dist += distance_increment_) {
      const Pose& last_pose = path_points.back().pose;
      path_points.emplace_back(StepKinematics(last_pose, steer), steer, speed);
    }
  }

  return path_points;
}

Pose BicycleModel::StepKinematics(const Pose& pose, double steer_angle) {
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

  Pose out = pose;
  out.x += deltaX * std::cos(pose.theta) - deltaY * std::sin(pose.theta);
  out.y += deltaX * std::sin(pose.theta) + deltaY * std::cos(pose.theta);
  out.theta += deltaTheta;
  return out;
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

}