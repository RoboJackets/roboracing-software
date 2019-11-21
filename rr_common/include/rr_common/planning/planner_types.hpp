#pragma once

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Core>

namespace rr {

/**
 * Pose: 2D state vector
 */
struct Pose {
    double x;
    double y;
    double theta;

    Pose(double x, double y, double theta) : x(x), y(x), theta(theta) {}
    Pose() : x(0), y(0), theta(0) {}
};

std::ostream& operator<<(std::ostream& out, const Pose& p) {
    return out << "Pose(x=" << p.x << ", y=" << p.y << ", theta=" << p.theta << ")";
}

/**
 * PathPoint: Pose tagged with steering and speed info
 */
struct PathPoint {
    Pose pose;
    double steer;
    double speed;
    double time;

    PathPoint(const Pose& pose, double steer, double speed, double t)
          : pose(pose), steer(steer), speed(speed), time(t) {}
    PathPoint() : pose(), steer(0), speed(0), time(0) {}
};

std::ostream& operator<<(std::ostream& out, const PathPoint& p) {
    return out << "PathPoint(t=" << p.time << ", pose=" << p.pose << ", steer=" << p.steer << ", speed=" << p.speed
               << ")";
}

template <int R, int C>
using Matrix = Eigen::Matrix<double, R, C>;

template <int N>
using Vector = Eigen::Matrix<double, N, 1>;

template <int ctrl_dim>
using Controls = Eigen::Matrix<double, ctrl_dim, -1, Eigen::RowMajor, ctrl_dim, 10>;

/**
 * object representing a planned trajectory
 */
template <int ctrl_dim>
struct TrajectoryPlan {
    Controls<ctrl_dim> control;   // input to a path
    std::vector<PathPoint> path;  // results of path rollout
    double cost;                  // result of applying cost function
    bool has_collision;
};

template <int ctrl_dim>
using CostFunction = std::function<double(const Controls<ctrl_dim>&)>;

}  // namespace rr
