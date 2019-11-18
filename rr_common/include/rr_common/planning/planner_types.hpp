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

/**
 * PlannedPath: object representing a planned trajectory
 */
struct OptimizedTrajectory {
    std::vector<double> control;  // input to a path
    std::vector<PathPoint> path;  // results of path rollout
    double cost;                  // result of applying cost function
    bool has_collision;
};

/**
 * CenteredBox: container for forward, backward, left, and right offsets
 */
struct CenteredBox {
    double front;  // distance from origin to front edge
    double back;   // ditto for back edge, etc.
    double left;
    double right;
};

template <int R, int C>
using Matrix = Eigen::Matrix<double, R, C>;

template <int N>
using Vector = Matrix<N, 1>;

template <int ctrl_dim>
using Controls = Matrix<ctrl_dim, -1>;

template <int ctrl_dim>
using CostFunction = std::function<double(const Controls<ctrl_dim>&)>;

}  // namespace rr
