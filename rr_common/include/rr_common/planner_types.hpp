#pragma once

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rr {

using PCLMap = pcl::PointCloud<pcl::PointXYZ>;
using PCLPoint = pcl::PointXYZ;

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
    return out << "Pose(" << p.x << ", " << p.y << ", " << p.theta << ")";
}

/**
 * PathPoint: Pose tagged with steering and speed info
 */
struct PathPoint {
    Pose pose;
    double steer;
    double speed;

    PathPoint(const Pose& pose, double steer, double speed) : pose(pose), steer(steer), speed(speed) {}
    PathPoint() : pose(), steer(0), speed(0) {}
};

std::ostream& operator<<(std::ostream& out, const PathPoint& p) {
    return out << "PathPoint(" << p.pose << ", " << p.steer << ", " << p.speed << ")";
}

/**
 * PlannedPath: object representing a planned trajectory
 */
struct OptimizedTrajectory {
    std::vector<double> control;  // input to a path
    std::vector<PathPoint> path;  // results of path rollout
    std::vector<double> dists;    // results of collision checking
    double cost;                  // result of applying cost function
    bool has_collision;
};

/**
 * CenteredBox: container for forward, backward, left, and right offsets
 */
struct CenteredBox {
    double length_front;  // distance from origin to front edge
    double length_back;   // ditto for back edge, etc.
    double width_left;
    double width_right;
};

}  // namespace rr
