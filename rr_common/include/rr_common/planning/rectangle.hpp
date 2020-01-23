#pragma once

#include <parameter_assertions/assertions.h>
#include <ros/node_handle.h>

namespace rr {

/**
 * CenteredBox: container for forward, backward, left, and right offsets
 */
struct Rectangle {
    double min_x;  // x lower boundary relative to origin
    double max_x;  // x upper boundary relative to origin
    double min_y;  // y lower boundary relative to origin
    double max_y;  // y upper boundary relative to origin
    Pose origin;

    Rectangle(double x0, double x1, double y0, double y1, const Pose& origin_pose)
          : min_x(x0), max_x(x1), min_y(y0), max_y(y1), origin(origin_pose) {}

    Rectangle(double x0, double x1, double y0, double y1) : Rectangle(x0, x1, y0, y1, Pose(0, 0, 0)) {}

    Rectangle() = default;
    Rectangle(const Rectangle& other) = default;

    explicit Rectangle(const ros::NodeHandle& nh) {
        assertions::getParam(nh, "min_x", min_x);
        assertions::getParam(nh, "max_x", max_x);
        assertions::getParam(nh, "min_y", min_y);
        assertions::getParam(nh, "max_y", max_y);
        assertions::param(nh, "origin_x", origin.x, 0.0);
        assertions::param(nh, "origin_y", origin.y, 0.0);
        assertions::param(nh, "origin_theta", origin.theta, 0.0);
    }

    [[nodiscard]] inline bool PointInside(double x, double y) const {
        double cos_th = std::cos(origin.theta);
        double sin_th = std::sin(origin.theta);
        double dx = x - origin.x;
        double dy = y - origin.y;
        double local_x = dx * cos_th - dy * sin_th;
        double local_y = dx * sin_th + dy * cos_th;
        return local_x >= min_x && local_x < max_x && local_y >= min_y && local_y < max_y;
    }

    /**
     * Calculates radius of the inscribed circle and maximum forward shift the circle can be placed from the rect's
     * origin Assumes symmetry on the y-axis (origin-y = 0)
     */
    inline std::pair<double, double> getForwardInscribedCircle() {
        double radius = fmin(max_x - min_x, max_y - min_y) / 2.0;
        double shift = max_x - radius;
        return std::make_pair(radius, shift);
    }
};

}  // namespace rr
