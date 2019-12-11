#pragma once

#include <parameter_assertions/assertions.h>
#include <ros/ros.h>

namespace rr {

class LinearTrackingFilter {
  private:
    double val_;
    double target_;
    double val_min_;
    double val_max_;
    double rate_min_;
    double rate_max_;
    double last_update_;

  public:
    explicit LinearTrackingFilter(const ros::NodeHandle& nh) {
        assertions::getParam(nh, "init_val", val_);
        target_ = val_;
        assertions::getParam(nh, "val_max", val_max_);
        assertions::getParam(nh, "val_min", val_min_, { assertions::less(val_max_) });
        assertions::getParam(nh, "rate_min", rate_min_, { assertions::less<double>(0) });
        assertions::getParam(nh, "rate_max", rate_max_, { assertions::greater<double>(0) });
        last_update_ = 0;
    }

    LinearTrackingFilter(const LinearTrackingFilter& t) = default;

    inline double GetValue() const {
        return val_;
    }
    inline double GetLastUpdateTime() const {
        return last_update_;
    }
    inline double GetValMin() const {
        return val_min_;
    }
    inline double GetValMax() const {
        return val_max_;
    }
    inline double GetRateMin() const {
        return rate_min_;
    }
    inline double GetRateMax() const {
        return rate_max_;
    }

    inline void SetTarget(double x) {
        target_ = x;
    }

    inline void Update(double t) {
        if (last_update_ > 0) {
            double dt = t - last_update_;
            if (dt > 0) {
                val_ = std::clamp(target_, val_ + rate_min_ * dt, val_ + rate_max_ * dt);
                val_ = std::clamp(val_, val_min_, val_max_);
            } else if (dt < 0)  {
                ROS_WARN("[%s] linear tracking model found jump backwards in time %f %f",
                         ros::this_node::getName().c_str(), last_update_, t);
            }
        }

        last_update_ = t;
    }

    inline void Update(double setpoint, double t) {
        target_ = setpoint;
        Update(t);
    }

    inline void UpdateRawDT(double dt) {
        double l1 = val_ + rate_min_ * dt;
        double l2 = val_ + rate_max_ * dt;
        val_ = std::clamp(target_, std::min(l1, l2), std::max(l1, l2));
        val_ = std::clamp(val_, val_min_, val_max_);
        last_update_ += dt;
    }

    inline void Reset(double x, double t) {
        val_ = x;
        last_update_ = t;
    }
};

}  // namespace rr
