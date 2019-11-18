#pragma once

#include <algorithm>
#include <random>
#include <vector>

#include "planner_types.hpp"

namespace rr {

namespace detail {

// inline int dim(const std::vector<double>& v) {
//    return v.size();
//}
//
// inline int dim(const Eigen::MatrixXd& m) {
//    return m.rows();
//}

// inline int n_control_points(const Eigen::MatrixXd& m) {
//    return m.cols();
//}

}  // namespace detail

// template <class T1, class T2, class... Ts>
// inline void assert_controls_dimension(const T1& a, const T2& b, const Ts&... others) {
//    ROS_ASSERT(detail::dim(a) == detail::dim(b));
//    if constexpr (sizeof...(Ts) > 0) {
//        assert_controls_dimension(b, others...);
//    }
//}

template <int ctrl_dim>
inline Controls<ctrl_dim> controls_neighbor(const Controls<ctrl_dim>& ctrl, const Matrix<ctrl_dim, 2>& limits,
                                            const Vector<ctrl_dim>& stddevs) {
    static std::mt19937 rand_gen(1234567);  // constant value allows for repeatable testing if desired
    static std::normal_distribution<double> normal_pdf(0, 1);

    Controls<ctrl_dim> neighbor(ctrl_dim, ctrl.cols());
    for (long dim = 0; dim < ctrl.rows(); ++dim) {
        for (long i = 0; i < ctrl.cols(); ++i) {
            double raw = ctrl(dim, i) + normal_pdf(rand_gen) * stddevs(dim);
            neighbor(dim, i) = std::clamp(raw, limits(dim, 0), limits(dim, 1));
        }
    }
    return neighbor;
}

template <int ctrl_dim>
inline Controls<ctrl_dim> init_controls(int n_control_points, const Matrix<ctrl_dim, 2>& limits,
                                        const Vector<ctrl_dim>& stddevs) {
    Controls<ctrl_dim> ctrl(ctrl_dim, n_control_points);
    auto mid = (limits.col(1) + limits.col(0)) * 0.5;
    for (int dim = 0; dim < ctrl_dim; ++dim) {
        ctrl.row(dim).setConstant(mid(dim));
    }
    return controls_neighbor(ctrl, limits, stddevs);
}

template <int ctrl_dim>
inline Controls<ctrl_dim> init_controls(int n_control_points, const Matrix<ctrl_dim, 2>& limits) {
    static std::mt19937 rand_gen(1234567);  // constant value allows for repeatable testing if desired
    static std::uniform_real_distribution<double> uniform_01(0, 1);

    Controls<ctrl_dim> ctrl(ctrl_dim, n_control_points);
    for (long dim = 0; dim < ctrl.rows(); ++dim) {
        for (long i = 0; i < ctrl.cols(); ++i) {
            ctrl(dim, i) = limits(dim, 0) + uniform_01(rand_gen) * (limits(dim, 1) - limits(dim, 0));
        }
    }

    return ctrl;
}

}  // namespace rr
