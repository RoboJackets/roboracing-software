#pragma once

#include <algorithm>
#include <random>
#include <vector>

namespace rr {

template <class Generator>
std::vector<double> controls_neighbor(const std::vector<double>& ctrl, Generator& rand_gen, double stddev,
                                      double min_val, double max_val) {
    std::vector<double> neighbor(ctrl.size());
    std::normal_distribution<double> dis(0, stddev);
    for (size_t i = 0; i < ctrl.size(); ++i) {
        neighbor[i] = std::clamp(ctrl + normal_distribution_(rand_gen) * stddev, min_val, max_val);
    }
    return neighbor;
}

}  // namespace rr
