#pragma once

#include <functional>
#include <vector>

namespace rr {

class PlanningOptimizer {
  public:
    using CostFunction = std::function<double(const std::vector<double>&)>;

    virtual std::vector<double> Optimize(const CostFunction& cost_fn, const std::vector<double>& init_controls) = 0;
};

}  // namespace rr
