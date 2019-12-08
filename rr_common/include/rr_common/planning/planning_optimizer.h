#pragma once

#include <vector>

#include "planner_types.hpp"

namespace rr {

template <int ctrl_dim>
class PlanningOptimizer {
  public:
    virtual Controls<ctrl_dim> Optimize(const CostFunction<ctrl_dim>& cost_fn, const Controls<ctrl_dim>& init_controls,
                                        const Matrix<ctrl_dim, 2>& ctrl_limits) = 0;
};

}  // namespace rr
