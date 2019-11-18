#pragma once

#include <optional>

#include <ros/node_handle.h>

#include "planning_optimizer.h"

namespace rr {

template <int ctrl_dim>
class HillClimbOptimizer : public PlanningOptimizer<ctrl_dim> {
  public:
    explicit HillClimbOptimizer(const ros::NodeHandle& nh);

    Controls<ctrl_dim> Optimize(const CostFunction<ctrl_dim>& cost_fn, const Controls<ctrl_dim>& init_controls,
                                const Matrix<ctrl_dim, 2>& ctrl_limits) override;

  private:
    int num_workers_;                   // number of threads to run in parallel
    int num_restarts_;                  // total number of hill descents to do
    Vector<ctrl_dim> neighbor_stddev_;  // standard deviation of noise added in neighbor function
    int local_optimum_tries_;           // we are at a local optimum if we try this many times with no improvement
};

}  // namespace rr
