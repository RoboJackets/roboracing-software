#pragma once

#include <rclcpp/rclcpp.hpp>

#include <random>
#include <vector>

#include "planning_optimizer.h"

namespace rr {

template <int ctrl_dim>
class AnnealingOptimizer : public PlanningOptimizer<ctrl_dim> {
  public:
    struct Params {
        int annealing_steps;            // number of timesteps to run simulated annealing
        double temperature_end;         // temperature at last iteration
        Vector<ctrl_dim> stddev_start;  // neighbor standard deviation when temperature=1
        double acceptance_scale;        // strictness for accepting bad paths
    };

    explicit AnnealingOptimizer(const ros::NodeHandle& nh);

    ~AnnealingOptimizer() = default;

    Controls<ctrl_dim> Optimize(const CostFunction<ctrl_dim>& cost_fn, const Controls<ctrl_dim>& init_controls,
                                const Matrix<ctrl_dim, 2>& ctrl_limits) override;

  private:
    double GetTemperature(unsigned int t);

    Params params_;
    std::uniform_real_distribution<double> uniform_01_;
    std::mt19937 rand_gen_;
};

}  // namespace rr
