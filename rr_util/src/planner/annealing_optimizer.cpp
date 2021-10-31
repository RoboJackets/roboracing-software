#include <parameter_assertions/assertions.h>
#include <rr_util/planning/annealing_optimizer.h>
#include <rr_util/planning/planning_utils.h>

namespace rr {

template class AnnealingOptimizer<1>;
template class AnnealingOptimizer<2>;

template <int ctrl_dim>
AnnealingOptimizer<ctrl_dim>::AnnealingOptimizer(const ros::NodeHandle& nh)
      : params_(), uniform_01_(0, 1), rand_gen_(42) {
    assertions::getParam(nh, "annealing_steps", params_.annealing_steps, { assertions::greater(0) });
    assertions::getParam(nh, "acceptance_scale", params_.acceptance_scale, { assertions::greater(0.0) });
    assertions::getParam(nh, "temperature_end", params_.temperature_end, { assertions::greater(0.0) });

    std::vector<double> stddev_start;
    assertions::getParam(nh, "stddevs_start", stddev_start, { assertions::size<std::vector<double>>(ctrl_dim) });

    for (size_t i = 0; i < ctrl_dim; ++i) {
        ROS_ASSERT(stddev_start[i] > 0);
        params_.stddev_start(i) = stddev_start[i];
    }
}

template <int ctrl_dim>
double AnnealingOptimizer<ctrl_dim>::GetTemperature(unsigned int t) {
    return std::exp(t * std::log(params_.temperature_end) / params_.annealing_steps);
}

template <int ctrl_dim>
Controls<ctrl_dim> AnnealingOptimizer<ctrl_dim>::Optimize(const CostFunction<ctrl_dim>& cost_fn,
                                                          const Controls<ctrl_dim>& init_controls,
                                                          const Matrix<ctrl_dim, 2>& ctrl_limits) {
    auto controls_state = init_controls;
    auto controls_best = init_controls;
    double cost_state = cost_fn(init_controls);
    double cost_best = cost_state;

    for (int t = 0; t < params_.annealing_steps; t++) {
        double temperature = GetTemperature(t);
        Vector<ctrl_dim> stddevs = params_.stddev_start / temperature;
        auto controls_new = controls_neighbor(controls_state, ctrl_limits, stddevs);
        double cost_new = cost_fn(controls_new);

        double dcost = cost_new - cost_state;
        if (dcost < 0) {
            controls_state = controls_new;
            cost_state = cost_new;
        } else {
            double p_accept = std::exp(-params_.acceptance_scale * dcost / temperature);
            if (uniform_01_(rand_gen_) < p_accept) {
                controls_state = controls_new;
                cost_state = cost_new;
            }
        }

        if (cost_new < cost_best) {
            controls_best = controls_new;
            cost_best = cost_new;
        }
    }

    return controls_best;
}

}  // namespace rr
