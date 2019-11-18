#include <rr_common/planning/annealing_optimizer.h>
#include <rr_common/planning/planning_utils.h>

#include <parameter_assertions/assertions.h>

namespace rr {

AnnealingPlanner::AnnealingPlanner(const ros::NodeHandle& nh, const NearestPointCache& c, const BicycleModel& m)
      : rand_gen_(42) {
    using assertions::getParam;

    getParam(nh, "n_path_segments", params_.n_path_segments, { assertions::greater(0) });
    getParam(nh, "annealing_steps", params_.annealing_steps, { assertions::greater(0) });
    getParam(nh, "k_dist", params_.k_dist, { assertions::greater_eq(0.0) });
    getParam(nh, "k_speed", params_.k_speed, { assertions::greater_eq(0.0) });
    getParam(nh, "k_final_pose", params_.k_final_pose, { assertions::greater_eq(0.0) });
    getParam(nh, "k_angle", params_.k_angle, { assertions::greater_eq(0.0) });
    getParam(nh, "collision_penalty", params_.collision_penalty, { assertions::greater_eq(0.0) });
    getParam(nh, "max_steering", params_.max_steering, { assertions::greater(0.0) });
    getParam(nh, "acceptance_scale", params_.acceptance_scale, { assertions::greater(0.0) });
    getParam(nh, "temperature_start", params_.temperature_start, { assertions::greater(0.0) });
    getParam(nh, "temperature_end", params_.temperature_end,
             { assertions::greater(0.0), assertions::less(params_.temperature_start) });
}

double AnnealingPlanner::GetTemperature(unsigned int t) {
    static const double K = std::log(params_.temperature_end / params_.temperature_start) / params_.annealing_steps;

    return params_.temperature_start * std::exp(K * t);
}

std::vector<double> AnnealingPlanner::Optimize(const rr::PlanningOptimizer::CostFunction& cost_fn,
                                               const std::vector<double>& init_controls) {
    std::vector<double> controls_state = init_controls;
    std::vector<double> controls_best = init_controls;
    double cost_state = cost_fn(init_controls);
    double cost_best = cost_state;

    for (int t = 0; t < params_.annealing_steps; t++) {
        auto controls_new = controls_neighbor(controls_state, rand_gen_, GetTemperature(t), -params_.max_steering,
                                              params_.max_steering);

        double cost_new = cost_fn(controls_new);
        double dcost = cost_new - cost_state;

        if (dcost < 0) {
            controls_state = controls_new;
            cost_state = cost_new;
        } else {
            double p_accept = std::exp(-params_.acceptance_scale * dcost / GetTemperature(t));
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
