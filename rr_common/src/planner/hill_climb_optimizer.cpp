#include <rr_common/planning/hill_climb_optimizer.h>

#include <mutex>
#include <thread>

#include <parameter_assertions/assertions.h>

#include <rr_common/planning/planning_utils.h>

namespace rr {

template class HillClimbOptimizer<1>;
template class HillClimbOptimizer<2>;

template <int ctrl_dim>
HillClimbOptimizer<ctrl_dim>::HillClimbOptimizer(const ros::NodeHandle& nh) : previous_best_controls_set_(false) {
    assertions::getParam(nh, "num_workers", num_workers_, { assertions::greater(0) });
    assertions::getParam(nh, "num_restarts", num_restarts_, { assertions::greater(0) });
    assertions::getParam(nh, "local_optimum_tries", local_optimum_tries_, { assertions::greater(0) });

    std::vector<double> stddev;
    assertions::getParam(nh, "stddev", stddev, { assertions::size<std::vector<double>>(ctrl_dim) });

    for (size_t i = 0; i < ctrl_dim; ++i) {
        ROS_ASSERT(stddev[i] > 0);
        neighbor_stddev_(i) = stddev[i];
    }
}

// void HillClimbOptimizer::FillObstacleCosts(OptimizedTrajectory& plan) const {
//    plan.cost = 0;
//    plan.has_collision = false;
//    for (size_t i = 0; i < plan.path.size(); ++i) {
//        auto dist = distance_checker_.GetCollisionDistance(plan.path[i].pose);
//        if (dist > 0) {
//            plan.cost += k_dist_ * std::exp(-dist);
//            plan.cost -= k_speed_ * plan.path[i].speed;
//            plan.cost += k_steering_ * plan.path[i].steer;
//            plan.cost += k_angle_ * std::abs(plan.path[i].pose.theta);
//        } else {
//            plan.cost += collision_penalty_ * (plan.path.size() - i);
//            plan.has_collision = true;
//            break;
//        }
//    }
//}

template <int ctrl_dim>
Controls<ctrl_dim> HillClimbOptimizer<ctrl_dim>::Optimize(const CostFunction<ctrl_dim>& cost_fn,
                                                          const Controls<ctrl_dim>& init_controls,
                                                          const Matrix<ctrl_dim, 2>& ctrl_limits) {
    auto descend_hill = [&](Controls<ctrl_dim> controls) {
        double best_cost = std::numeric_limits<double>::max();
        int stuck_counter = local_optimum_tries_;
        while (stuck_counter > 0) {
            const auto new_controls = controls_neighbor(controls, ctrl_limits, neighbor_stddev_);
            auto cost = cost_fn(controls);

            if (cost >= best_cost) {
                --stuck_counter;
            } else {
                controls = std::move(new_controls);
                best_cost = cost;
                stuck_counter = local_optimum_tries_;
            }
        }
        return std::make_tuple(best_cost, std::move(controls));
    };

    Controls<ctrl_dim> global_best_controls;
    double global_best_cost = std::numeric_limits<double>::max();
    int plan_count = 0;
    std::mutex plan_count_mutex, global_best_plan_mutex;

    auto worker = [&, this](int thread_idx) {
        Controls<ctrl_dim> controls;
        double best_cost = global_best_cost;
        while (true) {
            {
                std::lock_guard lock(plan_count_mutex);
                if (plan_count >= num_restarts_) {
                    break;
                }
                plan_count++;
            }

            if (plan_count == 1 && previous_best_controls_set_) {
                // for one start, init to previous best controls
                controls = std::move(previous_best_controls_);
            } else {
                // select a random starting configuration
                Vector<ctrl_dim> half_range = (ctrl_limits.col(1) - ctrl_limits.col(0)) * 0.5;
                controls = rr::init_controls(init_controls.cols(), ctrl_limits, half_range);
            }

            auto [cost, controls_opt] = descend_hill(controls);

            if (cost < best_cost) {
                controls = std::move(controls_opt);
                best_cost = cost;
            }
        }

        std::lock_guard lock(global_best_plan_mutex);
        if (best_cost < global_best_cost) {
            global_best_controls = std::move(controls);
            global_best_cost = best_cost;
        }
    };

    std::vector<std::thread> threads;
    for (int th_id = 0; th_id < num_workers_; ++th_id) {
        threads.emplace_back(worker, th_id);
    }
    for (auto& t : threads) {
        t.join();
    }

    previous_best_controls_ = std::move(global_best_controls);
    previous_best_controls_set_ = true;
    return previous_best_controls_;
}

}  // namespace rr
