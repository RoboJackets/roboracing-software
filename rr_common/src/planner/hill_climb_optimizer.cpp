#include <rr_common/planning/hill_climb_optimizer.h>

#include <mutex>
#include <thread>

#include <parameter_assertions/assertions.h>

namespace rr {

HillClimbPlanner::HillClimbPlanner(const ros::NodeHandle& nh, rr::NearestPointCache distanceChecker,
                                   rr::BicycleModel bicycleModel)
      : distance_checker_(std::move(distanceChecker))
      , model_(std::move(bicycleModel))
      , rand_gen_(0)
      , normal_distribution_(0, 1) {
    assertions::getParam(nh, "n_segments", state_dim_, { assertions::greater<int>(0) });
    assertions::getParam(nh, "k_dist", k_dist_, { assertions::greater_eq(0.0) });
    assertions::getParam(nh, "k_speed", k_speed_, { assertions::greater_eq(0.0) });
    assertions::getParam(nh, "k_angle", k_angle_, { assertions::greater_eq(0.0) });
    assertions::getParam(nh, "k_steering", k_steering_, { assertions::greater_eq(0.0) });
    assertions::getParam(nh, "collision_penalty", collision_penalty_, { assertions::greater_eq(0.0) });
    assertions::getParam(nh, "max_steering", max_steering_, { assertions::greater(0.0) });
    assertions::getParam(nh, "num_workers", num_workers_, { assertions::greater(0) });
    assertions::getParam(nh, "num_restarts", num_restarts_, { assertions::greater(0) });
    assertions::getParam(nh, "neighbor_stddev", neighbor_stddev_, { assertions::greater(0.0) });
    assertions::getParam(nh, "local_optimum_tries", local_optimum_tries_, { assertions::greater(0) });
}

void HillClimbPlanner::FillObstacleCosts(OptimizedTrajectory& plan) const {
    plan.cost = 0;
    plan.has_collision = false;
    for (size_t i = 0; i < plan.path.size(); ++i) {
        auto dist = distance_checker_.GetCollisionDistance(plan.path[i].pose);
        if (dist > 0) {
            plan.cost += k_dist_ * std::exp(-dist);
            plan.cost -= k_speed_ * plan.path[i].speed;
            plan.cost += k_steering_ * plan.path[i].steer;
            plan.cost += k_angle_ * std::abs(plan.path[i].pose.theta);
        } else {
            plan.cost += collision_penalty_ * (plan.path.size() - i);
            plan.has_collision = true;
            break;
        }
    }
}

void HillClimbPlanner::JitterControls(std::vector<double>& ctrl, double stddev) {
    for (double& x : ctrl) {
        x = std::clamp(x + normal_distribution_(rand_gen_) * stddev, -max_steering_, max_steering_);
    }
}

std::vector<double> HillClimbPlanner::Optimize(const CostFunction& cost_fn, const std::vector<double>& init_controls) {
    auto descend_hill = [this, &cost_fn](std::vector<double> controls) {
        double best_cost = std::numeric_limits<double>::max();
        int stuck_counter = local_optimum_tries_;
        while (stuck_counter > 0) {
            const auto last_ctrl = controls;  // copy
            JitterControls(controls, neighbor_stddev_);
            auto cost = cost_fn(controls);

            if (cost >= best_cost) {
                --stuck_counter;
                controls = last_ctrl;
            } else {
                stuck_counter = local_optimum_tries_;
                best_cost = cost;
            }
        }
        return std::make_tuple(best_cost, std::move(controls));
    };

    std::vector<double> global_best_controls;
    double global_best_cost = std::numeric_limits<double>::max();
    int plan_count = 0;
    std::mutex plan_count_mutex, global_best_plan_mutex;

    auto worker = [&, this](int thread_idx) {
        std::vector<double> controls;
        double best_cost = global_best_cost;
        while (true) {
            {
                std::lock_guard lock(plan_count_mutex);
                if (plan_count >= num_restarts_) {
                    break;
                }
                plan_count++;
            }

            if (plan_count == 1 && !previous_best_plan_.control.empty()) {
                // for one start, init to previous best controls
                controls = previous_best_plan_.control;
            } else {
                // select a random starting configuration
                controls.resize(state_dim_);
                std::fill(controls.begin(), controls.end(), 0.0);
                JitterControls(controls, max_steering_ / 2);
            }

            descend_hill(plan);

            if (plan.cost < best_plan.cost) {
                best_plan = plan;
            }
        }

        std::lock_guard lock(global_best_plan_mutex);
        if (best_plan.cost < global_best_plan.cost) {
            global_best_plan = best_plan;
        }
    };

    std::vector<std::thread> threads;
    for (int th_id = 0; th_id < num_workers_; ++th_id) {
        threads.emplace_back(worker, th_id);
    }
    for (auto& t : threads) {
        t.join();
    }

    previous_best_plan_ = global_best_plan;
    return global_best_plan;
}

}  // namespace rr
