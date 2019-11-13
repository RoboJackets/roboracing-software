#include <mutex>
#include <thread>

#include <planner/hill_climb_planner.h>

#include <parameter_assertions/assertions.h>

namespace rr {

template <class T>
assertions::Assertion<T> container_not_empty{ [](const T& v) { return !v.empty(); }, "container not empty" };

HillClimbPlanner::HillClimbPlanner(const ros::NodeHandle& nh, const rr::DistanceChecker& distanceChecker,
                                   const rr::BicycleModel& bicycleModel)
      : distance_checker_(distanceChecker), model_(bicycleModel), rand_gen_(), normal_distribution_(0, 1) {
    std::vector<int> segment_sections;
    assertions::getParam(nh, "segment_sections", segment_sections, { container_not_empty<std::vector<int>> });
    state_dim_ = segment_sections.size();

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

void HillClimbPlanner::FillObstacleCosts(PlannedPath& plan) const {
    if (plan.dists.size() != plan.path.size()) {
        plan.dists.resize(plan.path.size());
    }

    plan.cost = 0;
    plan.has_collision = false;
    for (size_t i = 0; i < plan.path.size(); ++i) {
        auto opt_dist = distance_checker_.GetCollisionDistance(plan.path[i].pose);
        if (opt_dist) {
            plan.cost += k_dist_ * std::exp(-std::max(0.01, opt_dist.value()));
            plan.cost -= k_speed_ * plan.path[i].speed;
            plan.cost += k_steering_ * plan.path[i].steer;
            plan.cost += std::abs(plan.path[i].pose.theta) * k_angle_;
            plan.dists[i] = opt_dist.value();
        } else {
            plan.cost += collision_penalty_ * (plan.path.size() - i);
            std::fill(plan.dists.begin() + i, plan.dists.end(), 0);
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

PlannedPath HillClimbPlanner::Plan(const rr::PCLMap& map) {
    distance_checker_.SetMap(map);

    // precondition: plan.control is initialized
    auto descend_hill = [this, &map](PlannedPath& plan) {
        plan.has_collision = false;
        double best_cost = std::numeric_limits<double>::max();
        int stuck_counter = local_optimum_tries_;
        int step_counter = 0;
        while (stuck_counter > 0) {
            ++step_counter;
            const auto last_ctrl = plan.control;  // copy
            JitterControls(plan.control, neighbor_stddev_);
            model_.RollOutPath(plan.control, plan.path);
            FillObstacleCosts(plan);

            if (plan.cost >= best_cost) {
                --stuck_counter;
                plan.control = last_ctrl;
            } else {
                stuck_counter = local_optimum_tries_;
                best_cost = plan.cost;
            }
        }

        // ROS_INFO_STREAM("descent took " << step_counter << " optimization steps");
    };

    std::vector<PlannedPath> best_plans(num_workers_);
    for (PlannedPath& plan : best_plans) {
        plan.cost = std::numeric_limits<double>::max();
    }
    int plan_count = 0;
    std::mutex plan_count_mutex;

    auto worker = [&, this](int thread_idx) {
        while (true) {
            {
                std::lock_guard lock(plan_count_mutex);
                if (plan_count >= num_restarts_) {
                    break;
                }
                plan_count++;
            }

            PlannedPath plan;
            if (plan_count == 1 && !previous_best_plan_.control.empty()) {
                // for one start, init to previous best controls
                plan.control = previous_best_plan_.control;
            } else {
                // select a random starting configuration
                plan.control.resize(state_dim_);
                std::fill(plan.control.begin(), plan.control.end(), 0.0);
                JitterControls(plan.control, max_steering_ / 2);
            }

            descend_hill(plan);

            if (plan.cost < best_plans[thread_idx].cost) {
                best_plans[thread_idx] = plan;
            }
        }
    };

    std::vector<std::thread> threads;
    for (int th_id = 0; th_id < num_workers_; ++th_id) {
        threads.emplace_back(worker, th_id);
    }
    for (auto& t : threads) {
        t.join();
    }

    int best_worker = 0;
    for (int i = 1; i < num_workers_; ++i) {
        if (best_plans[i].cost < best_plans[best_worker].cost) {
            best_worker = i;
        }
    }

    model_.UpdateSteeringAngle(best_plans[best_worker].control[0]);
    previous_best_plan_ = best_plans[best_worker];
    return best_plans[best_worker];
}

}  // namespace rr
