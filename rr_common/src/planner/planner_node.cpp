#include <geometry_msgs/Pose.h>
#include <parameter_assertions/assertions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rr_common/planning/annealing_optimizer.h>
#include <rr_common/planning/bicycle_model.h>
#include <rr_common/planning/distance_map.h>
#include <rr_common/planning/effector_tracker.h>
#include <rr_common/planning/global_path.h>
#include <rr_common/planning/hill_climb_optimizer.h>
#include <rr_common/planning/inflation_map.h>
#include <rr_common/planning/map_cost_interface.h>
#include <rr_common/planning/nearest_point_cache.h>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>
#include <visualization_msgs/Marker.h>

#include <rr_common/linear_tracking_filter.hpp>

constexpr int ctrl_dim = 2;

std::unique_ptr<rr::PlanningOptimizer<ctrl_dim>> g_planner;
std::unique_ptr<rr::MapCostInterface> g_map_cost_interface;
std::unique_ptr<rr::BicycleModel> g_vehicle_model;
std::unique_ptr<rr::EffectorTracker> g_effector_tracker;
std::unique_ptr<rr::GlobalPath> g_global_path_cost;

std::shared_ptr<rr::LinearTrackingFilter> g_speed_model;
std::shared_ptr<rr::LinearTrackingFilter> g_steer_model;

double k_map_cost_, k_speed_, k_steering_, k_angle_, k_global_path_cost_, collision_penalty_;
rr::Controls<ctrl_dim> g_last_controls;

ros::Publisher speed_pub;
ros::Publisher steer_pub;
ros::Publisher viz_pub;

rr_msgs::speedPtr speed_message;
rr_msgs::steeringPtr steer_message;

enum reverse_state_t { OK, CAUTION, REVERSE };

ros::Duration caution_duration;
ros::Duration reverse_duration;
double reverse_speed;
ros::Time caution_start_time;
ros::Time reverse_start_time;
reverse_state_t reverse_state;

double steering_gain;
double viz_path_scale;

double total_planning_time;
size_t total_plans;

void update_messages(double speed, double angle) {
    auto now = ros::Time::now();

    speed_message->speed = std::min(speed, 1.0);
    speed_message->header.stamp = now;

    steer_message->angle = angle;
    steer_message->header.stamp = now;
}

void publish_path_viz(const std::vector<rr::PathPoint>& path_rollout) {
    visualization_msgs::Marker line_strip;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = viz_path_scale;
    line_strip.id = 374;  // Unique ID (from issue number)
    line_strip.pose.orientation.x = 0;
    line_strip.pose.orientation.y = 0;
    line_strip.pose.orientation.z = 0;
    line_strip.pose.orientation.w = 1;
    for (const rr::PathPoint& path_point : path_rollout) {
        geometry_msgs::Point p;
        p.x = path_point.pose.x;
        p.y = path_point.pose.y;
        std_msgs::ColorRGBA c;
        if (OK == reverse_state) {
            c.r = 0;
            c.g = std::abs(path_point.speed) / g_speed_model->GetValMax();
            c.b = 0;
        } else if (CAUTION == reverse_state) {
            c.r = 1.0;
            c.g = 1.0;
            c.b = 0;
        } else if (REVERSE == reverse_state) {
            c.r = 1.0;
            c.g = 0;
            c.b = 0;
        }
        c.a = 1.0;
        line_strip.points.push_back(p);
        line_strip.colors.push_back(c);
    }
    line_strip.header.frame_id = "base_footprint";
    viz_pub.publish(line_strip);
}

void generatePath() {
    auto max_speed = g_speed_model->GetValMax();

    rr::CostFunction<ctrl_dim> cost_fn = [&](const rr::Controls<ctrl_dim>& controls) -> double {
        rr::TrajectoryRollout rollout;
        g_vehicle_model->RollOutPath(controls, rollout);
        const auto& path = rollout.path;

        std::vector<double> map_costs = g_map_cost_interface->DistanceCost(path);
        double global_path_costs = g_global_path_cost->CalculateCost(path);
        double cost = 0;
        double inflator = 1;
        double gamma = 1.01;
        for (size_t i = 0; i < rollout.path.size(); ++i) {
            cost *= gamma;
            inflator *= gamma;
            if (map_costs[i] >= 0) {
                cost += k_map_cost_ * map_costs[i];
                cost += k_speed_ * std::pow(max_speed - path[i].speed, 2);
                cost += k_steering_ * std::abs(path[i].steer);
                cost += k_angle_ * std::abs(path[i].pose.theta);
            } else {
                cost += collision_penalty_ * (path.size() - i);
                break;
            }
        }
        cost += k_global_path_cost_ * global_path_costs;
        return cost / inflator;
    };

    rr::Matrix<ctrl_dim, 2> ctrl_limits;
    ctrl_limits.row(0) << g_steer_model->GetValMin(), g_steer_model->GetValMax();
    ctrl_limits.row(1) << g_speed_model->GetValMin(), g_speed_model->GetValMax();

    rr::TrajectoryPlan plan;
    rr::Controls<ctrl_dim> controls = g_planner->Optimize(cost_fn, g_last_controls, ctrl_limits);
    plan.cost = cost_fn(controls);
    g_vehicle_model->RollOutPath(controls, plan.rollout);

    std::vector<double> map_costs = g_map_cost_interface->DistanceCost(plan.rollout.path);
    auto negative_it = std::find_if(map_costs.begin(), map_costs.end(), [](double x) { return x < 0; });
    plan.has_collision = (negative_it != map_costs.end());

    g_global_path_cost->visualize_global_segment(plan.rollout.path);

    g_last_controls = controls;

    ROS_INFO_STREAM("Best path cost is " << plan.cost << ", collision = " << plan.has_collision);

    // update impasse state machine
    auto now = ros::Time::now();

    if (OK == reverse_state) {
        if (plan.has_collision) {
            if (caution_duration.toSec() <= 0) {
                reverse_state = REVERSE;
                reverse_start_time = now;
            } else {
                reverse_state = CAUTION;
                caution_start_time = now;
            }
        }
    } else if (CAUTION == reverse_state) {
        if (!plan.has_collision) {
            reverse_state = OK;
        } else if (now - caution_start_time > caution_duration) {
            reverse_state = REVERSE;
            reverse_start_time = now;
        }
    } else if (REVERSE == reverse_state) {
        if (now - reverse_start_time > reverse_duration) {
            reverse_state = CAUTION;
            caution_start_time = now;
        }
    } else {
        ROS_ERROR_STREAM("Planner encountered unknown reverse state");
    }

    if (REVERSE == reverse_state) {
        update_messages(reverse_speed, 0);
        ROS_WARN_STREAM("Planner reversing");
    } else if (plan.has_collision) {
        ROS_WARN_STREAM("Planner: no path found but not reversing; reusing previous message");
    } else {
        g_speed_model->Update(plan.rollout.apply_speed, now.toSec());
        update_messages(g_speed_model->GetValue(), plan.rollout.apply_steering * steering_gain);
    }

    speed_pub.publish(speed_message);
    steer_pub.publish(steer_message);

    if (viz_pub.getNumSubscribers() > 0) {
        publish_path_viz(plan.rollout.path);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    assertions::getParam(nhp, "k_map_cost", k_map_cost_);
    assertions::getParam(nhp, "k_speed", k_speed_);
    assertions::getParam(nhp, "k_steering", k_steering_);
    assertions::getParam(nhp, "k_angle", k_angle_);
    assertions::getParam(nhp, "k_global_path_cost", k_global_path_cost_);
    assertions::getParam(nhp, "collision_penalty", collision_penalty_);

    std::string map_type;
    assertions::getParam(nhp, "map_type", map_type);
    if (map_type == "obstacle_points") {
        g_map_cost_interface = std::make_unique<rr::NearestPointCache>(ros::NodeHandle(nhp, "obstacle_points_map"));
    } else if (map_type == "inflation_map") {
        g_map_cost_interface = std::make_unique<rr::InflationMap>(ros::NodeHandle(nhp, "inflation_map"));
    } else if (map_type == "distance_map") {
        g_map_cost_interface = std::make_unique<rr::DistanceMap>(ros::NodeHandle(nhp, "distance_map"));
    } else {
        ROS_ERROR_STREAM("[Planner] Error: unknown map type \"" << map_type << "\"");
        ros::shutdown();
    }

    g_steer_model = std::make_shared<rr::LinearTrackingFilter>(ros::NodeHandle(nhp, "steering_filter"));
    g_speed_model = std::make_shared<rr::LinearTrackingFilter>(ros::NodeHandle(nhp, "speed_filter"));
    g_vehicle_model =
          std::make_unique<rr::BicycleModel>(ros::NodeHandle(nhp, "bicycle_model"), g_steer_model, g_speed_model);

    std::string planner_type;
    assertions::getParam(nhp, "planner_type", planner_type);

    if (planner_type == "annealing") {
        g_planner = std::make_unique<rr::AnnealingOptimizer<ctrl_dim>>(ros::NodeHandle(nhp, "annealing_optimizer"));
    } else if (planner_type == "hill_climbing") {
        g_planner = std::make_unique<rr::HillClimbOptimizer<ctrl_dim>>(ros::NodeHandle(nhp, "hill_climb_optimizer"));
    } else {
        ROS_ERROR_STREAM("[Planner] Error: unknown planner type \"" << planner_type << "\"");
        ros::shutdown();
    }

    int n_control_points = 0;
    assertions::getParam(nhp, "n_segments", n_control_points);
    g_last_controls = rr::Controls<ctrl_dim>(ctrl_dim, n_control_points);
    g_last_controls.setZero();

    caution_duration = ros::Duration(assertions::param(nhp, "impasse_caution_duration", 0.0));
    reverse_duration = ros::Duration(assertions::param(nhp, "impasse_reverse_duration", 0.0));
    reverse_speed = assertions::param(nhp, "impasse_reverse_speed", -0.8);
    caution_start_time = ros::Time(0);
    reverse_start_time = ros::Time(0);
    reverse_state = OK;

    g_global_path_cost = std::make_unique<rr::GlobalPath>(ros::NodeHandle(nhp, "global_path_cost"));

    steering_gain = assertions::param(nhp, "steering_gain", 1.0);
    assertions::getParam(nhp, "viz_path_scale", viz_path_scale);

    speed_pub = nh.advertise<rr_msgs::speed>("plan/speed", 1);
    steer_pub = nh.advertise<rr_msgs::steering>("plan/steering", 1);
    viz_pub = nh.advertise<visualization_msgs::Marker>("plan/path", 1);

    speed_message.reset(new rr_msgs::speed);
    steer_message.reset(new rr_msgs::steering);
    update_messages(0, 0);
    g_effector_tracker =
          std::make_unique<rr::EffectorTracker>(ros::NodeHandle(nhp, "effector_tracker"), speed_message, steer_message);

    total_planning_time = 0;
    total_plans = 0;

    g_steer_model->Reset(0, ros::Time::now().toSec());
    g_speed_model->Reset(0, ros::Time::now().toSec());

    g_map_cost_interface->SetMapStale();

    ROS_INFO("planner initialized");

    ros::Rate rate(30);
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();

        g_steer_model->Update(g_effector_tracker->getAngle(), ros::Time::now().toSec());
        g_speed_model->Update(g_effector_tracker->getSpeed(), ros::Time::now().toSec());

        if (g_map_cost_interface->IsMapUpdated()) {
            auto start = ros::WallTime::now();

            g_global_path_cost->PreProcess();
            generatePath();
            g_map_cost_interface->SetMapStale();

            double seconds = (ros::WallTime::now() - start).toSec();
            total_planning_time += seconds;
            total_plans++;
            double sec_avg = total_planning_time / total_plans;
            ROS_INFO("PlanningOptimizer took %0.1fms, average %0.2fms", seconds * 1000, sec_avg * 1000);
        }
    }

    return 0;
}