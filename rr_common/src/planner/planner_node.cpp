#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <parameter_assertions/assertions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>

#include <rr_common/annealing_planner.h>
#include <rr_common/hill_climb_planner.h>
#include <rr_common/random_sample_planner.h>
#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>

std::unique_ptr<rr::PlanningOptimizer> planner;
std::unique_ptr<rr::NearestPointCache> distance_checker;
std::unique_ptr<rr::LinearTrackingFilter> speed_filter;

ros::Publisher speed_pub;
ros::Publisher steer_pub;
ros::Publisher path_pub;

rr_msgs::speedPtr speed_message;
rr_msgs::steeringPtr steer_message;

sensor_msgs::PointCloud2ConstPtr last_map_msg;
bool is_new_msg;

enum reverse_state_t { OK, CAUTION, REVERSE };

ros::Duration caution_duration;
ros::Duration reverse_duration;
ros::Time caution_start_time;
ros::Time reverse_start_time;
reverse_state_t reverse_state;

double steering_gain;

double total_planning_time;
size_t total_plans;

void mapCallback(const sensor_msgs::PointCloud2ConstPtr& map) {
    last_map_msg = map;
    is_new_msg = true;
}

void update_messages(double speed, double angle) {
    auto now = ros::Time::now();

    speed_message->speed = speed;
    speed_message->header.stamp = now;

    steer_message->angle = angle;
    steer_message->header.stamp = now;
}

void processMap(const sensor_msgs::PointCloud2ConstPtr& map) {
    pcl::PCLPointCloud2 pcl_pc2;
    rr::PCLMap cloud;

    pcl_conversions::toPCL(*map, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);

    for (auto point_it = cloud.begin(); point_it != cloud.end();) {
        if (distance_checker->GetCollision(*point_it)) {
            point_it = cloud.erase(point_it);
        } else {
            point_it++;
        }
    }

    if (cloud.empty()) {
        ROS_WARN("environment map pointcloud is empty");
    }

    rr::OptimizedTrajectory plan = planner->Optimize(cloud);

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
        ROS_ERROR_STREAM("PlanningOptimizer encountered unknown reverse state");
    }

    if (REVERSE == reverse_state) {
        update_messages(-0.8, 0);
        ROS_WARN_STREAM("PlanningOptimizer reversing");
    } else if (plan.has_collision) {
        ROS_WARN_STREAM("PlanningOptimizer: no path found but not reversing; reusing previous message");
    } else {
        speed_filter->Update(plan.path[0].speed, now.toSec());
        update_messages(speed_filter->GetValue(), plan.path[0].steer * steering_gain);
    }

    speed_pub.publish(speed_message);
    steer_pub.publish(steer_message);

    if (path_pub.getNumSubscribers() > 0) {
        nav_msgs::Path pathMsg;

        for (auto path_point : plan.path) {
            geometry_msgs::PoseStamped ps;
            ps.pose.position.x = path_point.pose.x;
            ps.pose.position.y = path_point.pose.y;
            pathMsg.poses.push_back(ps);
        }

        pathMsg.header.frame_id = "base_footprint";
        path_pub.publish(pathMsg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    rr::CenteredBox hitbox;
    ros::NodeHandle nh_hitbox(nhp, "collision_hitbox");
    assertions::getParam(nh_hitbox, "front", hitbox.length_front);
    assertions::getParam(nh_hitbox, "back", hitbox.length_back);
    assertions::getParam(nh_hitbox, "side", hitbox.width_left);
    hitbox.width_right = hitbox.width_left;

    rr::CenteredBox map_dimensions;
    map_dimensions.length_front = 15;
    map_dimensions.length_back = 5;
    map_dimensions.width_right = map_dimensions.width_left = 8;

    distance_checker = std::make_unique<rr::NearestPointCache>(hitbox, map_dimensions);

    auto steering_model = std::make_shared<rr::LinearTrackingFilter>(ros::NodeHandle(nhp, "steering_filter"));
    rr::BicycleModel vehicle_model(ros::NodeHandle(nhp, "bicycle_model"), steering_model);

    speed_filter = std::make_unique<rr::LinearTrackingFilter>(ros::NodeHandle(nhp, "speed_filter"));

    std::string obstacle_cloud_topic;
    assertions::getParam(nhp, "input_cloud_topic", obstacle_cloud_topic);
    std::string planner_type;
    assertions::getParam(nhp, "planner_type", planner_type);

    if (planner_type == "random_sample") {
        planner = std::make_unique<rr::RandomSamplePlanner>(nhp, *distance_checker, vehicle_model);
    } else if (planner_type == "annealing") {
        planner = std::make_unique<rr::AnnealingPlanner>(nhp, *distance_checker, vehicle_model);
    } else if (planner_type == "hill_climbing") {
        planner = std::make_unique<rr::HillClimbPlanner>(nhp, *distance_checker, vehicle_model);
    } else {
        ROS_ERROR_STREAM("[PlanningOptimizer] Error: unknown planner type \"" << planner_type << "\"");
        ros::shutdown();
    }

    caution_duration = ros::Duration(assertions::param(nhp, "impasse_caution_duration", 0.0));
    reverse_duration = ros::Duration(assertions::param(nhp, "impasse_reverse_duration", 0.0));
    caution_start_time = ros::Time(0);
    reverse_start_time = ros::Time(0);
    reverse_state = OK;

    steering_gain = assertions::param(nhp, "steering_gain", 1.0);

    auto map_sub = nh.subscribe(obstacle_cloud_topic, 1, mapCallback);
    speed_pub = nh.advertise<rr_msgs::speed>("plan/speed", 1);
    steer_pub = nh.advertise<rr_msgs::steering>("plan/steering", 1);
    path_pub = nh.advertise<nav_msgs::Path>("plan/path", 1);

    speed_message.reset(new rr_msgs::speed);
    steer_message.reset(new rr_msgs::steering);
    update_messages(0, 0);

    total_planning_time = 0;
    total_plans = 0;

    steering_model->Reset(0, ros::Time::now().toSec());
    speed_filter->Reset(0, ros::Time::now().toSec());

    ROS_INFO("planner initialized");

    ros::Rate rate(30);
    is_new_msg = false;
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();

        steering_model->Update(steer_message->angle, ros::Time::now().toSec());

        if (is_new_msg) {
            is_new_msg = false;

            auto start = ros::WallTime::now();
            processMap(last_map_msg);
            double seconds = (ros::WallTime::now() - start).toSec();

            total_planning_time += seconds;
            total_plans++;
            double sec_avg = total_planning_time / total_plans;
            ROS_INFO("PlanningOptimizer took %0.1fms, average %0.2fms", seconds * 1000, sec_avg * 1000);
        }
    }

    return 0;
}
