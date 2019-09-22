#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>

#include <rr_platform/speed.h>
#include <rr_platform/steering.h>

#include "planner/annealing_planner.h"
#include "planner/random_sample_planner.h"

std::unique_ptr<rr::Planner> planner;
std::unique_ptr<rr::DistanceChecker> distance_checker;

ros::Publisher speed_pub;
ros::Publisher steer_pub;
ros::Publisher path_pub;

rr_platform::speedPtr speed_message;
rr_platform::steeringPtr steer_message;

sensor_msgs::PointCloud2ConstPtr last_map_msg;
bool is_new_msg;

enum reverse_state_t { OK, CAUTION, REVERSE };

ros::Duration caution_duration;
ros::Duration reverse_duration;
ros::Time caution_start_time;
ros::Time reverse_start_time;
reverse_state_t reverse_state;

ros::Time last_speed_time;
double max_drive_accel;

double steering_gain;

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
        // Do not publish new commands
        ROS_WARN("environment map pointcloud is empty");
        return;
    }

    rr::PlannedPath plan = planner->Plan(cloud);

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
        ROS_WARN_STREAM("Planner encountered unknown reverse state");
    }

    if (REVERSE == reverse_state) {
        update_messages(-0.8, 0);
        ROS_INFO_STREAM("Planner reversing");
    } else if (plan.has_collision) {
        ROS_INFO_STREAM("Planner: no path found but not reversing; reusing previous message");
    } else {
        // filter/cap acceleration
        double dt;
        if (last_speed_time == ros::Time(0)) {
            dt = 0;
        } else {
            dt = (ros::Time::now() - last_speed_time).toSec();
        }

        double max_new_speed = speed_message->speed + max_drive_accel * dt;
        double new_speed = std::min(plan.path.front().speed, max_new_speed);

        update_messages(new_speed, plan.path[0].steer * steering_gain);
    }

    last_speed_time = ros::Time::now();

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

template <typename T>
T getParamAssert(const ros::NodeHandle& nhp, const std::string& name) {
    T out;
    if (!nhp.getParam(name, out)) {
        ROS_ERROR_STREAM("[Planner] Param name " << name << " needs to be defined");
        std::exit(-1);
    }
    return out;
}

template <typename T>
std::vector<T> getNumericListParam(const ros::NodeHandle& nhp, const std::string& name, char delim) {
    auto listAsString = getParamAssert<std::string>(nhp, name);
    std::vector<T> out;

    std::stringstream ss(listAsString);
    std::string s;
    while (std::getline(ss, s, delim)) {
        out.push_back(static_cast<T>(std::stod(s)));
    }

    return out;
}

rr::RandomSamplePlanner::Params getRandomSampleParams(const ros::NodeHandle& nhp) {
    rr::RandomSamplePlanner::Params params;

    params.n_path_segments = getParamAssert<int>(nhp, "n_path_segments");
    params.steer_limits = getNumericListParam<double>(nhp, "steer_limits", ' ');
    params.steer_stddevs = getNumericListParam<double>(nhp, "steer_stddevs", ' ');

    params.path_similarity_cutoff = getParamAssert<double>(nhp, "path_similarity_cutoff");
    params.max_relative_cost = getParamAssert<double>(nhp, "max_relative_cost");
    params.k_dist = getParamAssert<double>(nhp, "k_dist");
    params.k_speed = getParamAssert<double>(nhp, "k_speed");

    params.n_control_samples = getParamAssert<int>(nhp, "n_control_samples");

    params.smoothing_array_size = getParamAssert<int>(nhp, "smoothing_array_size");

    params.obs_dist_slow_thresh = getParamAssert<double>(nhp, "obs_dist_slow_thresh");
    params.obs_dist_slow_ratio = getParamAssert<double>(nhp, "obs_dist_slow_ratio");

    return params;
}

rr::AnnealingPlanner::Params getAnnealingParams(const ros::NodeHandle& nhp) {
    rr::AnnealingPlanner::Params params;

    params.n_path_segments = static_cast<unsigned int>(getParamAssert<int>(nhp, "n_path_segments"));
    params.annealing_steps = static_cast<unsigned int>(getParamAssert<int>(nhp, "annealing_steps"));

    params.k_dist = getParamAssert<double>(nhp, "k_dist");
    params.k_speed = getParamAssert<double>(nhp, "k_speed");
    params.k_final_pose = getParamAssert<double>(nhp, "k_final_pose");
    params.k_angle = getParamAssert<double>(nhp, "k_angle");
    params.collision_penalty = getParamAssert<double>(nhp, "collision_penalty");
    params.max_steering = getParamAssert<double>(nhp, "max_steering");
    params.acceptance_scale = getParamAssert<double>(nhp, "acceptance_scale");

    params.temperature_start = getParamAssert<double>(nhp, "temperature_start");
    params.temperature_end = getParamAssert<double>(nhp, "temperature_end");

    return params;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    rr::CenteredBox box;
    ros::NodeHandle nh_hitbox(nhp, "collision_hitbox");
    box.length_front = getParamAssert<double>(nh_hitbox, "front");
    box.length_back = getParamAssert<double>(nh_hitbox, "back");
    box.width_left = box.width_right = getParamAssert<double>(nh_hitbox, "side");

    rr::CenteredBox map_dimensions;
    map_dimensions.length_front = 7;
    map_dimensions.length_back = 5;
    map_dimensions.width_right = map_dimensions.width_left = 6;

    distance_checker = std::make_unique<rr::DistanceChecker>(box, map_dimensions);

    auto wheel_base = getParamAssert<double>(nhp, "wheel_base");
    auto lateral_accel = getParamAssert<double>(nhp, "lateral_accel");
    auto distance_increment = getParamAssert<double>(nhp, "distance_increment");
    auto max_speed = getParamAssert<double>(nhp, "max_speed");
    auto steering_speed = getParamAssert<double>(nhp, "steering_speed");
    auto segment_sections = getNumericListParam<int>(nhp, "segment_sections", ' ');

    rr::BicycleModel model(wheel_base, lateral_accel, distance_increment, max_speed, steering_speed, segment_sections);

    auto obstacle_cloud_topic = getParamAssert<std::string>(nhp, "input_cloud_topic");
    auto planner_type = getParamAssert<std::string>(nhp, "planner_type");

    if (planner_type == "random_sample") {
        auto params = getRandomSampleParams(nhp);
        planner = std::make_unique<rr::RandomSamplePlanner>(*distance_checker, model, params);
    } else if (planner_type == "annealing") {
        auto params = getAnnealingParams(nhp);
        planner = std::make_unique<rr::AnnealingPlanner>(*distance_checker, model, params);
    } else {
        ROS_ERROR_STREAM("[Planner] Error: unknown planner type \"" << planner_type << "\"");
        std::exit(-2);
    }

    caution_duration = ros::Duration(getParamAssert<double>(nhp, "impasse_caution_duration"));
    reverse_duration = ros::Duration(getParamAssert<double>(nhp, "impasse_reverse_duration"));
    caution_start_time = ros::Time(0);
    reverse_start_time = ros::Time(0);
    reverse_state = OK;

    last_speed_time = ros::Time(0);
    max_drive_accel = getParamAssert<double>(nhp, "max_drive_accel");

    steering_gain = getParamAssert<double>(nhp, "steering_gain");

    auto map_sub = nh.subscribe(obstacle_cloud_topic, 1, mapCallback);
    speed_pub = nh.advertise<rr_platform::speed>("plan/speed", 1);
    steer_pub = nh.advertise<rr_platform::steering>("plan/steering", 1);
    path_pub = nh.advertise<nav_msgs::Path>("plan/path", 1);

    speed_message.reset(new rr_platform::speed);
    steer_message.reset(new rr_platform::steering);
    update_messages(0, 0);

    ROS_INFO("Planner initialized");

    ros::Rate rate(30);
    is_new_msg = false;
    while (ros::ok()) {
        ros::spinOnce();

        if (is_new_msg) {
            is_new_msg = false;
            auto start = ros::Time::now();

            processMap(last_map_msg);

            double seconds = (ros::Time::now() - start).toSec();
            ROS_INFO("Planner took %0.1fms", seconds * 1000);
        }

        rate.sleep();
    }

    return 0;
}
