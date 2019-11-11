#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <parameter_assertions/assertions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>

#include <rr_msgs/speed.h>
#include <rr_msgs/steering.h>

#include "planner/annealing_planner.h"
#include "planner/random_sample_planner.h"

std::unique_ptr<rr::Planner> planner;
std::unique_ptr<rr::DistanceChecker> distance_checker;

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

    distance_checker = std::make_unique<rr::DistanceChecker>(hitbox, map_dimensions);

    double wheel_base;
    assertions::getParam(nhp, "wheel_base", wheel_base);
    double lateral_accel;
    assertions::getParam(nhp, "lateral_accel", lateral_accel);
    double distance_increment;
    assertions::getParam(nhp, "distance_increment", distance_increment);
    double max_speed;
    assertions::getParam(nhp, "max_speed", max_speed);
    double steering_speed;
    assertions::getParam(nhp, "steering_speed", steering_speed);
    std::vector<int> segment_sections;
    assertions::getParam(nhp, "segment_sections", segment_sections);

    rr::BicycleModel model(wheel_base, lateral_accel, distance_increment, max_speed, steering_speed, segment_sections);

    std::string obstacle_cloud_topic;
    assertions::getParam(nhp, "input_cloud_topic", obstacle_cloud_topic);
    std::string planner_type;
    assertions::getParam(nhp, "planner_type", planner_type);

    if (planner_type == "random_sample") {
        planner = std::make_unique<rr::RandomSamplePlanner>(nhp, *distance_checker, model);
    } else if (planner_type == "annealing") {
        planner = std::make_unique<rr::AnnealingPlanner>(nhp, *distance_checker, model);
    } else {
        ROS_ERROR_STREAM("[Planner] Error: unknown planner type \"" << planner_type << "\"");
        ros::shutdown();
    }

    caution_duration = ros::Duration(assertions::param(nhp, "impasse_caution_duration", 0.0));
    reverse_duration = ros::Duration(assertions::param(nhp, "impasse_reverse_duration", 0.0));
    caution_start_time = ros::Time(0);
    reverse_start_time = ros::Time(0);
    reverse_state = OK;

    last_speed_time = ros::Time(0);
    max_drive_accel = assertions::param(nhp, "max_drive_accel", 1000.0);

    steering_gain = assertions::param(nhp, "steering_gain", 1.0);

    auto map_sub = nh.subscribe(obstacle_cloud_topic, 1, mapCallback);
    speed_pub = nh.advertise<rr_msgs::speed>("plan/speed", 1);
    steer_pub = nh.advertise<rr_msgs::steering>("plan/steering", 1);
    path_pub = nh.advertise<nav_msgs::Path>("plan/path", 1);

    speed_message.reset(new rr_msgs::speed);
    steer_message.reset(new rr_msgs::steering);
    update_messages(0, 0);

    ROS_INFO("Planner initialized");

    ros::Rate rate(30);
    is_new_msg = false;
    while (ros::ok()) {
        ros::spinOnce();

        if (is_new_msg) {
            is_new_msg = false;
            auto start = ros::WallTime::now();

            processMap(last_map_msg);

            double seconds = (ros::WallTime::now() - start).toSec();
            ROS_INFO("Planner took %0.1fms", seconds * 1000);
        }

        rate.sleep();
    }

    return 0;
}
