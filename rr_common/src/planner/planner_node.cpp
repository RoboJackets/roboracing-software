#include <map>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PoseStamped.h>

#include <rr_platform/speed.h>
#include <rr_platform/steering.h>

#include "random_sample_planner.h"


std::unique_ptr<rr::Planner> planner;
std::unique_ptr<rr::DistanceChecker> distance_checker;

ros::Publisher speed_pub;
ros::Publisher steer_pub;
ros::Publisher path_pub;


void mapCallback(const sensor_msgs::PointCloud2ConstPtr& map) {
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*map, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

  for(auto point_it = cloud->begin(); point_it != cloud->end();) {
    if (distance_checker->GetCollision(*point_it)) {
      point_it = cloud->erase(point_it);
    } else {
      point_it++;
    }
  }

  if (cloud->empty()) {
    // Do not publish new commands
    ROS_WARN("environment map pointcloud is empty");
    return;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree(false);
  kdtree.setInputCloud(cloud);

  rr::PlannedPath plan = planner->Plan(kdtree);

  rr_platform::speedPtr speedMSG(new rr_platform::speed);
  rr_platform::steeringPtr steerMSG(new rr_platform::steering);
  steerMSG->angle = plan.path[0].steer;
  speedMSG->speed = plan.path[0].speed;
  steerMSG->header.stamp = ros::Time::now();
  speedMSG->header.stamp = ros::Time::now();
  speed_pub.publish(speedMSG);
  steer_pub.publish(steerMSG);

  if(path_pub.getNumSubscribers() > 0) {
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

std::vector<double> getDoubleListParam(const ros::NodeHandle& nhp,
                                       const std::string& name, char delim) {
  std::string listAsString;
  nhp.getParam(name, listAsString);

  std::vector<double> out;

  std::stringstream ss(listAsString);
  std::string s;
  while (std::getline(ss, s, delim)) {
    out.push_back(std::stod(s));
  }

  return out;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planner");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  rr::RandomSamplePlanner::Params params;

  double length_front, length_back, width_left, width_right,
      obs_search_rad;
  nhp.getParam("collision_dist_front", length_front);
  nhp.getParam("collision_dist_back", length_back);
  nhp.getParam("collision_dist_side", width_left);
  width_right = width_left;
  nhp.getParam("obstacle_search_radius", obs_search_rad);

  distance_checker.reset(new rr::DistanceChecker(
      length_front, length_back, width_left, width_right, obs_search_rad));

  double wheel_base, lateral_accel, distance_increment, max_speed;
  nhp.getParam("wheel_base", wheel_base);
  nhp.getParam("lateral_accel", lateral_accel);
  nhp.getParam("distance_increment", distance_increment);
  nhp.getParam("max_speed", max_speed);
  auto segment_distances = getDoubleListParam(nhp, "segment_distances", ' ');

  rr::BicycleModel model(wheel_base, lateral_accel, distance_increment,
      max_speed, segment_distances);

  nhp.getParam("n_path_segments", params.n_path_segments);
  params.steer_limits = getDoubleListParam(nhp, "steer_limits", ' ');
  params.steer_stddevs = getDoubleListParam(nhp, "steer_stddevs", ' ');

  nhp.getParam("path_similarity_cutoff", params.path_similarity_cutoff);
  nhp.getParam("max_relative_cost", params.max_relative_cost);
  nhp.getParam("k_dist", params.k_dist);
  nhp.getParam("k_speed", params.k_speed);

  nhp.getParam("n_control_samples", params.n_control_samples);

  nhp.getParam("smoothing_array_size", params.smoothing_array_size);

  nhp.getParam("obs_dist_slow_thresh", params.obs_dist_slow_thresh);
  nhp.getParam("obs_dist_slow_ratio", params.obs_dist_slow_ratio);

  std::string obstacle_cloud_topic;
  nhp.getParam("input_cloud_topic", obstacle_cloud_topic);

  planner.reset(new rr::RandomSamplePlanner(*distance_checker, model, params));

  auto map_sub = nh.subscribe(obstacle_cloud_topic, 1, mapCallback);
  speed_pub = nh.advertise<rr_platform::speed>("plan/speed", 1);
  steer_pub = nh.advertise<rr_platform::steering>("plan/steering", 1);
  path_pub = nh.advertise<nav_msgs::Path>("plan/path", 1);

  ROS_INFO("Planner initialized");
  ros::spin();

  return 0;
}
