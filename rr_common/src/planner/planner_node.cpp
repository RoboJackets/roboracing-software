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

#include "planner.h"


std::unique_ptr<rr::planning::Planner> planner;

ros::Publisher speed_pub;
ros::Publisher steer_pub;
ros::Publisher path_pub;


void mapCallback(const sensor_msgs::PointCloud2ConstPtr& map) {
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*map, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

  if (cloud->empty()) {
    // Do not publish new commands
    ROS_WARN("environment map pointcloud is empty");
    return;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree(false);
  kdtree.setInputCloud(cloud);

  rr::planning::PlannedPath plan = planner->Plan(kdtree);

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

std::vector<double> getDoubleListParam(const ros::NodeHandle& nhp, const std::string& name, char delim) {
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

  rr::planning::Planner::Params params;

  nhp.getParam("wheel_base", params.wheel_base);
  nhp.getParam("lateral_accel", params.lateral_accel);
  nhp.getParam("collision_dist_front", params.collision_box.length_front);
  nhp.getParam("collision_dist_back", params.collision_box.length_back);
  nhp.getParam("collision_dist_side", params.collision_box.width_left);
  params.collision_box.width_right = params.collision_box.width_left;

  nhp.getParam("n_path_segments", params.n_path_segments);
  params.segment_distances = getDoubleListParam(nhp, "segment_distances", ' ');
  params.steer_limits = getDoubleListParam(nhp, "steer_limits", ' ');
  params.steer_stddevs = getDoubleListParam(nhp, "steer_stddevs", ' ');

  nhp.getParam("obstacle_search_radius", params.obstacle_search_radius);
  nhp.getParam("path_similarity_cutoff", params.path_similarity_cutoff);
  nhp.getParam("max_relative_cost", params.max_relative_cost);
  nhp.getParam("k_dist", params.k_dist);
  nhp.getParam("k_speed", params.k_speed);

  nhp.getParam("n_control_samples", params.n_control_samples);
  nhp.getParam("distance_increment", params.distance_increment);

  nhp.getParam("smoothing_array_size", params.smoothing_array_size);
  nhp.getParam("max_speed", params.max_speed);

  std::string obstacle_cloud_topic;
  nhp.getParam("input_cloud_topic", obstacle_cloud_topic);

  planner.reset(new rr::planning::Planner(params));

  auto map_sub = nh.subscribe(obstacle_cloud_topic, 1, mapCallback);
  speed_pub = nh.advertise<rr_platform::speed>("plan/speed", 1);
  steer_pub = nh.advertise<rr_platform::steering>("plan/steering", 1);
  path_pub = nh.advertise<nav_msgs::Path>("plan/path", 1);

  ROS_INFO("Planner initialized");
  ros::spin();

  return 0;
}
