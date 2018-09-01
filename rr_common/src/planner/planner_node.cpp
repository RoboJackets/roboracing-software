#include <map>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>

#include "planner.h"

void mapCallback(const sensor_msgs::PointCloud2ConstPtr& map) {
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planner");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  // nhp.param("N_PATH_SEGMENTS", N_PATH_SEGMENTS, 2);
  // nhp.param("N_CONTROL_SAMPLES", N_CONTROL_SAMPLES, 200);
  // nhp.param("SEGMENT_DISTANCES", segmentDistancesStr, string("3 5"));
  // nhp.param("STEER_LIMITS", steerLimitsStr, string("0.4 0.4"));
  // nhp.param("STEER_STDDEVS", steerStdDevsStr, string("0.2 0.2"));
  // nhp.param("DISTANCE_INCREMENT", DISTANCE_INCREMENT, 0.2f);
  // nhp.param("MAX_SPEED", MAX_SPEED, 0.5f);
  // nhp.param("WHEEL_BASE", WHEEL_BASE, 0.37f); //TODO get from tf tree
  // nhp.param("COLLISION_DIST_FRONT", robot_collision_box.lengthFront, 0.3f);
  // nhp.param("COLLISION_DIST_BACK", robot_collision_box.lengthBack, 0.3f);
  // nhp.param("COLLISION_DIST_SIDE", robot_collision_box.widthLeft, 0.3f);
  // nhp.param("COLLISION_PENALTY", COLLISION_PENALTY, 1000.0f);
  // nhp.param("PATH_SIMILARITY_CUTOFF", PATH_SIMILARITY_CUTOFF, 0.05f);
  // nhp.param("MAX_RELATIVE_COST", MAX_RELATIVE_COST, 2.0f);
  // nhp.param("SMOOTHING_ARRAY_SIZE", SMOOTHING_ARRAY_SIZE, 20);
  // nhp.param("OBSTACLE_SEARCH_RADIUS", OBSTACLE_SEARCH_RADIUS, 2.0f);

  nhp.getParam("input_cloud_topic", obstacleCloudTopic);
  auto map_sub = nh.subscribe(obstacleCloudTopic, 1, mapCallback);
  speed_pub = nh.advertise<rr_platform::speed>("plan/speed", 1);
  steer_pub = nh.advertise<rr_platform::steering>("plan/steering", 1);
  path_pub = nh.advertise<nav_msgs::Path>("plan/path", 1);

  auto planner = rr::planning::Planner();

  ROS_INFO("planner initialized");

  ros::spin();
  return 0;
}
