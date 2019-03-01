#include <deque>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "pose_tracker/RelativePoseHistoryClient.h"

// types
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

struct SourcePair {
  PointCloud cloud;
  ros::Time time;
};

// global variables for PointCloud transformation
std::deque<SourcePair> sources;
RelativePoseHistoryClient pose_history;
PointCloud::Ptr local_map_unfiltered;

// other global variables
ros::Publisher map_publisher;
pcl::VoxelGrid<pcl::PointXYZ> filter;
ros::Duration time_horizon;


void obstacles_callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  // populate new source pair
  auto& new_source = sources.emplace_back();
  pcl::fromROSMsg<pcl::PointXYZ>(*msg, new_source.cloud);
  new_source.time = msg->header.stamp;

  // remove old sources
  while (sources.front().time + time_horizon < new_source.time) {
    sources.pop_front();
  }

  // handle rosbag time loop
  if (sources.front().time > sources.back().time) {
    sources.erase(sources.begin(), sources.end() - 1);
  }

  // build map
  local_map_unfiltered->clear();
  for (const auto& source : sources) {
    // look up source point cloud, time of publication, and offset from current to source
    const auto relative_pose_2d = pose_history.GetRelativePoseAtTime(source.time);

    // construct Transform object for offset from current pose to source pose
    tf::Quaternion rotation = tf::createQuaternionFromYaw(relative_pose_2d.theta);
    tf::Vector3 translation = {relative_pose_2d.x, relative_pose_2d.y, 0};
    tf::Transform transform(rotation, translation);

    // apply linear transformation
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl_ros::transformPointCloud(source.cloud, transformed_cloud, transform);

    // add transfromed cloud to map
    local_map_unfiltered->insert(local_map_unfiltered->end(), transformed_cloud.begin(), transformed_cloud.end());
  }

  // filter using VoxelGrid
  PointCloud local_map;
  filter.setInputCloud(local_map_unfiltered);
  filter.filter(local_map);

  // publish message
  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(local_map, map_msg);
  map_msg.header.stamp = sources.back().time;  // use time from most recent included point cloud
  map_msg.header.frame_id = "base_footprint";
  map_publisher.publish(map_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_mapper");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  std::string obstacles_topic;
  nhp.getParam("current_obstacles_topic", obstacles_topic);

  double time_horizon_tmp;
  nhp.getParam("time_horizon", time_horizon_tmp);
  time_horizon = ros::Duration(time_horizon_tmp);

  auto sub1 = nh.subscribe(obstacles_topic, 1, obstacles_callback);
  auto sub2 = pose_history.RegisterCallback(nh);

  map_publisher = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 1);

  local_map_unfiltered.reset(new PointCloud);

  filter.setLeafSize(0.05, 0.05, 0.05);

  ros::spin();
  return 0;
}
