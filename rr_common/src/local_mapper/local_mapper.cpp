#include <deque>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <rr_common/RelativePoseHistoryClient.h>
#include <rr_platform/CameraGeometry.h>
#include <rr_platform/angle_utils.hpp>

// types
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

struct SourcePair {
  PointCloud cloud;
  ros::Time time;
};

// global variables for PointCloud transformation
std::deque<SourcePair> sources;
rr::RelativePoseHistoryClient pose_history;
PointCloud::Ptr local_map_unfiltered;

// other global variables
ros::Publisher map_publisher;
pcl::VoxelGrid<pcl::PointXYZ> filter;
ros::Duration time_horizon;
std::vector<geometry_msgs::Point> in_frame_polygon;


void obstacles_callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  // populate new source pair
  auto& new_source = sources.emplace_front();
  pcl::fromROSMsg<pcl::PointXYZ>(*msg, new_source.cloud);
  new_source.time = msg->header.stamp;

  // remove old sources
  while (sources.back().time + time_horizon < new_source.time) {
    sources.pop_back();
  }

  // handle rosbag time loop
  if (sources.back().time > sources.front().time) {
    sources.erase(sources.begin() + 1, sources.end());
  }

  // build map
  bool is_newest_source = true;
  local_map_unfiltered->clear();
  for (auto& source : sources) {
    // look up source point cloud, time of publication, and offset from current to source
    const auto relative_pose_2d = pose_history.GetRelativePoseAtTime(source.time);

    // construct Transform object for offset from current pose to source pose
    tf::Quaternion rotation = tf::createQuaternionFromYaw(relative_pose_2d.theta);
    tf::Vector3 translation = {relative_pose_2d.x, relative_pose_2d.y, 0};
    tf::Transform transform(rotation, translation);

    // apply linear transformation
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl_ros::transformPointCloud(source.cloud, transformed_cloud, transform);

    // if this source pointcloud is not the most recent one, only keep the points
    //   from it which are not in the current FOV
    if (is_newest_source) {
      local_map_unfiltered->insert(local_map_unfiltered->end(), transformed_cloud.begin(), transformed_cloud.end());
    } else {
      for (int point_idx = transformed_cloud.size()-1; point_idx >= 0; point_idx--) {
        const auto& pt = transformed_cloud.points[point_idx];
        bool add;
        if (pt.x < in_frame_polygon[0].x) {
          add = true;
        } else {
          bool inside_all = true;
          for (int i = 0; i < 4; i++) {
            const auto& border1 = in_frame_polygon[i];
            const auto& border2 = in_frame_polygon[(i+1) % 4];
            Eigen::Vector2d diff(pt.x - border1.x, pt.y - border1.y);
            Eigen::Vector2d normal(border1.y - border2.y, border2.x - border1.x);

            inside_all &= (normal.dot(diff) > 0);
            if (!inside_all) {
              break;
            }
          }
          add = !inside_all;
        }

        if (add) {
          local_map_unfiltered->push_back(pt);
        } else {
          source.cloud.erase(source.cloud.begin() + point_idx);
        }
      }
    }

    is_newest_source = false;
  }

  // filter using VoxelGrid
  PointCloud local_map;
  filter.setInputCloud(local_map_unfiltered);
  filter.filter(local_map);

  // publish message
  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(local_map, map_msg);
  map_msg.header.stamp = sources.front().time;  // use time from most recent included point cloud
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

  std::string cam_info_topic;
  nhp.getParam("camera_info_topic", cam_info_topic);
  std::string cam_link_name;
  nhp.getParam("camera_link_name", cam_link_name);
  rr::CameraGeometry camera_geometry;
  camera_geometry.LoadInfo(nh, cam_info_topic, cam_link_name, 300);

  // proportion of each side of the image that should *not* be considered part of the
  //   field of view
  double keep_border_prop;
  nhp.getParam("keep_border_prop", keep_border_prop);

  // find FOV convex polygon
  int horizon_row = 0;
  for (int row = 0; row < camera_geometry.GetImageHeight(); row++) {
    auto [crossed_horizon, projection] = camera_geometry.ProjectToWorld(row, 0);
    if (crossed_horizon && projection.x < 100) {
      horizon_row = row;
      break;
    }
  }

  const auto w1 = static_cast<int>(camera_geometry.GetImageWidth() * keep_border_prop);
  const auto w2 = camera_geometry.GetImageWidth() - w1;
  const auto h1 = static_cast<int>(camera_geometry.GetImageHeight() * 0.8);
  in_frame_polygon.push_back(std::get<1>(camera_geometry.ProjectToWorld(h1, w1)));
  in_frame_polygon.push_back(std::get<1>(camera_geometry.ProjectToWorld(h1, w2)));
  in_frame_polygon.push_back(std::get<1>(camera_geometry.ProjectToWorld(horizon_row, w2)));
  in_frame_polygon.push_back(std::get<1>(camera_geometry.ProjectToWorld(horizon_row, w1)));

  auto sub1 = nh.subscribe(obstacles_topic, 1, obstacles_callback);
  auto sub2 = pose_history.RegisterCallback(nh);

  map_publisher = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 1);

  local_map_unfiltered.reset(new PointCloud);

  filter.setLeafSize(0.05, 0.05, 0.05);

  ros::spin();
  return 0;
}
