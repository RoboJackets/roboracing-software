/**
 * Simple program to subscribe to several pointclouds and output their combined result at
 * a set frequency.
 */

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

using cloud_t = pcl::PointCloud<pcl::PointXYZ>;
using cloud_ptr_t = pcl::PointCloud<pcl::PointXYZ>::Ptr;

std::map<std::string, sensor_msgs::PointCloud2ConstPtr> cache;
bool has_new_info;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, std::string topic) {
  cache[topic] = msg;
  has_new_info = true;
}

/**
 * @note http://stackoverflow.com/a/27511119
 */
std::vector<std::string> split(const std::string& s, char delim) {
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> elems;
  while (std::getline(ss, item, delim)) {
    elems.push_back(std::move(item));
  }
  return elems;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_combiner");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  cloud_ptr_t combo_cloud(new cloud_t);
  cloud_ptr_t transformed(new cloud_t);

  std::string sourceList = nh_private.param("sources", std::string());
  std::string publishName = nh_private.param("destination", std::string("/map"));
  std::string combinedFrame = nh_private.param("combined_frame", std::string("base_footprint"));
  int refreshRate = nh_private.param("refresh_rate", 30);
  float vgFilterSize = nh_private.param("vg_filter_size", 0.05);

  auto topics = split(sourceList, ' ');

  std::vector<ros::Subscriber> partial_Subscribers;

  for (const auto& topic : topics) {
    partial_Subscribers.push_back(
        nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, boost::bind(cloudCallback, _1, topic)));
    ROS_INFO_STREAM("Mapper subscribed to " << topic);
  }

  auto combo_pub = nh.advertise<sensor_msgs::PointCloud2>(publishName, 1);

  // set up point reduction filter
  pcl::VoxelGrid<pcl::PointXYZ> filterVG;
  filterVG.setLeafSize(vgFilterSize, vgFilterSize, vgFilterSize);

  tf::TransformListener tfListener;

  ROS_INFO("starting pointcloud_combiner main loop");

  ros::Rate rate(refreshRate);
  while (ros::ok()) {
    ros::spinOnce();

    if (has_new_info) {
      combo_cloud->clear();

      for (std::pair<std::string, sensor_msgs::PointCloud2ConstPtr> entry_pair :
           cache) {  // copy for kind-of thread safety?
        if (!entry_pair.second) {
          continue;  // null pointer for message
        }

        const auto& cloud_msg = *(entry_pair.second);

        // convert from message to pcl pointcloud
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(cloud_msg, pcl_pc2);

        cloud_t partialCloud;
        pcl::fromPCLPointCloud2(pcl_pc2, partialCloud);

        if (partialCloud.points.empty()) {
          continue;
        }

        // frame transform
        transformed->clear();
        tfListener.waitForTransform(cloud_msg.header.frame_id, combinedFrame, ros::Time(0), ros::Duration(5.0));
        pcl_ros::transformPointCloud(combinedFrame, partialCloud, *transformed, tfListener);

        *(combo_cloud) += *transformed;
      }

      // make 2D
      for (auto& pt : combo_cloud->points) {
        pt.z = 0;
      }

      ROS_INFO("combo_cloud has %d points", (int)combo_cloud->points.size());

      // convert back to message
      if (!combo_cloud->points.empty()) {
        pcl::PCLPointCloud2 combo_pc2;
        pcl::toPCLPointCloud2(*combo_cloud, combo_pc2);
        sensor_msgs::PointCloud2 msg;
        pcl_conversions::fromPCL(combo_pc2, msg);

        msg.header.frame_id = combinedFrame;
        msg.header.stamp = ros::Time::now();

        combo_pub.publish(msg);
      } else {
        ROS_INFO("pointcloud empty");
      }

      has_new_info = false;
    }

    rate.sleep();
  }

  return 0;
}
