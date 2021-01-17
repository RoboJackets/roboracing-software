#pragma once

#include <ros/ros.h>

#include <pcl/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

// publishers
ros::Publisher cloud_pub;
ros::Publisher marker_pub;

// colors
std::vector<std::array<int, 3>> colors = {{255,127,0},{255,127,0},{0,255,127},{126,0,255},{0,0,255}};

// publishes clustered clouds
void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

// publishes clustered clouds as Markers
void publishMarker(pcl::PointCloud<pcl::PointXYZ>::Ptr marker_ptr, int color);

// callback of subscriber
void callback(sensor_msgs::PointCloud2 cloud_msg);
