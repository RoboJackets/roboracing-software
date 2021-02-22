#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// parameters set in launch file
double low_passthru_lim;
double high_passthru_lim;

double cluster_tolerance;
int min_cluster_size;
int max_cluster_size;

// publishers
ros::Publisher cloud_pub;
ros::Publisher marker_pub;

// individual marker
visualization_msgs::Marker marker;

// final marker array
visualization_msgs::MarkerArray marker_array;

// colors
std::vector<std::array<int, 3>> colors = { { 255, 0, 0 },
                                           { 255, 127, 0 },
                                           { 0, 255, 127 },
                                           { 255, 0, 255 },
                                           { 0, 0, 255 } };

// publishes clustered clouds
void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

// adds markers to array
void addMarkers(std::vector<geometry_msgs::Point> markers, int color);

// callback of subscriber
void callback(sensor_msgs::PointCloud2 cloud_msg);
