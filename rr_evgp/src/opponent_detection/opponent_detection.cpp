#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

visualization_msgs::Marker cluster_points;
cluster_points.header.frame_id = "/base_footprint";
cluster_points.header.stamp = ros::Time::now();

cluster_points.ns = "points";
cluster_points.id = 0;

cluster_points.type = visualization_msgs::Marker::POINTS;
cluster_points.action = visualization_msgs::Marker::ADD;

cluster_points.pose.position.x = 0;
cluster_points.pose.position.y = 0;
cluster_points.pose.position.z = 0;
cluster_points.pose.orientation.x = 0.0;
cluster_points.pose.orientation.y = 0.0;
cluster_points.pose.orientation.z = 0.0;
cluster_points.pose.orientation.w = 1.0;

cluster_points.scale.x = 0.1;
cluster_points.scale.y = 0.1;
cluster_points.scale.z = 0.1;

cluster_points.color.r = 1.0f;
cluster_points.color.g = 0.0f;
cluster_points.color.b = 0.0f;
cluster_points.color.a = 1.0;

cluster_points.points = [];

cluster_points.lifetime = ros::Duration();

// define callback function
void cluster_callback(sensor_msgs::PointCloud2 cloud_msg)
{
  // initialize PCLPointCloud2 object
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);

  // convert cloud_msg to PointCloud2 type
  pcl_conversions::toPCL(cloud_msg, *cloud);

  // initialize PointCloud<pcl::PointXYZRGB> object
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

  // convert the pcl::PCLPointCloud2 type to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(*cloud, *cloud_filtered);

  // **FLOOR SEGMENTATION**

  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);

  // passthrough filter to segment out ground
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*floor_segmented);

  // set cloud_filtered to the passthrough filter results
  *cloud_filtered = *floor_segmented;

  // for each point in cloud_filtered, add to points variable of Marker
  BOOST_FOREACH (pcl::PointXYZRGB pt, cloud_filtered->points)
    cluster_points.points.push_back(pt);

  // **WALL SEGMENTATION**

  // perform euclidean cluster segmentation to separate individual objects

  // Create the KdTree object for the search method of the extraction
  // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  // tree->setInputCloud (xyzCloudPtrRansacFiltered);

  // // create the extraction object for the clusters
  // std::vector<pcl::PointIndices> cluster_indices;
  // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // // specify euclidean cluster parameters
  // ec.setClusterTolerance (0.02); // 2cm
  // ec.setMinClusterSize (10);
  // ec.setMaxClusterSize (1000);
  // ec.setSearchMethod (tree);
  // ec.setInputCloud (xyzCloudPtrRansacFiltered);
  // // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  // ec.extract (cluster_indices);

  // // declare an instance of the SegmentedClustersArray message
  // rr_msgs::clusters CloudClusters;

  // // declare the output variable instances
  // sensor_msgs::PointCloud2 output;
  // pcl::PCLPointCloud2 outputPCL;

  // // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
  // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  // {

  //   // create a pcl object to hold the extracted cluster
  //   pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

  //   // now we are in a vector of indices pertaining to a single cluster.
  //   // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
  //   for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  //   {
  //     clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);

  //       }

  //   // convert to pcl::PCLPointCloud2
  //   pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);

  //   // Convert to ROS data type
  //   pcl_conversions::fromPCL(outputPCL, output);

  //   // add the cluster to the array message
  //   //clusterData.cluster = output;
  //   CloudClusters.clusters.push_back(output);

  // }

  // // publish the clusters
  // m_clusterPub.publish(CloudClusters);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "opponent_detection");
  ros::NodeHandle nh;

  ros::Subscriber m_sub = nh.subscribe ("/velodyne_points", 1, &cluster_callback); 
  ros::Publisher m_clusterPub = nh.advertise<rr_msgs::clusters> ("/clusters", 1);

  while(ros::ok())
  {
    m_clusterPub.publish(clusters_msg);
    ros::spinOnce();
  }

}
