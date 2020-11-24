#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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
#include <rr_msgs/clusters.h>

rr_msgs::clusters clusters_msg;

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

  // FLOOR SEGMENTATION

  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);

  // passthrough filter to segment out ground
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*floor_segmented);

  *cloud_filtered = *floor_segmented;

  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 outputPCL;

  // convert to pcl::PCLPointCloud2
  pcl::toPCLPointCloud2(*cloud_filtered, outputPCL);

  // Convert to ROS data type
  pcl_conversions::fromPCL(outputPCL, output);

  // add the cluster to the array message
  clusters_msg.clusters.push_back(output);

  // // WALL SEGMENTATION

  // // create a pcl object to hold the ransac filtered results
  // pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);


  // // perform ransac planar filtration to remove table top
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // // Create the segmentation object
  // pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
  // // Optional
  // seg1.setOptimizeCoefficients (true);
  // // Mandatory
  // seg1.setModelType (pcl::SACMODEL_PLANE);
  // seg1.setMethodType (pcl::SAC_RANSAC);
  // seg1.setDistanceThreshold (0.04);

  // seg1.setInputCloud (xyzCloudPtrFiltered);
  // seg1.segment (*inliers, *coefficients);

  // // Create the filtering object
  // pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  // //extract.setInputCloud (xyzCloudPtrFiltered);
  // extract.setInputCloud (xyzCloudPtrFiltered);
  // extract.setIndices (inliers);
  // extract.setNegative (true);
  // extract.filter (*xyzCloudPtrRansacFiltered);

  // // perform euclidean cluster segmentation to separate individual objects

  // // Create the KdTree object for the search method of the extraction
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
