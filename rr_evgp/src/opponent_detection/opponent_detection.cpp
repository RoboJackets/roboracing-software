#include <rr_evgp/opponent_detection.h>

// publishes clustered clouds
void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
  cloud_ptr->header.frame_id = "base_footprint";

  // initialize PCLPointCloud2 for output
  pcl::PCLPointCloud2 outputPCL;

  // convert cloud_result back to PCLPointCloud2
  pcl::toPCLPointCloud2(*cloud_ptr, outputPCL);

  // initialize sensor_msgs PointCloud2 for output
  sensor_msgs::PointCloud2 output;

  // convert outputPCL back to ROS data type (sensor_msgs::PointCloud2)
  pcl_conversions::fromPCL(outputPCL, output);

  cloud_pub.publish(output);
}

// publishes clustered clouds as Markers
void publishMarker(std::vector<geometry_msgs::Point> markers, int color)
{
  visualization_msgs::Marker marker;

  marker.header.stamp = ros::Time::now();
  marker.lifetime = ros::Duration();
  marker.header.frame_id = "base_footprint";

  marker.ns = "marked_clusters";

  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.points = markers;

  marker.pose.position.x = 1;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0.5;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.color.a = 1.0;
  marker.color.r = colors[color % colors.size()][0];
  marker.color.g = colors[color % colors.size()][1];
  marker.color.b = colors[color % colors.size()][2];

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker_pub.publish(marker);
}

void callback(sensor_msgs::PointCloud2 cloud_msg)
{
  // initialize PCLPointCloud2 object
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);

  // convert cloud_msg to PCLPointCloud2 type
  pcl_conversions::toPCL(cloud_msg, *cloud);

  // initialize PointCloud of PointXYZ objects
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

  // convert cloud_msg from PCLPointCloud2 to PointCloud of PointXYZ objects
  pcl::fromPCLPointCloud2(*cloud, *cloud2);

  // **GROUND SEGMENTATION**

  // initialize another PC of PointXYZ objects to hold the passthrough filter results
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_segmented (new pcl::PointCloud<pcl::PointXYZ>);

  // passthrough filter to segment out ground
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud2);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.4, 5.0);
  pass.filter(*ground_segmented);

  // **DOWNSAMPLING/CLUSTERING**

  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(ground_segmented);
  vg.setLeafSize(0.1f, 0.1f, 0.1f);
  vg.filter(*voxel_filtered);

  // creating KdTree object for extracting clusters
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (voxel_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.35);
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(100);
  ec.setSearchMethod(tree);
  ec.setInputCloud(voxel_filtered);
  ec.extract(cluster_indices);

  // **PUBLISHING**

  // holding containers for PCs and Markers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<geometry_msgs::Point> marker_cluster = {};
  geometry_msgs::Point marker_point;

  // iterators
  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;

  // for each PointIndices object, turn it into a PointCloud of PointXYZ objects
  for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    for (pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      cloud_cluster->push_back((*voxel_filtered)[*pit]);

      marker_point.x = (*voxel_filtered)[*pit].x;
      marker_point.y = (*voxel_filtered)[*pit].y;
      marker_point.z = (*voxel_filtered)[*pit].z;

      marker_cluster.push_back(marker_point);
    }

    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    publishCloud(cloud_cluster);
    publishMarker(marker_cluster, *pit);

    cloud_cluster->clear();
    marker_cluster.clear();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "opponent_detection");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, &callback);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/clusters", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("/markers", 1);

  ros::spin();
  return 0;
}
