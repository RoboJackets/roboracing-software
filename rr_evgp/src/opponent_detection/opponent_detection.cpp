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
#include <geometry_msgs/PoseArray.h>

double cluster_tolerance_;
int min_cluster_size_, max_cluster_size_;

ros::Publisher marker_pub, centroid_pub;

// adds markers to marker array
visualization_msgs::Marker makeMarkers(const std::vector<geometry_msgs::Point>& points, int id,
                                       std_msgs::Header header) {
    visualization_msgs::Marker marker;

    marker.header = header;
    marker.ns = "marked_clusters";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points = points;

    marker.pose.orientation.w = 1.0;

    marker.color.r = rand() / double(RAND_MAX);
    marker.color.g = rand() / double(RAND_MAX);
    marker.color.b = rand() / double(RAND_MAX);
    marker.color.a = 1.0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    return marker;
}

// main callback function
void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert from sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2 -> pcl::PointCloud<PointXYZ>
    pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*cloud_msg, *pcl_pc2);
    pcl::fromPCLPointCloud2(*pcl_pc2, *cloud);

    // **CLUSTERING**
    // Link: https://pcl.readthedocs.io/en/latest/cluster_extraction.html

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_clusters;
    for (const pcl::PointIndices& point_idx : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
        for (const int& idx : point_idx.indices) {
            cloud_cluster.push_back((*cloud)[idx]);
        }
        cloud_clusters.push_back(cloud_cluster);
    }

    // **PUBLISHING**
    visualization_msgs::MarkerArray marker_array;
    std::vector<geometry_msgs::Pose> centroids;

    for (int cluster_idx = 0; cluster_idx < cloud_clusters.size(); cluster_idx++) {
        // Marker Cluster
        std::vector<geometry_msgs::Point> marker_cluster;
        for (const pcl::PointXYZ& point : cloud_clusters[cluster_idx]) {
            geometry_msgs::Point marker_point;
            marker_point.x = point.x;
            marker_point.y = point.y;
            marker_point.z = point.z;
            marker_cluster.push_back(marker_point);
        }
        marker_array.markers.push_back(makeMarkers(marker_cluster, cluster_idx, cloud_msg->header));

        // Centroid
        Eigen::Vector4f centroid_eigen;
        pcl::compute3DCentroid(cloud_clusters[cluster_idx], centroid_eigen);

        geometry_msgs::Pose centroid;
        centroid.position.x = centroid_eigen[0];
        centroid.position.y = centroid_eigen[1];
        centroid.position.z = centroid_eigen[2];
        centroid.orientation.w = 1;
        centroids.push_back(centroid);
    }

    marker_pub.publish(marker_array);

    geometry_msgs::PoseArray poses;
    poses.header = cloud_msg->header;
    poses.poses = centroids;
    centroid_pub.publish(poses);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "opponent_detection");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    nhp.getParam("cluster_tolerance", cluster_tolerance_);
    nhp.getParam("min_cluster_size", min_cluster_size_);
    nhp.getParam("max_cluster_size", max_cluster_size_);

    std::string input_cloud, output_markers, output_centroids;
    nhp.getParam("input_cloud", input_cloud);
    nhp.getParam("output_markers", output_markers);
    nhp.getParam("output_centroids", output_centroids);

    ros::Subscriber sub = nh.subscribe(input_cloud, 1, &callback);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>(output_markers, 1);
    centroid_pub = nh.advertise<geometry_msgs::PoseArray>(output_centroids, 1);

    ros::spin();
    return 0;
}
