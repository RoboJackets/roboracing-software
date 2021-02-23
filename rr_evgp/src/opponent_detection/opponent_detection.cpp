#include <rr_evgp/opponent_detection.h>

// adds markers to marker array
void addMarkers(std::vector<geometry_msgs::Point> markers, int id) {
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration();
    marker.header.frame_id = "lidar";

    marker.ns = "marked_clusters";
    marker.id = id;

    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.points = markers;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.color.a = 1.0;
    marker.color.r = rand() % 256;
    marker.color.g = rand() % 256;
    marker.color.b = rand() % 256;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker_array.markers.push_back(marker);
}

// main callback function
void callback(sensor_msgs::PointCloud2 cloud_msg) {
    // initialize PCLPointCloud2 object
    pcl::PCLPointCloud2::Ptr pcl_cloud2(new pcl::PCLPointCloud2);

    // convert cloud_msg to PCLPointCloud2 type
    pcl_conversions::toPCL(cloud_msg, *pcl_cloud2);

    // initialize PointCloud of PointXYZ objects
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // convert cloud_msg from PCLPointCloud2 to PointCloud of PointXYZ objects
    pcl::fromPCLPointCloud2(*pcl_cloud2, *xyz_cloud);

    // **GROUND SEGMENTATION**

    // initialize another PC of PointXYZ objects to hold the passthrough filter results
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_segmented(new pcl::PointCloud<pcl::PointXYZ>);

    // passthrough filter to segment out ground
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(xyz_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(low_passthru_lim, high_passthru_lim);
    pass.filter(*ground_segmented);

    // **CLUSTERING**

    // creating KdTree object for extracting clusters
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(ground_segmented);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(ground_segmented);
    ec.extract(cluster_indices);

    // **PUBLISHING**

    // holding containers for PCs and Markers
    std::vector<geometry_msgs::Point> marker_cluster = {};
    geometry_msgs::Point marker_point;

    // cluster count
    int cluster_ct = 0;

    // for each PointIndices object, turn it into a PointCloud of PointXYZ objects
    for (const pcl::PointIndices& point_idx : cluster_indices) {
        for (const int& point : point_idx.indices) {
            marker_point.x = (*ground_segmented)[point].x;
            marker_point.y = (*ground_segmented)[point].y;
            marker_point.z = (*ground_segmented)[point].z;

            marker_cluster.push_back(marker_point);
        }

        addMarkers(marker_cluster, cluster_ct);
        cluster_ct++;

        marker_cluster.clear();
    }

    marker_pub.publish(marker_array);
    marker_array = {};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "opponent_detection");

    ros::NodeHandle nh;

    ros::NodeHandle nhp("~");

    nhp.getParam("low_passthru_lim", low_passthru_lim);
    nhp.getParam("high_passthru_lim", high_passthru_lim);

    nhp.getParam("cluster_tolerance", cluster_tolerance);
    nhp.getParam("min_cluster_size", min_cluster_size);
    nhp.getParam("max_cluster_size", max_cluster_size);

    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, &callback);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/marker_array", 1);

    ros::spin();
    return 0;
}
