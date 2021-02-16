#include <rr_evgp/opponent_detection.h>

// publishes clustered clouds
void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
    cloud_ptr->header.frame_id = "lidar";

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

// adds markers to array
void addMarkers(std::vector<geometry_msgs::Point> markers, int color) {
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration();
    marker.header.frame_id = "lidar";

    marker.ns = "marked_clusters";
    marker.id = color;

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
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);

    // convert cloud_msg to PCLPointCloud2 type
    pcl_conversions::toPCL(cloud_msg, *cloud);

    // initialize PointCloud of PointXYZ objects
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    // convert cloud_msg from PCLPointCloud2 to PointCloud of PointXYZ objects
    pcl::fromPCLPointCloud2(*cloud, *cloud2);

    // **GROUND SEGMENTATION**

    // initialize another PC of PointXYZ objects to hold the passthrough filter results
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_segmented(new pcl::PointCloud<pcl::PointXYZ>);

    // passthrough filter to segment out ground
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud2);
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
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<geometry_msgs::Point> marker_cluster = {};
    geometry_msgs::Point marker_point;
    bool notWall = true;

    // cluster count
    int cluster_ct = 0;

    // for each PointIndices object, turn it into a PointCloud of PointXYZ objects
    for (const pcl::PointIndices& point_idx : cluster_indices) {
        for (const int& point : point_idx.indices) {
            // cloud_cluster->push_back((*ground_segmented)[point]);

            marker_point.x = (*ground_segmented)[point].x;
            marker_point.y = (*ground_segmented)[point].y;
            marker_point.z = (*ground_segmented)[point].z;

            if (marker_point.x < front_lim && marker_point.x > back_lim && marker_point.y < left_lim && marker_point.y > right_lim) {
                marker_cluster.push_back(marker_point);
            } else {
                notWall = false;
                break;
            }
        }

        /* cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true; */

        if (notWall) {
            addMarkers(marker_cluster, cluster_ct);
            cluster_ct += 1;
        }

        // cloud_cluster->clear();
        marker_cluster.clear();
        notWall = true;
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

    nhp.getParam("front_lim", front_lim);
    nhp.getParam("back_lim", back_lim);
    nhp.getParam("left_lim", left_lim);
    nhp.getParam("right_lim", right_lim);

    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, &callback);
    // cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/clusters", 1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/marker_array", 1);

    ros::spin();
    return 0;
}
