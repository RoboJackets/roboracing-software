#include "cone_connection_node.h"

#include <nav_msgs/OccupancyGrid.h>
#include <parameter_assertions/assertions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <pcl/impl/point_types.hpp>
#include <unordered_set>

#include "opencv2/core.hpp"
#include "ros/ros.h"

/**
 * @author Charles Jenkins
 * Helpful guide to understand this file: http://wiki.ros.org/costmap_2d/Tutorials/Creating%20a%20New%20Layer
 */

ConeConnection::ConeConnection() {
    // Initialize parameters
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Initialize class variables
    max_x_ = 0;
    max_y_ = 0;
    min_x_ = 0;
    min_y_ = 0;
    distance_between_walls = 18;
    distance_between_cones = distance_between_walls / 3;
    cluster_tolerance_ = 0.5;
    min_cluster_size_ = 1;
    max_cluster_size_ = 10;

    std::string walls_topic;
    assertions::param(private_nh, "walls_topic", walls_topic, std::string("/global_walls_topic"));
    pub_walls = nh.advertise<nav_msgs::OccupancyGrid>(walls_topic, 1);

    std::string cone_connection_status_topic;
    assertions::param(private_nh, "cone_connection_status", cone_connection_status_topic,
                      std::string("/cone_connection_status"));
    cone_connection_status = nh.advertise<std_msgs::String>(cone_connection_status_topic, 1);

    assertions::param(private_nh, "distance_between_walls", distance_between_walls, 18);
    assertions::param(private_nh, "distance_between_cones", distance_between_cones, distance_between_walls / 3);

    assertions::param(private_nh, "cluster_tolerance", cluster_tolerance_, 0.5);
    assertions::param(private_nh, "min_cluster_size", min_cluster_size_, 1);
    assertions::param(private_nh, "max_cluster_size", max_cluster_size_, 10);

    std::string cones_topic;
    assertions::param(private_nh, "cones_topic", cones_topic, std::string("/cones_topic"));
    cones_subscriber = nh.subscribe(cones_topic, 1, &ConeConnection::updateMap);
}

static inline double distance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) {
    return pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2) + pow((p2.z - p1.z), 2);
}

// http://mytechnotrick.blogspot.com/2015/07/c-program-to-implement-bresenhams-line.html
/**
 * Use breshnam's line algorithm to draw points onto the occupancy grid
 * @param start Start GridPosition for line
 * @param end End GridPosition for line
 * @param grid OccupancyGrid to place points
 */
void ConeConnection::bsline(GridPosition start, GridPosition end, nav_msgs::OccupancyGrid &grid) {
    int column_diff, row_diff, p;
    column_diff = end.col - start.col;
    row_diff = end.row - start.row;
    p = 2 * (row_diff) - (column_diff);
    while (start.col <= end.col) {
        if (p < 0) {
            start.col = start.col + 1;
            start.row = start.row;
            p = p + 2 * (row_diff);
        } else {
            start.col = start.col + 1;
            start.row = start.row + 1;
            p = p + 2 * (row_diff - column_diff);
        }

        // Set deadly object at provided row/column
        grid.data[start.row * grid.info.width + start.col] = 100;
    }
}

void ConeConnection::clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                nav_msgs::OccupancyGrid &occupancyGrid) {
    // **CLUSTERING**
    // Link: https://pcl.readthedocs.io/en/latest/cluster_extraction.html
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // Add all points to the tree
    tree->setInputCloud(cloud);
    // visited set

    const double pnt_origin_x = .5 * (occupancyGrid.info.height);
    const double pnt_origin_y = .5 * (occupancyGrid.info.width);
    //  col = int((pnt.y - pnt_origin) / resolution)
    const int K = 4;

    std::unordered_set<pcl::PointXYZ *> visited;
    std::queue<pcl::PointXYZ *> queue;
    queue.push(&cloud->at(0));
    visited.insert(&cloud->at(0));

    auto occupancyPosition = [&](const pcl::PointXYZ &point) {
        GridPosition ret{ .row = int((point.x - pnt_origin_x) / occupancyGrid.info.resolution),
                          .col = int((point.y - pnt_origin_y) / occupancyGrid.info.resolution) };
        return ret;
    };

    while (!queue.empty()) {
        pcl::PointXYZ *curr = queue.front();                   // Grab value from queue
        GridPosition currPosition = occupancyPosition(*curr);  // Get position of curr on graph
        queue.pop();                                           // Remove value from queue

        std::vector<int> nearbyPoints(K);
        std::vector<float> nearbyPointsSquaredDistance(K);
        int close = tree->nearestKSearch(*curr, K, nearbyPoints, nearbyPointsSquaredDistance);

        // Is a cone that should be added
        if (close > 0) {
            for (auto nearby = nearbyPoints.begin(); nearby < nearbyPoints.end(); nearby++) {
                pcl::PointXYZ *c = &cloud->at(*nearby.base());
                if (visited.find(c) != visited.end()) {
                    // Draw line on occupancy grid to each neighbor
                    bsline(occupancyPosition(*c), currPosition, occupancyGrid);
                    // Add neighbors to visited and queue
                    visited.insert(c);
                    queue.push(c);
                }
            }
        }
    }
}

// main callback function
void ConeConnection::updateMap(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert from sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2 -> pcl::PointCloud<PointXYZ>
    pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*cloud_msg, *pcl_pc2);
    pcl::fromPCLPointCloud2(*pcl_pc2, *cloud);

//    std::vector<geometry_msgs::Pose> centroids;
//    walls_ = linkWalls(centroids);

    // Initialize occupancy grid
    nav_msgs::OccupancyGrid occupancyGrid;
    occupancyGrid.info.height = 350;
    occupancyGrid.info.width = 350;
    clustering(cloud, occupancyGrid);


}