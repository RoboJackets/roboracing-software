#include "cone_connection_node.h"

#include <parameter_assertions/assertions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/String.h>

#include <random>
#include <unordered_set>

#include "opencv2/core.hpp"
#include "ros/ros.h"

ConeConnection::ConeConnection(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
    // Initialize class variables
    distance_between_walls = 18;
    distance_between_cones = distance_between_walls / 3;

    std::string walls_topic;
    assertions::param(pnh, "walls_topic", walls_topic, std::string("/global_walls_topic"));
    pub_walls = nh.advertise<nav_msgs::OccupancyGrid>(walls_topic, 1);

    std::string cone_connection_status_topic;
    assertions::param(pnh, "cone_connection_status", cone_connection_status_topic,
                      std::string("/cone_connection_status"));
    cone_connection_status = nh.advertise<std_msgs::String>(cone_connection_status_topic, 1);

    assertions::param(pnh, "distance_between_walls", distance_between_walls, 18);
    assertions::param(pnh, "distance_between_cones", distance_between_cones, distance_between_walls / 3);

    std::string cones_topic;
    assertions::param(pnh, "cones_topic", cones_topic, std::string("/cones_topic"));
    cones_subscriber = nh.subscribe(cones_topic, 1, &ConeConnection::updateMap, this);
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
                                nav_msgs::OccupancyGrid &occupancyGrid, const int radius) {
    static int id = 0;
    id++;
    // **CLUSTERING**
    // Link: https://pcl.readthedocs.io/en/latest/cluster_extraction.html
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // Add all points to the tree
    tree->setInputCloud(cloud);

    const double pnt_origin_x = .5 * (occupancyGrid.info.height);
    const double pnt_origin_y = .5 * (occupancyGrid.info.width);


    std::unordered_set<pcl::PointXYZ> visited;
    std::queue<pcl::PointXYZ> queue;
    queue.push(cloud->at(0));
    visited.insert(cloud->at(0));

    auto occupancyPosition = [&](const pcl::PointXYZ &point) {
        GridPosition ret{ .row = int((pnt_origin_x)-point.x * occupancyGrid.info.resolution),
                          .col = int((pnt_origin_y)-point.y * occupancyGrid.info.resolution) };
        return ret;
    };

    while (!queue.empty()) {
        std::string queueSize = "Queue Size: " + std::to_string(queue.size());
        ROS_INFO_STREAM(queueSize);
        pcl::PointXYZ curr = queue.front();                   // Grab value from queue
        GridPosition currPosition = occupancyPosition(curr);  // Get position of curr on graph
        queue.pop();                                          // Remove value from queue

        std::vector<int> nearbyPoints;
        std::vector<float> nearbyPointsSquaredDistance;
        int close = tree->radiusSearch(curr, radius, nearbyPoints, nearbyPointsSquaredDistance, 10);

        ROS_INFO_STREAM(std::to_string(id) + " Close points: " + std::to_string(close));
        // Is a cone that should be added
        if (close > 0) {
            int addedToQueue = 0;
            for (auto nearby : nearbyPoints) {
                pcl::PointXYZ c = cloud->at(nearby);
                if (visited.find(c) == visited.end()) {
                    // Draw line on occupancy grid to each neighbor
                    bsline(occupancyPosition(c), currPosition, occupancyGrid);
                    // Add neighbors to visited and queue
                    visited.insert(c);
                    queue.push(c);
                    ROS_INFO_STREAM(std::to_string(id) + " Pushing onto queue: " + std::to_string(++addedToQueue) + " Visited Size: " + std::to_string(visited.size()));
                }
                ROS_INFO_STREAM(std::to_string(id) + " Point: " + std::to_string(c.x) + ", " + std::to_string(c.y) +
                                ", " + std::to_string(c.z));
            }
            ROS_INFO_STREAM(std::to_string(id) + " Finished for loop");
            ROS_INFO_STREAM(std::to_string(id) + " Queue Size: " + std::to_string(queue.size()));
        }
    }
    ROS_INFO_STREAM(std::to_string(id) + " exited");
}

// main callback function
void ConeConnection::updateMap(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    // Convert from sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2 -> pcl::PointCloud<PointXYZ>
    pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*cloud_msg, *pcl_pc2);
    pcl::fromPCLPointCloud2(*pcl_pc2, *cloud);

    // Initialize occupancy grid
    nav_msgs::OccupancyGrid occupancyGrid;
    occupancyGrid.info.height = 350;
    occupancyGrid.info.width = 350;

    // info.resolution scales grid size to the provided meters
    occupancyGrid.info.resolution = 0.25;
    std::vector<signed char> grid(occupancyGrid.info.height * occupancyGrid.info.width);
    occupancyGrid.data = grid;
    clustering(cloud, occupancyGrid, static_cast<int>(distance_between_cones * 1.5));

    std_msgs::String msg;
    msg.data = "Publishing grid";
    cone_connection_status.publish(msg);
    pub_walls.publish(occupancyGrid);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cone_connection_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    ConeConnection coneConnection(nh, nhp);

    ros::spin();
    return 0;
}
