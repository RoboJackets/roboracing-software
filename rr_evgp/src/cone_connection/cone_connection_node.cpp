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

    assertions::param(pnh, "distance_between_walls", distance_between_walls, 15);
    assertions::param(pnh, "distance_between_cones", distance_between_cones, distance_between_walls / 3);

    std::string cones_topic;
    assertions::param(pnh, "cones_topic", cones_topic, std::string("/cones_topic"));
    cones_subscriber = nh.subscribe(cones_topic, 1, &ConeConnection::updateMap, this);
}

static inline double distance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) {
    return pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2) + pow((p2.z - p1.z), 2);
}

int abs(int n) {
    return ((n > 0) ? n : (n * (-1)));
}

// DDA Function for line generation
void DDA(int X0, int Y0, int X1, int Y1, nav_msgs::OccupancyGrid &occupancyGrid) {
    // calculate dx & dy
    int dx = X1 - X0;
    int dy = Y1 - Y0;

    // calculate steps required for generating pixels
    int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);

    // calculate increment in x & y for each steps
    float Xinc = dx / (float)steps;
    float Yinc = dy / (float)steps;

    // Put pixel for each step
    float X = X0;
    float Y = Y0;
    for (int i = 0; i <= steps; i++) {
        occupancyGrid.data[floor(X) * occupancyGrid.info.width + floor(Y)] = 126;  // put pixel at (X,Y)
        X += Xinc;                                                                 // increment in x at each step
        Y += Yinc;                                                                 // increment in y at each step
    }
}

void ConeConnection::clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                nav_msgs::OccupancyGrid &occupancyGrid, const int radius) {
    // **CLUSTERING**
    // Link: https://pcl.readthedocs.io/en/latest/cluster_extraction.html
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // Add all points to the tree
    tree->setInputCloud(cloud);

    int car_r = (int) (.5 * (occupancyGrid.info.height));
    int car_c = (int) (.5 * (occupancyGrid.info.width));
    double car_x = (double) car_r * occupancyGrid.info.resolution;  // m / cell
    double car_y = (double) car_c * occupancyGrid.info.resolution;

    std::unordered_set<pcl::PointXYZ> unvisited;
    std::queue<pcl::PointXYZ> queue;
    queue.push(cloud->at(0));

    for (pcl::PointXYZ point : cloud->points) {
        unvisited.insert(point);
    }

    //    at 5,5, lidar at -1,1 from car
    //    so lidar 4,6
    auto occupancyPosition = [&](const pcl::PointXYZ &point) {
        int r = (int)std::clamp((car_y + point.y) / occupancyGrid.info.resolution, 0.0,
                                (double)occupancyGrid.info.height);
        int c =
              (int)std::clamp((car_x + point.x) / occupancyGrid.info.resolution, 0.0, (double)occupancyGrid.info.width);
        GridPosition ret{ .row = r, .col = c };
        return ret;
    };

    while (!unvisited.empty()) {
        queue.push(*unvisited.begin());
        unvisited.erase(queue.back());
        while (!queue.empty()) {
            pcl::PointXYZ curr = queue.front();                   // Grab value from queue
            GridPosition currPosition = occupancyPosition(curr);  // Get position of curr on graph
            queue.pop();                                          // Remove value from queue

            std::vector<int> nearbyPoints;
            std::vector<float> nearbyPointsSquaredDistance;
            int close = tree->radiusSearch(curr, radius, nearbyPoints, nearbyPointsSquaredDistance, 100);

            // Is a cone that should be added
            if (close > 0) {
                int addedToQueue = 0;
                for (auto nearby : nearbyPoints) {
                    pcl::PointXYZ c = cloud->at(nearby);
                    if (unvisited.find(c) != unvisited.end()) {
                        GridPosition start = occupancyPosition(c);
                        if ((start.col > car_c + 1 || start.col < car_c - 1) &&
                            (start.row > car_r + 1 || start.row < car_r - 1)) {
                            // Draw line on occupancy grid to each neighbor
                            DDA(start.row, start.col, currPosition.row, currPosition.col, occupancyGrid);
                            // Add neighbors to visited and queue (only if outside of car footprint)
                            queue.push(c);
                        }
                        unvisited.erase(c);
                    }
                }
            }
        }
    }
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
    // info.resolution scales grid size to the provided meters
    occupancyGrid.info.resolution = 0.25;
    occupancyGrid.info.height = 350;
    occupancyGrid.info.width = 350;
    geometry_msgs::Pose origin;
    double car_x = -.5 * occupancyGrid.info.height * occupancyGrid.info.resolution;  // m / cell
    double car_y = -.5 * occupancyGrid.info.width * occupancyGrid.info.resolution;

    origin.position.x = car_x;
    origin.position.y = car_y;

    occupancyGrid.info.origin = origin;
    occupancyGrid.header.frame_id = "base_footprint";

    std::vector<signed char> grid(occupancyGrid.info.height * occupancyGrid.info.width);
    occupancyGrid.data = grid;
    clustering(cloud, occupancyGrid, static_cast<int>(distance_between_cones * 1.15));

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
