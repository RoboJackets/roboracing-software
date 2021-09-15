#include "ros/ros.h"
#include <parameter_assertions/assertions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <unordered_set>
#include <algorithm>
#include <geometry_msgs/PoseArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub_walls;
double cone_distance;
nav_msgs::OccupancyGrid occupancyGrid;

double distance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) {
    return pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2);
}

double distance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2) {
    return pow((p2.position.x - p1.position.x), 2) + pow((p2.position.y - p1.position.y), 2);
}

std::pair<double, double> pos2cell(const pcl::PointXYZ &point) {
    int r = std::clamp((-occupancyGrid.info.origin.position.y + point.y) / occupancyGrid.info.resolution, 0.0, (double) occupancyGrid.info.height-1);
    int c = std::clamp((-occupancyGrid.info.origin.position.x + point.x) / occupancyGrid.info.resolution, 0.0, (double) occupancyGrid.info.width-1);
    return std::make_pair(r, c);
}

std::pair<double, double> pos2cell(const geometry_msgs::Pose &point) {
    int r = std::clamp((-occupancyGrid.info.origin.position.y + point.position.y) / occupancyGrid.info.resolution, 0.0, (double) occupancyGrid.info.height-1);
    int c = std::clamp((-occupancyGrid.info.origin.position.x + point.position.x) / occupancyGrid.info.resolution, 0.0, (double) occupancyGrid.info.width-1);
    return std::make_pair(r, c);
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
        occupancyGrid.data[round(X) * occupancyGrid.info.width + round(Y)] = 126;  // put pixel at (X,Y)
        X += Xinc;  // increment in x at each step
        Y += Yinc;  // increment in y at each step
    }
}

void createMap(const geometry_msgs::PoseArrayConstPtr &array_msg) {
    occupancyGrid.data = std::vector<signed char>(occupancyGrid.info.height * occupancyGrid.info.width);

    int n = array_msg->poses.size();
    for (int i = 0; i < n; i++) {
        auto[r0, c0] = pos2cell(array_msg->poses[i]);
        int idx0 = r0 * occupancyGrid.info.width + c0;
        if (occupancyGrid.data[idx0] == 100) {
            continue;
        }
        occupancyGrid.data[idx0] = 100;
        // occupancyGrid.data
        for (int j = i + 1; j < n; j++) {
            // double dist = ;
            auto[r1, c1] = pos2cell(array_msg->poses[j]);
            int idx1 = r1 * occupancyGrid.info.width + c1;
            if (occupancyGrid.data[idx1] != 100 && distance(array_msg->poses[i], array_msg->poses[j]) < cone_distance) {
                DDA(r0, c0, r1, c1, occupancyGrid);
            }
        }
    }

    occupancyGrid.header = array_msg->header;
    pub_walls.publish(occupancyGrid);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "cone_connection_node");
    ros::NodeHandle nhp("~");

    assertions::getParam(nhp, "cone_distance", cone_distance);

    std::string walls_topic, cones_topic;
    assertions::getParam(nhp, "walls_topic", walls_topic);
    assertions::getParam(nhp, "cones_topic", cones_topic);

    double map_resolution;
    int map_size;
    assertions::getParam(nhp, "map_resolution", map_resolution);
    assertions::getParam(nhp, "map_size", map_size);

    pub_walls = nhp.advertise<nav_msgs::OccupancyGrid>(walls_topic, 1);
    ros::Subscriber cones_subscriber = nhp.subscribe(cones_topic, 1, createMap);

    // Initialize occupancy grid
    occupancyGrid.info.resolution = map_resolution;
    occupancyGrid.info.height = map_size;
    occupancyGrid.info.width = map_size;

    geometry_msgs::Pose origin;
    origin.position.y = -map_size * map_resolution / 2.0;  // m / cell;
    origin.position.x = -map_size * map_resolution / 2.0;
    occupancyGrid.info.origin = origin;

    ros::spin();
    return 0;
}
