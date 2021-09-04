//
// Created by charlie on 6/10/21.
//

#ifndef RR_EVGP_CONE_CONNECTION_NODE_H
#define RR_EVGP_CONE_CONNECTION_NODE_H

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl/impl/point_types.hpp>

namespace std {
template <>
struct hash<pcl::PointXYZ> {
    std::size_t operator()(const pcl::PointXYZ &pointXyz) const {
        std::hash<float> hash_i;
        return (std::size_t)(31 * hash_i(pointXyz.x) + 31 * hash_i(pointXyz.y) + 31 * hash_i(pointXyz.z));
    };
};  // namespace

template <>
struct equal_to<pcl::PointXYZ> {
    constexpr bool operator()(const pcl::PointXYZ &lhs, const pcl::PointXYZ &rhs) const {
        return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
    }
};
}  // namespace std

class ConeConnection {
  private:
    class Node {
      public:
        geometry_msgs::Pose value;
        Node *next;
        Node *prev;
    };

    class LinkedList {
      public:
        Node *head;
        int size;
    };

    struct GraphNode {
        geometry_msgs::Pose value;
        std::vector<GraphNode> children;
    };

    struct Graph {
        GraphNode *head;
        int size;
    };

    struct GridPosition {
        int row;
        int col;
    };

    std::vector<tf::Pose> points;

    double max_x_;
    double max_y_;
    double min_x_;
    double min_y_;
    int distance_between_walls;
    int distance_between_cones;

    std::vector<LinkedList> walls_;

    ros::Publisher pub_walls;
    ros::Publisher cone_connection_status;
    ros::Publisher image;
    nav_msgs::MapMetaData mapMetaData;
    ros::Subscriber global_costmap_;
    ros::Subscriber cones_subscriber;
    std::unique_ptr<tf::TransformListener> listener;
    ros::Publisher distance_map_pub;

    double cluster_tolerance_;
    int min_cluster_size_, max_cluster_size_;

    static void updateMap(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    static void clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, nav_msgs::OccupancyGrid &occupancyGrid);

    static void bsline(GridPosition start, GridPosition end, nav_msgs::OccupancyGrid &grid);

    ConeConnection();

  public:
    ~ConeConnection();
};
#endif  // RR_EVGP_CONE_CONNECTION_NODE_H
