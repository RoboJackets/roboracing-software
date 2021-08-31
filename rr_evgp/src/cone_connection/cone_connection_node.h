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

    void updateMap(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void updateMapGraph(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    std::vector<LinkedList> linkWalls(std::vector<geometry_msgs::Pose> &cone_positions);
    static int comparePoses(geometry_msgs::Pose &first, geometry_msgs::Pose &second);
    std::vector<std::vector<int>> linkWallGraph(const pcl::PointCloud<pcl::PointXYZ>::Ptr &poses) const;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const;

    ConeConnection();

  public:
    ~ConeConnection();
};
#endif  // RR_EVGP_CONE_CONNECTION_NODE_H
