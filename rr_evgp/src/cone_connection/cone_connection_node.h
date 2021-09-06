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
#include <pcl/point_types.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

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
    struct GridPosition {
        int row;
        int col;
    };

    std::vector<tf::Pose> points;

    int distance_between_walls;
    int distance_between_cones;

    ros::Publisher pub_walls;
    ros::Publisher cone_connection_status;
    ros::Subscriber cones_subscriber;
    std::unique_ptr<tf::TransformListener> listener;

    void updateMap(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, nav_msgs::OccupancyGrid &occupancyGrid,
                           int radius);

    static void bsline(GridPosition start, GridPosition end, nav_msgs::OccupancyGrid &grid);

  public:
    ConeConnection(ros::NodeHandle &nh, ros::NodeHandle &pnh);
};
#endif  // RR_EVGP_CONE_CONNECTION_NODE_H
