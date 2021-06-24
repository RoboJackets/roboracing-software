//
// Created by charlie on 6/10/21.
//

#ifndef RR_EVGP_CONE_CONNECTION_CV_H
#define RR_EVGP_CONE_CONNECTION_CV_H

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <tf/transform_listener.h>

class ConeConnectionCv : public costmap_2d::Layer {
  private:
    double init_robot_x;
    double init_robot_y;
    double init_robot_yaw;
    double curr_robot_x;
    double curr_robot_y;
    double mark_x_, mark_y_;

    std::vector<tf::Pose> points;
    std::vector<std::vector<geometry_msgs::Pose>> walls;

    ros::Publisher pub_walls;
    int distance_between_walls;
    int distance_between_cones;
    ros::Publisher image;
    nav_msgs::MapMetaData mapMetaData;
    ros::Subscriber map_sub;
    [[maybe_unused]] ros::Subscriber cones_subscriber;
    std::unique_ptr<tf::TransformListener> listener;
    ros::Publisher distance_map_pub;

    void create_image();

    //    std::pair<unsigned int, unsigned int> PoseToGridPosition(const rr::Pose&);
    //    void SetMapMessage(const boost::shared_ptr<const nav_msgs::OccupancyGrid>& map_msg);

    inline void reconfigureCB(costmap_2d::GenericPluginConfig& config) {
        enabled_ = config.enabled;
    }

    void updateMap(const geometry_msgs::PoseArray &cone_positions);

    std::unique_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>> dsrv_;

  public:
    void onInitialize() override;
    void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                      double* max_y) override;
    void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
};

#endif  // RR_EVGP_CONE_CONNECTION_CV_H
