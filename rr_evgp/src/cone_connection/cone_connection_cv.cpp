#include "cone_connection_cv.h"

#include <nav_msgs/OccupancyGrid.h>
#include <parameter_assertions/assertions.h>
#include <pluginlib/class_list_macros.h>

#include "opencv2/core.hpp"
#include "ros/ros.h"

/**
 * @author Charles Jenkins
 * Helpful guide to understand this file: http://wiki.ros.org/costmap_2d/Tutorials/Creating%20a%20New%20Layer
 */

//int main(int argc, char** argv) {
//    ros::init(argc, argv, "cone_connection_cv");
//    ros::NodeHandle nh;
//    ros::NodeHandle nhp("~");
//
//    //    nh.advertise<cv::Image>(output_centroids, 1);
//
//    ConeConnectionCv connection(nh);
//
//    ros::spin();
//    return 0;
//}
//
//double ConeConnectionCv::DistanceCost(const rr::Pose& pose) {
//    auto [mx, my] = this->PoseToGridPosition(pose);
//
//    if (my < 0 || mapMetaData.height <= my || mx < 0 || mapMetaData.width <= mx)
//        return 0.0;
//
//    return distance_cost_map.at<float>(my, mx);
//}
//C
//void ConeConnectionCv::SetMapMessage(const boost::shared_ptr<nav_msgs::OccupancyGrid const>& map_msg) {
//    try {
//        listener->waitForTransform(map_msg->header.frame_id, robot_base_frame, ros::Time(0), ros::Duration(.05));
//        listener->lookupTransform(map_msg->header.frame_id, robot_base_frame, ros::Time(0), transform);
//    } catch (tf::TransformException& ex) {
//        ROS_ERROR_STREAM(ex.what());
//    }
//}
//
//std::pair<unsigned int, unsigned int> ConeConnectionCv::PoseToGridPosition(const rr::Pose& pose) {
//    tf::Pose w_pose = transform * tf::Pose(tf::createQuaternionFromYaw(0), tf::Vector3(pose.x, pose.y, 0));
//
//    unsigned int mx = std::floor((w_pose.getOrigin().x() - mapMetaData.origin.position.x) / mapMetaData.resolution);
//    unsigned int my = std::floor((w_pose.getOrigin().y() - mapMetaData.origin.position.y) / mapMetaData.resolution);
//
//    return std::make_pair(mx, my);
//}
//
//void ConeConnectionCv::create_image() {}
//
//ConeConnectionCv::ConeConnectionCv(ros::NodeHandle nh) : listener(new tf::TransformListener) {
//    std::string map_topic;
//    assertions::getParam(nh, "map_topic", map_topic);
//    map_sub = nh.subscribe(map_topic, 1, &ConeConnectionCv::SetMapMessage, this);
//
//    distance_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("distance_map", 1);
//}

void ConeConnectionCv::onInitialize() {
    // Setup dynamic reconfigure
    dsrv_ = std::make_unique<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>();
    dsrv_->setCallback([this](auto genericPluginConfig, auto level) {
        reconfigureCB(genericPluginConfig);
    });

    // Initial variables
    init_robot_x = 0.0;
    init_robot_y = 0.0;
    init_robot_yaw = 0.0;

    // Initialize parameters
    ros::NodeHandle nh;
    ros::NodeHandle costmap_nh("~costmap");
    ros::NodeHandle private_nh("~" + getName());

    std::string walls_topic;
    assertions::param(private_nh, "walls_topic", walls_topic, std::string("/global_walls_topic"));
    pub_walls = nh.advertise<nav_msgs::OccupancyGrid>(walls_topic, 1);

    assertions::param(private_nh, "distance_between_walls", distance_between_walls, 18);
    assertions::param(private_nh, "distance_between_cones", distance_between_cones, distance_between_walls / 3);

    std::string cones_topic;
    assertions::param(private_nh, "cones_topic", cones_topic, std::string("/cones_topic"));
    cones_subscriber = nh.subscribe(cones_topic, 1, &ConeConnectionCv::update_map, this);
}

void ConeConnectionCv::update_map(const geometry_msgs::PoseArray &cone_positions) {

}

void ConeConnectionCv::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                    double *max_x, double *max_y) {
    Layer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ConeConnectionCv::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
    Layer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
}

// Expose this layer to the costmap2d configuration
PLUGINLIB_EXPORT_CLASS(ConeConnectionCv, costmap_2d::Layer);
