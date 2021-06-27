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

void ConeConnectionCv::onInitialize() {
    // Setup dynamic reconfigure
    dsrv_ = std::make_unique<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>();
    dsrv_->setCallback([this](auto genericPluginConfig, auto level) { reconfigureCB(genericPluginConfig); });

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
    cones_subscriber = nh.subscribe(cones_topic, 1, &ConeConnectionCv::updateMap, this);
}

static inline double distance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) {
    return pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2) + pow((p2.z - p1.z), 2);
}

void ConeConnectionCv::updateMap(const geometry_msgs::PoseArray &cone_positions) {
    std::vector<std::vector<geometry_msgs::Pose>> walls = linkWalls(cone_positions);
}

int ConeConnectionCv::comparePoses(geometry_msgs::Pose &first, geometry_msgs::Pose &second) {
    return std::floor((second.position.x - first.position.x) + (second.position.y - first.position.y) +
                      (second.position.z - first.position.z));
}

std::vector<std::vector<geometry_msgs::Pose>>
ConeConnectionCv::linkWalls(const geometry_msgs::PoseArray &cone_positions) const {
    std::vector<LinkedList> localWalls;
    std::queue<Node *> cone_queue;

    std::vector<geometry_msgs::Pose> cone_poses = cone_positions.poses;
    while (!cone_poses.empty()) {
        if (!cone_queue.empty()) {
            Node *curr = cone_queue.front();
            cone_queue.pop();

            for (auto cone = cone_poses.begin(); cone < cone_poses.end(); cone++) {
                if (distance(cone->position, curr->value.position) <= distance_between_cones) {
                    Node *cone_to_add = nullptr;

                    // Add new Node. Must maintain the head ptr when added before
                    if (comparePoses(curr->value, *cone)) {
                        cone_to_add = new Node{ *cone, curr, curr->prev };
                        curr->prev = cone_to_add;
                        localWalls.back().head = cone_to_add;
                    } else {
                        cone_to_add = new Node{ *cone, curr->next, curr };
                        curr->next = cone_to_add;
                    }
                    localWalls.back().size++;
                    cone_queue.push(cone_to_add);
                    cone_poses.erase(cone--);
                }
            }
        } else {  // Create new wall, there are no more points close to points in previous wall
            Node *new_cone = new Node{ cone_poses.back(), nullptr, nullptr };
            cone_queue.push(new_cone);
            localWalls.push_back(LinkedList{ new_cone, 1 });
            cone_poses.pop_back();
        }
    }

    // Convert linked list to vector, delete Node classes
    std::vector<std::vector<geometry_msgs::Pose>> walls;
    for (LinkedList wall : localWalls) {
        geometry_msgs::Pose wall_array_to_return[wall.size];

        Node *curr = wall.head;
        for (int i = 0; i < wall.size; i++) {
            wall_array_to_return[i] = curr->value;
            curr = curr->next;
            if (curr && curr->prev) {
                delete curr->prev;
            }
        }

        std::vector<geometry_msgs::Pose> wall_to_return;
        wall_to_return.insert(wall_to_return.cend(), &(wall_array_to_return[0]), &(wall_array_to_return[wall.size]));
        walls.push_back(wall_to_return);
    }
    return walls;
}

void ConeConnectionCv::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                    double *max_x, double *max_y) {
    if (!enabled_) {
        return;
    }
    Layer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ConeConnectionCv::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
    if (!enabled_) {
        return;
    }
    Layer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
}

// Expose this layer to the costmap2d configuration
PLUGINLIB_EXPORT_CLASS(ConeConnectionCv, costmap_2d::Layer);
