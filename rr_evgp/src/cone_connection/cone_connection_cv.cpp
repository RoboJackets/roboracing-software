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
    walls_ = linkWalls(cone_positions);
}

int ConeConnectionCv::comparePoses(geometry_msgs::Pose &first, geometry_msgs::Pose &second) {
    return std::floor((second.position.x - first.position.x) + (second.position.y - first.position.y) +
                      (second.position.z - first.position.z));
}

/**
 * Group all of the points into lines. There is no limit to the number of lines
 * that can be created. A line will be extended as long as there are points close
 * to it.
 * @param cone_positions
 * @return a vector of all of the lines. A line is represented by a vector
 * of geometry_msgs::Pose
 */
std::vector<ConeConnectionCv::LinkedList> ConeConnectionCv::linkWalls(const geometry_msgs::PoseArray &cone_positions) {
    std::vector<LinkedList> localWalls;
    std::queue<Node *> cone_queue;

    std::vector<geometry_msgs::Pose> cone_poses = cone_positions.poses;

    bool max_x_init, min_x_init;
    bool max_y_init, min_y_init;

    while (!cone_poses.empty()) {
        if (!cone_queue.empty()) {
            Node *curr = cone_queue.front();
            cone_queue.pop();

            for (auto cone = cone_poses.begin(); cone < cone_poses.end(); cone++) {
                if (distance(cone->position, curr->value.position) <= distance_between_cones) {
                    Node *cone_to_add;

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
                if (max_x_init || (cone->position.x > max_x_)) {
                    max_x_init = true;
                    max_x_ = cone->position.x;
                }
                if (min_x_init || (cone->position.x < min_x_)) {
                    min_x_init = true;
                    min_x_ = cone->position.x;
                }
                if (min_y_init || (cone->position.y < min_y_)) {
                    min_y_init = true;
                    min_y_ = cone->position.y;
                }
                if (max_y_init || (cone->position.y < min_y_)) {
                    max_y_init = true;
                    max_y_ = cone->position.y;
                }
            }
        } else {  // Create new wall because there are no more points close to points in the previous wall
            Node *new_cone = new Node{ cone_poses.back(), nullptr, nullptr };
            cone_queue.push(new_cone);
            localWalls.push_back(LinkedList{ new_cone, 1 });
            cone_poses.pop_back();
        }
    }

    // Convert linked list to vector, delete Node classes
    //    std::vector<std::vector<geometry_msgs::Pose>> walls;
    //    for (LinkedList wall : localWalls) {
    //        geometry_msgs::Pose wall_array_to_return[wall.size];
    //
    //        Node *curr = wall.head;
    //        for (int i = 0; i < wall.size; i++) {
    //            wall_array_to_return[i] = curr->value;
    //            curr = curr->next;
    //            if (curr && curr->prev) {
    //                delete curr->prev;
    //            }
    //        }
    //
    //        std::vector<geometry_msgs::Pose> wall_to_return;
    //        wall_to_return.insert(wall_to_return.cend(), &(wall_array_to_return[0]),
    //        &(wall_array_to_return[wall.size])); walls.push_back(wall_to_return);
    //    }
    return localWalls;
}

void ConeConnectionCv::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                    double *max_x, double *max_y) {
    if (!enabled_) {
        return;
    }
    *min_x = std::min(*min_x, min_x_);
    *min_y = std::min(*min_y, min_y_);
    *max_x = std::max(*max_x, max_x_);
    *max_y = std::max(*max_y, max_y_);
}

void ConeConnectionCv::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
    if (!enabled_) {
        return;
    }
    for (ConeConnectionCv::LinkedList wall : walls_) {
        Node *curr = wall.head;
        while (curr->next) {
            int x, y, x2, y2;

            x = static_cast<int>(curr->value.position.x);
            y = static_cast<int>(curr->value.position.y);
            x2 = static_cast<int>(curr->next->value.position.x);
            y2 = static_cast<int>(curr->next->value.position.y);

            int dx, dy, p;
            dx = x2 - x;
            dy = y2 - y;
            p = 2 * (dy) - (dx);
            while (x <= x2) {
                if (p < 0) {
                    x = x + 1;
                    y = y;
                    p = p + 2 * (dy);
                } else {
                    x = x + 1;
                    y = y + 1;
                    p = p + 2 * (dy - dx);
                }
                master_grid.setCost(x,y, costmap_2d::LETHAL_OBSTACLE);
            }
        }
    }
}

// Expose this layer to the costmap2d configuration
PLUGINLIB_EXPORT_CLASS(ConeConnectionCv, costmap_2d::Layer);
