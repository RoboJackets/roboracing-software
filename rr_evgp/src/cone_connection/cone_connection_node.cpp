#include "cone_connection_node.h"

#include <nav_msgs/OccupancyGrid.h>
#include <parameter_assertions/assertions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <pcl/impl/point_types.hpp>
#include <unordered_set>

#include "opencv2/core.hpp"
#include "ros/ros.h"

/**
 * @author Charles Jenkins
 * Helpful guide to understand this file: http://wiki.ros.org/costmap_2d/Tutorials/Creating%20a%20New%20Layer
 */

ConeConnection::ConeConnection() {
    // Initialize parameters
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Initialize class variables
    max_x_ = 0;
    max_y_ = 0;
    min_x_ = 0;
    min_y_ = 0;
    distance_between_walls = 18;
    distance_between_cones = distance_between_walls / 3;
    cluster_tolerance_ = 0.5;
    min_cluster_size_ = 1;
    max_cluster_size_ = 10;

    std::string walls_topic;
    assertions::param(private_nh, "walls_topic", walls_topic, std::string("/global_walls_topic"));
    pub_walls = nh.advertise<nav_msgs::OccupancyGrid>(walls_topic, 1);

    std::string cone_connection_status_topic;
    assertions::param(private_nh, "cone_connection_status", cone_connection_status_topic,
                      std::string("/cone_connection_status"));
    cone_connection_status = nh.advertise<std_msgs::String>(cone_connection_status_topic, 1);

    assertions::param(private_nh, "distance_between_walls", distance_between_walls, 18);
    assertions::param(private_nh, "distance_between_cones", distance_between_cones, distance_between_walls / 3);

    assertions::param(private_nh, "cluster_tolerance", cluster_tolerance_, 0.5);
    assertions::param(private_nh, "min_cluster_size", min_cluster_size_, 1);
    assertions::param(private_nh, "max_cluster_size", max_cluster_size_, 10);

    //    std::string map_topic;
    //    assertions::getParam(nh, "map_topic", map_topic);
    //    map_sub = nh.subscribe(map_topic, 1, &ConeConnection::SetMapMessage, this);

    std::string cones_topic;
    assertions::param(private_nh, "cones_topic", cones_topic, std::string("/cones_topic"));
    ROS_INFO_STREAM(cones_topic);
    cones_subscriber = nh.subscribe(cones_topic, 1, &ConeConnection::updateMap, this);
}

// void ConeConnection::SetMapMessage(const boost::shared_ptr<nav_msgs::OccupancyGrid const>& map_msg) {
//
// }

static inline double distance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) {
    return pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2) + pow((p2.z - p1.z), 2);
}

std::vector<pcl::PointCloud<pcl::PointXYZ>> ConeConnection::clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const {
    // **CLUSTERING**
    // Link: https://pcl.readthedocs.io/en/latest/cluster_extraction.html
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    // visited set

    std::vector<std::vector<int>> cloudList;
    nav_msgs::OccupancyGrid occupancyGrid;
    occupancyGrid.info.height = 350;
    occupancyGrid.info.width = 350;

//  col = int((pnt.y - pnt_origin) / resolution)
    int K = 4;
    std::vector<int> nearbyPoints(K);
    std::vector<float> nearbyPointsSquaredDistance(K);

    for(auto iter = cloud->begin(); iter < cloud->end(); iter++) {
        int close = tree->nearestKSearch(*iter.base(), K, nearbyPoints, nearbyPointsSquaredDistance);
        if (close > 0) {

        }
    }

//    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//    ec.setClusterTolerance(cluster_tolerance_); // track_width / 3
//    ec.setMinClusterSize(1);
//    ec.setMaxClusterSize(100000);
//    ec.setSearchMethod(tree);
//    ec.setInputCloud(cloud);
//    ec.extract(cluster_indices);
//
//    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
//    for (const pcl::PointIndices &point_idx : cluster_indices) {
//        pcl::PointCloud<pcl::PointXYZ> cluster;
//        for (const int &idx : point_idx.indices) {
//            cluster.push_back((*cloud)[idx]);
//        }
//        clusters.push_back(cluster);
//    }
    return clusters;
}

// main callback function
void ConeConnection::updateMap(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    std_msgs::String msg;
    msg.data = "Updating map";
    cone_connection_status.publish(msg);
    nav_msgs::OccupancyGrid occupancyGrid;

    // Convert from sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2 -> pcl::PointCloud<PointXYZ>
    pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*cloud_msg, *pcl_pc2);
    pcl::fromPCLPointCloud2(*pcl_pc2, *cloud);

    std::vector<geometry_msgs::Pose> centroids;
    walls_ = linkWalls(centroids);
    //    clustering(centroids, cloud);

    //    get points, cluster, find closest, bfs and connected closest, graph to map,
    // opencv img MxM, convert poses to indices based on resolution and map origin,
    //

    std_msgs::String updateCosts;
    msg.data = "Updating costs";
    cone_connection_status.publish(updateCosts);

    // Iterate through all the walls points and draw
    // straight lines connecting all the points on the wall
    for (ConeConnection::LinkedList wall : walls_) {
        Node *curr = wall.head;
        while (curr != nullptr && curr->next != nullptr) {
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
                std_msgs::String usingWalls;
                usingWalls.data = "Using walls: x -> " + std::to_string(x) + ", y -> " + std::to_string(y);
                cone_connection_status.publish(usingWalls);
//              global_costmap_.setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
            }
            curr = curr->next;
        }
    }
}

std::vector<std::vector<int>> ConeConnection::linkWallGraph(const pcl::PointCloud<pcl::PointXYZ>::Ptr& poses) const {
    std::queue<int> queue;
    std::unordered_set<int> visited;
    pcl::PointXYZ curr = poses->front();
    poses->erase(poses->begin());

    while(!poses->empty()) {
        std::vector<std::vector<int>> adj_list(poses->size());
        while(!queue.empty()) {
            int curr_i = queue.front();
            queue.pop();
            curr = poses->at(0, curr_i);

            for (int i = 0; i < poses->size(); i++) {
                if (visited.count(i) == 0 && distance(curr, poses->at(0, i)) <= distance_between_cones) {
                    queue.push(i);
                    visited.insert(i);
                    adj_list[curr_i].push_back(i);
                }
            }
        }
    }
    return adj_list;
}

void ConeConnection::updateMapGraph(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    nav_msgs::OccupancyGrid occupancyGrid;

    // Convert from sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2 -> pcl::PointCloud<PointXYZ>
    pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*cloud_msg, *pcl_pc2);
    pcl::fromPCLPointCloud2(*pcl_pc2, *cloud);



//    std::vector<geometry_msgs::Pose> clusters;
//    auto cluster = clustering(cloud);
    auto walls = linkWallGraph(cloud);
//    for (const auto& cluster : clusters) {
//    }

}

int ConeConnection::comparePoses(geometry_msgs::Pose &first, geometry_msgs::Pose &second) {
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
std::vector<ConeConnection::LinkedList> ConeConnection::linkWalls(std::vector<geometry_msgs::Pose> &cone_poses) {
    std_msgs::String msg;
    msg.data = "Linking Walls: size -> " + std::to_string(cone_poses.size());
    for (geometry_msgs::Pose pose : cone_poses) {
        msg.data += ". x -> " + std::to_string(pose.position.x) + ", y -> " + std::to_string(pose.position.y) +
                    ", z -> " + std::to_string(pose.position.z) + ", ";
    }
    cone_connection_status.publish(msg);

    std::vector<LinkedList> localWalls;
    std::queue<Node *> cone_queue;

    bool max_x_init, min_x_init;
    bool max_y_init, min_y_init;

    while (!cone_poses.empty()) {
        if (!cone_queue.empty()) {
            Node *curr = cone_queue.front();
            cone_queue.pop();

            for (auto cone = cone_poses.begin(); cone < cone_poses.end(); cone++) {
                //distance(cone->position, curr->value.position) <= distance_between_cones
                if (true) {
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

                // Set the min/max values found on the costmap
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

    std_msgs::String linked;
    msg.data = "Linked Walls: ";
    for (LinkedList linkedList : localWalls) {
        msg.data += "size: " + std::to_string(linkedList.size) + ", items: [";
        Node *curr = linkedList.head;
        while (curr != nullptr) {
            msg.data += "x -> " + std::to_string(curr->value.position.x) + ", y -> " +
                        std::to_string(curr->value.position.y) + ", z -> " + std::to_string(curr->value.position.z) +
                        "], ";
            curr = curr->next;
        }
    }
    cone_connection_status.publish(msg);

    return localWalls;
}

ConeConnection::~ConeConnection() {
    for (LinkedList wall : walls_) {
        Node *curr = wall.head;
        for (int i = 0; i < wall.size; i++) {
            curr = curr->next;
            if (curr && curr->prev) {
                delete curr->prev;
            }
        }
    }
}
