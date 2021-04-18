//
// Created by Charlie on 4/4/21.
//

#include "geometry_msgs/PoseArray.h"
#include "ros/ros.h"

static double cone_distance;
static int final_index1 = 0;
static int final_index2 = 0;
static std::vector<geometry_msgs::Pose> wall1;
static std::vector<geometry_msgs::Pose> wall2;
static geometry_msgs::Pose car_pose;
static float close_distance = 0.25;

static double distance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) {
    return pow((p2.x - p1.x), 2) +
           pow((p2.y - p1.y), 2) +
           pow((p2.z - p1.z), 2);
}

static bool close(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) {
    return distance(p1, p2) < close_distance;
}

/*
 * Modifies p1 to be the average of itself and p2
 */
static inline void average_pose(geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2) {
    p1.position.x = (p1.position.x + p2.position.x) / 2;
    p1.position.y = (p1.position.y + p2.position.y) / 2;
    p1.position.z = (p1.position.z + p2.position.z) / 2;
}

static void update_cones(std::vector<geometry_msgs::Pose> &wall_vector,
                         std::vector<geometry_msgs::Pose> &output_wall_vector) {
    // Iterate backwards through global wall1 starting at final_index1
    // until reach value that is close to 0th item in local wall1
    auto start_wall_it = wall_vector.rbegin();
    for (auto it = wall_vector.rbegin(); it != wall_vector.rend(); --it) {
        if (close(it->position, output_wall_vector[0].position)) {
            start_wall_it = it; // get current index
            break;
        }
    }

    // Now iterate forwards and update overlapping cone points
    for (auto it = output_wall_vector.begin();
        it < output_wall_vector.end() && start_wall_it.base() < wall_vector.end();
        it++, start_wall_it++) {
        double dist;
        if (close(it->position, start_wall_it->position)) {
            average_pose(*start_wall_it.base(), *it.base());
        } else if (distance(car_pose.position, start_wall_it->position) <
                   (dist = distance(car_pose.position, it->position))) {
            if (dist < distance(car_pose.position, (start_wall_it + 1)->position)) {
                wall_vector.insert(start_wall_it.base(), (const geometry_msgs::Pose &) it.base());
            }
        }
    }
}

// Relies on the elements that are closes to the car being at the front of the array
// Must be sorted by distance to car
void callback(const geometry_msgs::PoseArray &cone_array) {
    std::vector<geometry_msgs::Pose> local_wall1;
    std::vector<geometry_msgs::Pose> local_wall2;

    wall1.push_back(cone_array.poses[0]);
    for (geometry_msgs::Pose cone : cone_array.poses) {
        if (distance(cone.position, wall1[0].position) < cone_distance) {
            wall1.push_back(cone);
        } else {
            wall2.push_back(cone);
        }
    }

    if (distance(local_wall1[local_wall1.size() - 1].position, wall1[final_index1].position) <
        distance(local_wall1[local_wall1.size() - 1].position, wall2[final_index2].position)) {
        // local_wall 1 is closer to global wall1

        // Update wall1 with cones from local wall 1
        update_cones(wall1, local_wall1);

        // Update wall2 with cones from local wall 2
        update_cones(wall2, local_wall2);
    } else {
        // local_wall1 is closer to global wall2
        // local_wall and global wall are reversed!

        // Update wall1 with cones from local wall 2
        update_cones(wall1, local_wall2);

        // Update wall2 with cones from local wall 1
        update_cones(wall2, local_wall1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cone_connection");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    std::string output_centroids;
    nhp.getParam("output_centroids", output_centroids);
    nhp.getParam("cone_distance", cone_distance);

    // Square cone distance so we don't have to square root the discovered distance
    cone_distance *= cone_distance;
    ros::Subscriber sub = nh.subscribe(output_centroids, 1, &callback);

    ros::spin();
    return 0;
}
