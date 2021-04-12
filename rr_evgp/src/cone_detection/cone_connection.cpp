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

static double distance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2) {
    return pow((p2.position.x - p1.position.x), 2) +
           pow((p2.position.y - p1.position.y), 2) +
           pow((p2.position.z - p1.position.z), 2);
}

static bool close(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) {

}

void callback(const geometry_msgs::PoseArray &cone_array) {
    std::vector<geometry_msgs::Pose> local_wall1;
    std::vector<geometry_msgs::Pose> local_wall2;

    wall1.push_back(cone_array.poses[0]);
    for (geometry_msgs::Pose cone : cone_array.poses) {
        if (distance(cone, wall1[0]) < cone_distance) {
            wall1.push_back(cone);
        } else {
            wall2.push_back(cone);
        }
    }

    if (distance(local_wall1[local_wall1.size() - 1], wall1[final_index1]) <
        distance(local_wall1[local_wall1.size() - 1], wall2[final_index2])) {
        // local_wall 1 is closer to global wall1

        // Iterate backwards through global wall1 starting at final_index1
        // until reach value that is close to 0th item in local wall1
        long start_wall1;
        for (auto it = wall1.rbegin() + static_cast<int>(wall1.size() - final_index1); it != wall1.rend(); ++it) {
            if (close(it->position, wall1[0].position)) {
                start_wall1 = std::distance(begin(wall1), it.base()) - 1; // get current index
            }
        }

        // Now iterate forwards and update overlapping cone points

        // Iterate backwards through global wall2 until reach value
        // that is close to 0th item in local wall2
    } else {
        // local_wall1 is closer to global wall2
        // local_wall and global wall are reversed!

        // Iterate backwards through global wall2 until reach value
        // that is close to 0th item in local wall1

        // Iterate backwards through global wall1 until reach value
        // that is close to 0th item in local wall2
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
