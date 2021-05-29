//
// Created by charlie on 5/23/21.
//

#include "test_cone_connection.h"

#include <thread>

#include "geometry_msgs/PoseArray.h"
#include "ros/ros.h"

geometry_msgs::PoseArray test_cone_connection::parse_points(XmlRpc::XmlRpcValue &test_points) {
    std::vector<geometry_msgs::Pose> poses;

    // Iterate through test points
    for (int32_t i = 0; i < test_points.size(); i++) {
        geometry_msgs::Pose pose;
        pose.position.x = test_points[i][0];
        pose.position.y = test_points[i][1];
        pose.position.z = test_points[i][2];
        poses.push_back(pose);
    }
    geometry_msgs::PoseArray poseArray;
    poseArray.poses = poses;
    return poseArray;
}

void test_cone_connection::publish_intermittently(void (&get_next_points)(geometry_msgs::PoseArray &, float)) {
    bool _execute = true;

    std::thread([=]() {
        geometry_msgs::PoseArray curr;
        while (_execute) {
            get_next_points(curr, distance);
            points.publish(curr);
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }
    }).detach();
}

test_cone_connection::test_cone_connection(ros::NodeHandle nh, const std::string &output_centroids,
                                           XmlRpc::XmlRpcValue test_points, float distance) {
    points = nh.advertise<geometry_msgs::PoseArray>(output_centroids, 1);
    geometry_msgs::PoseArray pose_array = parse_points(test_points);
    this->distance = distance;

    publish_intermittently(test_cone_connection::line);
}

void test_cone_connection::line(geometry_msgs::PoseArray &previous, float distance) {
    if (previous.poses.empty()) {
        geometry_msgs::Pose poseLeftStart;
        poseLeftStart.position.x = distance * 1.5;
        poseLeftStart.position.y = 0;
        poseLeftStart.position.z = 0;
        previous.poses.push_back(poseLeftStart);

        geometry_msgs::Pose poseRightStart;
        poseRightStart.position.x = -distance * 1.5;
        poseRightStart.position.y = 0;
        poseRightStart.position.z = 0;
        previous.poses.push_back(poseRightStart);

        geometry_msgs::Pose poseLeftNext;
        poseLeftNext.position.x = distance * 1.5;
        poseLeftNext.position.y = distance;
        poseLeftNext.position.z = 0;
        previous.poses.push_back(poseLeftNext);

        geometry_msgs::Pose poseRightNext;
        poseRightStart.position.x = -distance * 1.5;
        poseRightStart.position.y = distance;
        poseRightStart.position.z = 0;
        previous.poses.push_back(poseRightStart);
    } else {
        //cycle array to make room for new ones
        previous.poses[0] = previous.poses[2];
        previous.poses[1] = previous.poses[3];

        previous.poses[2].position.y += distance;
        previous.poses[3].position.y += distance;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_cone_connection");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    XmlRpc::XmlRpcValue test_points;
    std::string output_centroids;
    float distance;

    nhp.getParam("coords", test_points);
    nhp.getParam("output_centroids", output_centroids);
    nhp.getParam("cone_distance", distance);

    test_cone_connection test(nh, output_centroids, test_points, distance);

    ros::spin();
    return 0;
}
