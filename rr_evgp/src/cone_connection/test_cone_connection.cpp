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

void test_cone_connection::publish_intermittently(geometry_msgs::PoseArray (&get_next_points)(geometry_msgs::PoseArray)) {
    bool _execute = true;

    std::thread([=]() {
        geometry_msgs::PoseArray curr = geometry_msgs::PoseArray();
        while (_execute) {
            curr = get_next_points(curr);
            points.publish(curr);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }).detach();
}

test_cone_connection::test_cone_connection(ros::NodeHandle nh, const std::string &output_centroids,
                                           XmlRpc::XmlRpcValue test_points) {
    points = nh.advertise<geometry_msgs::PoseArray>(output_centroids, 1);
    geometry_msgs::PoseArray pose_array = parse_points(test_points);

    publish_intermittently(test_cone_connection::line);
}

geometry_msgs::PoseArray test_cone_connection::line(geometry_msgs::PoseArray previous) {

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_cone_connection");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    XmlRpc::XmlRpcValue test_points;
    std::string output_centroids;

    nhp.getParam("coords", test_points);
    nhp.getParam("output_centroids", output_centroids);

    test_cone_connection test(nh, output_centroids, test_points);

    ros::spin();
    return 0;
}
