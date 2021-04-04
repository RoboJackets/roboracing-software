//
// Created by Charlie on 4/4/21.
//

#include "geometry_msgs/PoseArray.h"
#include "ros/ros.h"

void callback(const geometry_msgs::PoseArray &cloud_msg) {}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cone_connection");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    std::string output_centroids;
    nhp.getParam("output_centroids", output_centroids);
    ros::Subscriber sub = nh.subscribe(output_centroids, 1, &callback);

    ros::spin();
    return 0;
}
