#include "tag_detection.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pub_pointcloud;
std::vector<pcl::PointXYZ> opponent_points;

pcl::PointCloud<pcl::PointXYZ> draw_opponents(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

void draw_opponent(pcl::PointCloud<pcl::PointXYZ>& cloud, const int &id, pcl::PointXYZ &april_tag_center);

pcl::PointCloud<pcl::PointXYZ> draw_opponents(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ> opponent_cloud;
//    opponent_points.clear();
    auto msgs = msg->detections;
    for (const auto& message : msgs) { //Iterate through all discovered April Tags
        pcl::PointXYZ opponent_point = pcl::PointXYZ(message.pose.pose.pose.position.x,
                                                     message.pose.pose.pose.position.y,
                                                     message.pose.pose.pose.position.z);
        draw_opponent(opponent_cloud, message.id[0], opponent_point);
    }
    return opponent_cloud;
}

void publishPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud) {
    sensor_msgs::PointCloud2 outmsg;
    pcl::toROSMsg(cloud, outmsg);
    outmsg.header.frame_id = "base_footprint";
    pub_pointcloud.publish(outmsg);
}

void draw_opponent(pcl::PointCloud<pcl::PointXYZ>& cloud, const int &id, pcl::PointXYZ &april_tag_center) {
    int side_length = 10;
    int start_x = april_tag_center.x - side_length / 2;
    for (int x = 0; x < side_length; x++) {
        for (int y = 0; y < side_length; y++) {
            cloud.push_back(pcl::PointXYZ(april_tag_center.z, april_tag_center.x * -1, april_tag_center.y * -1));
        }
    }
}

void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
    std::uint32_t second = msg->header.stamp.sec;
    auto msgs = msg->detections;
    pcl::PointCloud<pcl::PointXYZ> cloud = draw_opponents(msg);
    publishPointCloud(cloud);
    for (const auto& message : msgs) { //Iterate through all discovered April Tags
        ROS_INFO_STREAM((message.id[0]));
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tag_detection");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("tag_detections", 1, callback);
    pub_pointcloud = n.advertise<sensor_msgs::PointCloud2>("/apriltags/pointcloud", 1);
    ros::spin();
    return 0;
}
