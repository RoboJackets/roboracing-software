//
// Created by robojackets on 9/17/16.
//

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

std::map<std::string,pcl::PointCloud<pcl::PointXYZ>::Ptr> partials;
pcl::PointCloud<pcl::PointXYZ>::Ptr map;
bool needsUpdating = false;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg, std::string topic) {

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    auto& partial = partials[topic];

    if(!partial) {
        partial.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    pcl::fromPCLPointCloud2(pcl_pc2, *partial);

    needsUpdating = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapper");

    ros::NodeHandle nh;

    std::string topics[] = {"/scan/pointcloud"};

    std::vector<ros::Subscriber> partial_Subscribers;

    for(const auto& topic : topics) {
        partial_Subscribers.emplace_back(nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, boost::bind(cloudCallback, _1, topic)));
    }

    auto map_pub = nh.advertise<sensor_msgs::PointCloud2>("map",1);

    ros::Rate rate(20);
    while(ros::ok()) {
        ros::spinOnce();

        if(needsUpdating) {

            map->clear();

            for(const auto& partial : partials) {
                *map += *(partial.second);
            }

            pcl::PCLPointCloud2 map_pc2;
            pcl::toPCLPointCloud2(*map,map_pc2);
            sensor_msgs::PointCloud2 msg;
            pcl_conversions::fromPCL(map_pc2,msg);

            msg.header.frame_id = "map";
            msg.header.stamp = ros::Time::now();

            map_pub.publish(msg);

            needsUpdating = false;
            
            rate.sleep();
        }

    }

    return 0;
}