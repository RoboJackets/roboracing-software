//
// Created by robojackets on 9/17/16.
//

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr partialT;

std::map<std::string,pcl::PointCloud<pcl::PointXYZ>::Ptr> partials;
pcl::PointCloud<pcl::PointXYZ>::Ptr map;
pcl::PointCloud<pcl::PointXYZ>::Ptr map_filtered;
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

/**
 * @note http://stackoverflow.com/a/27511119
 */
std::vector <std::string> split(const std::string &s, char delim) {
    std::stringstream ss(s);
    std::string item;
    std::vector <std::string> elems;
    while (std::getline(ss, item, delim)) {
        elems.push_back(std::move(item));
    }
    return elems;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapper");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    map.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);

    std::string sourceList = nh_private.param("sources", std::string());

    auto topics = split(sourceList, ',');

    std::vector<ros::Subscriber> partial_Subscribers;

    for(const auto& topic : topics) {
        partial_Subscribers.push_back(nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, boost::bind(cloudCallback, _1, topic)));
        ROS_INFO_STREAM("Mapper subscribed to " << topic);
    }

    auto map_pub = nh.advertise<sensor_msgs::PointCloud2>("map",1);

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(0.2f, 0.2f, 0.2f);

    ros::Rate rate(20);
    while(ros::ok()) {
        ros::spinOnce();

        if(needsUpdating) {

            map->clear();
            map_filtered->clear();

            for(const auto& partial : partials) {
                *map += *(partial.second);
            }
            filter.setInputCloud(map);
            filter.filter(*map_filtered);

            pcl::PCLPointCloud2 map_pc2;
            pcl::toPCLPointCloud2(*map_filtered,map_pc2);
            sensor_msgs::PointCloud2 msg;
            pcl_conversions::fromPCL(map_pc2,msg);

            msg.header.frame_id = "map";
            msg.header.stamp = ros::Time::now();

            map_pub.publish(msg);

            needsUpdating = false;
        }

        rate.sleep();
    }

    return 0;
}