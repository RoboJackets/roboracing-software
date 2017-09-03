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
pcl::PointCloud<pcl::PointXYZ>::Ptr combo_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr combo_cloud_filtered;
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
    ros::init(argc, argv, "pcl_combiner");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    combo_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    combo_cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);

    std::string sourceList = nh_private.param("sources", std::string());
    std::string publishName = nh_private.param("destination", std::string("/map"));
    std::string combinedFrame = nh_private.param("combined_frame", std::string("base_footprint"));
    int refreshRate = nh_private.param("refresh_rate", 30);

    auto topics = split(sourceList, ',');

    std::vector<ros::Subscriber> partial_Subscribers;

    for(const auto& topic : topics) {
        partial_Subscribers.push_back(nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, boost::bind(cloudCallback, _1, topic)));
        ROS_INFO_STREAM("Pointcloud combiner subscribed to " << topic);
    }

    auto combo_pub = nh.advertise<sensor_msgs::PointCloud2>("map",1);

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(0.2f, 0.2f, 0.2f);

    ros::Rate rate(refreshRate);
    while(ros::ok()) {
        ros::spinOnce();

        if(needsUpdating) {

            combo_cloud->clear();
            combo_cloud_filtered->clear();

            for(const auto& partial : partials) {
                *combo_cloud += *(partial.second);
            }
            filter.setInputCloud(combo_cloud);
            filter.filter(*combo_cloud_filtered);

            pcl::PCLPointCloud2 combo_pc2;
            pcl::toPCLPointCloud2(*combo_cloud_filtered, combo_pc2);
            sensor_msgs::PointCloud2 msg;
            pcl_conversions::fromPCL(combo_pc2,msg);

            msg.header.frame_id = combinedFrame;
            msg.header.stamp = ros::Time::now();

            combo_pub.publish(msg);

            needsUpdating = false;
        }

        rate.sleep();
    }

    return 0;
}