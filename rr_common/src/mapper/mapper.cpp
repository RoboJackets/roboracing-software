//
// Created by robojackets on 9/17/16.
//

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointCloud<pcl::PointXYZ> cloud_t;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_t;

std::map<std::string,sensor_msgs::PointCloud2ConstPtr> partials;
cloud_ptr_t combo_cloud;
cloud_ptr_t combo_cloud_filtered;
bool needsUpdating = false;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg, std::string topic) {
    partials[topic] = msg;
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

    combo_cloud.reset(new cloud_t);
    combo_cloud_filtered.reset(new cloud_t);

    std::string sourceList = nh_private.param("sources", std::string());
    std::string publishName = nh_private.param("destination", std::string("/map"));
    std::string combinedFrame = nh_private.param("combined_frame", std::string("base_footprint"));
    int refreshRate = nh_private.param("refresh_rate", 30);
    float groundThreshold = nh_private.param("ground_threshold", 0.05);

    auto topics = split(sourceList, ',');

    std::vector<ros::Subscriber> partial_Subscribers;

    for(const auto& topic : topics) {
        partial_Subscribers.push_back(nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, boost::bind(cloudCallback, _1, topic)));
        ROS_INFO_STREAM("Mapper subscribed to " << topic);
    }

    auto combo_pub = nh.advertise<sensor_msgs::PointCloud2>(publishName, 1);

    pcl::VoxelGrid<pcl::PointXYZ> filterVG;
    filterVG.setLeafSize(0.1f, 0.1f, 0.1f);

    pcl::PassThrough<pcl::PointXYZ> filterPass;
    filterPass.setFilterFieldName("z");
    filterPass.setFilterLimits(groundThreshold, 5.0);

    tf::TransformListener tfListener;

    ros::Rate rate(refreshRate);
    while(ros::ok()) {
        ros::spinOnce();

        if(needsUpdating) {

            combo_cloud->clear();
            combo_cloud_filtered->clear();

            for(const auto& partial_pair : partials) {
                auto& msg = partial_pair.second;
                if(msg->width == 0) continue;

                pcl::PCLPointCloud2 pcl_pc2;
                pcl_conversions::toPCL(*msg, pcl_pc2);
                cloud_t partialCloud;
                pcl::fromPCLPointCloud2(pcl_pc2, partialCloud);

                cloud_ptr_t transformed(new cloud_t);
                pcl_ros::transformPointCloud(combinedFrame, partialCloud, *transformed, tfListener);

                cloud_t filtered;
                filterPass.setInputCloud(transformed);
                filterPass.filter(filtered);

                *(combo_cloud) += filtered;
            }

            for(auto& pt : *combo_cloud) {
                pt.z = 0;
            }

            filterVG.setInputCloud(combo_cloud);
            filterVG.filter(*combo_cloud_filtered);

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

