#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>


using cloud_t = pcl::PointCloud<pcl::PointXYZ>;
using cloud_ptr_t = pcl::PointCloud<pcl::PointXYZ>::Ptr;

std::map<std::string, std::queue<sensor_msgs::PointCloud2ConstPtr>> queues;
bool hasChanged = false;

bool shouldUpdate() {
    if (!hasChanged) {
        return false;
    }

    for (const auto& entry : queues) {
        ROS_INFO_STREAM("Queue " << entry.first << " has " << entry.second.size());
        if (entry.second.empty()) {
            return false;
        }
    }

    return true;
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg, std::string topic) {
    queues[topic].push(msg);
    hasChanged = true;
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

    cloud_ptr_t combo_cloud(new cloud_t);
    cloud_ptr_t combo_cloud_filtered(new cloud_t);
    cloud_ptr_t transformed(new cloud_t);
    cloud_ptr_t filtered_pass(new cloud_t);
    cloud_ptr_t filtered_vg(new cloud_t);
    cloud_ptr_t filtered_stat(new cloud_t);

    std::string sourceList = nh_private.param("sources", std::string());
    std::string publishName = nh_private.param("destination", std::string("/map"));
    std::string combinedFrame = nh_private.param("combined_frame", std::string("base_footprint"));
    int refreshRate = nh_private.param("refresh_rate", 30);
    float groundThreshold = nh_private.param("ground_threshold", 0.05);
    float outlierThreshold = nh_private.param("outlier_threshold", 2.0);
    int outlierNeighbors = nh_private.param("outlier_neighbors", 1);
    float vgFilterSize = nh_private.param("vg_filter_size", 0.05);

    auto topics = split(sourceList, ' ');

    std::vector<ros::Subscriber> partial_Subscribers;

    for(const auto& topic : topics) {
        partial_Subscribers.push_back(nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, boost::bind(cloudCallback, _1, topic)));
        ROS_INFO_STREAM("Mapper subscribed to " << topic);
    }

    auto combo_pub = nh.advertise<sensor_msgs::PointCloud2>(publishName, 1);

    // set up point reduction filter
    pcl::VoxelGrid<pcl::PointXYZ> filterVG;
    filterVG.setLeafSize(vgFilterSize, vgFilterSize, vgFilterSize);

    // set up z axis value filter (supposed to take out ground)
    pcl::PassThrough<pcl::PointXYZ> filterPass;
    filterPass.setFilterFieldName("z");
    filterPass.setFilterLimits(groundThreshold, 5.0);

    // set up statistical outlier filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filterOutliers;
    filterOutliers.setMeanK(outlierNeighbors);
    filterOutliers.setStddevMulThresh(outlierThreshold);

    tf::TransformListener tfListener;

    ROS_INFO("starting mapper main loop");

    ros::Rate rate(refreshRate);
    while(ros::ok()) {
        ros::spinOnce();

        if(shouldUpdate()) {

            combo_cloud->clear();
            combo_cloud_filtered->clear();

            for(auto& entry : queues) {
                auto& queue = entry.second;
                auto msg = queue.front();  // copy
                queue.pop();

                // convert from message to pcl pointcloud
                pcl::PCLPointCloud2 pcl_pc2;
                pcl_conversions::toPCL(*msg, pcl_pc2);

                cloud_t partialCloud;
                pcl::fromPCLPointCloud2(pcl_pc2, partialCloud);

                if (partialCloud.points.empty()) {
                    continue;
                }

                // frame transform
                transformed->clear();
                tfListener.waitForTransform(msg->header.frame_id, combinedFrame, ros::Time(0), ros::Duration(0.1));
                pcl_ros::transformPointCloud(combinedFrame, partialCloud, *transformed, tfListener);

                // filter by z axis height
                // filtered_pass->clear();
                // filterPass.setInputCloud(transformed);
                // filterPass.filter(*filtered_pass);
                *filtered_pass = *transformed;

                *(combo_cloud) += *filtered_pass;
            }

            // make 2D
            for(auto& pt : combo_cloud->points) {
                pt.z = 0;
            }

            ROS_INFO("combo_cloud has %d points", (int)combo_cloud->points.size());

            // reduce number of points using a grid
            // filtered_vg->clear();
            // filterVG.setInputCloud(combo_cloud);
            // filterVG.filter(*filtered_vg);

            // filter statistical outliers
            // filtered_pass->clear();
            // filterOutliers.setInputCloud(filtered_vg);
            // filterOutliers.filter(*filtered_pass);
            *filtered_pass = *combo_cloud;

            // convert back to message
            if (!combo_cloud->points.empty()) {
                pcl::PCLPointCloud2 combo_pc2;
                pcl::toPCLPointCloud2(*filtered_pass, combo_pc2);
                sensor_msgs::PointCloud2 msg;
                pcl_conversions::fromPCL(combo_pc2, msg);

                msg.header.frame_id = combinedFrame;
                msg.header.stamp = ros::Time::now();

                combo_pub.publish(msg);
            } else {
                ROS_INFO("pointcloud empty");
            }

            hasChanged = false;
        }

        rate.sleep();
    }

    return 0;
}

