#include "ground_segmentation.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

GroundSegmentation::GroundSegmentation() : Node("ground_segmentation") {
    pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation", rclcpp::SystemDefaultsQoS());
    velodyne_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", rclcpp::SensorDataQoS(),
        std::bind(&GroundSegmentation::PointCloudCallback, this, std::placeholders::_1)
    );
}

void GroundSegmentation::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PCLPointCloud2 pcl_cloud = pcl::PCLPointCloud2();

    pcl_conversions::moveToPCL(*msg, pcl_cloud);

    sensor_msgs::msg::PointCloud2 result = sensor_msgs::msg::PointCloud2();

    pcl_conversions::moveFromPCL(pcl_cloud, result);

    pcl_pub_->publish(result);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundSegmentation>());
    rclcpp::shutdown();
    return 0;
}