#include "ground_segmentation.hpp"

GroundSegmentation::GroundSegmentation() : Node("ground_segmentation") {
    pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation", rclcpp::SensorDataQoS());
    velodyne_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          "/velodyne_points", rclcpp::SensorDataQoS(),
          std::bind(&GroundSegmentation::PointCloudCallback, this, std::placeholders::_1));
}

void GroundSegmentation::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl_pub_->publish(*msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundSegmentation>());
    rclcpp::shutdown();
    return 0;
}