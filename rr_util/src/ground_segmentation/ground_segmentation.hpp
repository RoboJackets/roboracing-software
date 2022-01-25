#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class GroundSegmentation : public rclcpp::Node {
    public:
        GroundSegmentation();

    private:
        void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_sub_;
};