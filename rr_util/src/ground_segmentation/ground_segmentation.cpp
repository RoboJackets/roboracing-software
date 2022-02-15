#include "ground_segmentation.hpp"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include "ground_segmenter/ground_segmenter.hpp"

GroundSegmentation::GroundSegmentation() : Node("ground_segmentation") {
    pcl_ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/ground", rclcpp::SystemDefaultsQoS());
    pcl_obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/obstacle", rclcpp::SystemDefaultsQoS());
    velodyne_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          "/velodyne_points2", rclcpp::SensorDataQoS(),
          std::bind(&GroundSegmentation::PointCloudCallback, this, std::placeholders::_1));

    declare_parameter<double>("r_min_square", 0.9);
    declare_parameter<double>("r_max_square", 400.0);
    declare_parameter<int>("n_bins", 30);
    declare_parameter<int>("n_segments", 180);
    declare_parameter<double>("max_dist_to_line", .15);
    declare_parameter<double>("min_slope", 0.0);
    declare_parameter<double>("max_slope", 1.25);
    declare_parameter<double>("max_error_square", 0.01);
    declare_parameter<double>("long_threshold", 2.0);
    declare_parameter<double>("max_long_height", 0.1);
    declare_parameter<double>("max_start_height", 0.25);
    declare_parameter<double>("sensor_height", 0.6);
    declare_parameter<double>("line_search_angle", 0.3);
    declare_parameter<int>("n_threads", 4);
}

void GroundSegmentation::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud{};
    pcl::fromROSMsg(*msg, pcl_cloud);

    GroundSegmenterParams params = GroundSegmenterParams{};
    params.r_min_square = get_parameter("r_min_square").as_double();
    params.r_max_square = get_parameter("r_max_square").as_double();
    params.n_bins = get_parameter("n_bins").as_int();
    params.n_segments = get_parameter("n_segments").as_int();

    params.max_dist_to_line = get_parameter("max_dist_to_line").as_double();
    params.min_slope = get_parameter("min_slope").as_double();
    params.max_slope = get_parameter("max_slope").as_double();
    params.max_error_square = get_parameter("max_error_square").as_double();
    params.long_threshold = get_parameter("long_threshold").as_double();
    params.max_long_height = get_parameter("max_long_height").as_double();
    params.max_start_height = get_parameter("max_start_height").as_double();
    params.sensor_height = get_parameter("sensor_height").as_double();
    params.line_search_angle = get_parameter("line_search_angle").as_double();

    params.n_threads = get_parameter("n_threads").as_int();

    GroundSegmenter segmenter(params);
    pcl::PointCloud<pcl::PointXYZ> cloud_transformed;

    std::vector<int> labels;

    //todo: transform cloud so that the vertical axis (z) is always up relative to gravity
    //Eigen::Affine3d tf = Eigen::Affine3d::Identity<double, 3, 2>;
    //pcl::transformPointCloud(pcl_cloud, cloud_transformed, tf);

    segmenter.segment(pcl_cloud, &labels);
    pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;

    for (size_t i = 0; i < pcl_cloud.size(); ++i) {
        if (labels[i] == 1) {
            ground_cloud.push_back(pcl_cloud[i]);
        } else {
            obstacle_cloud.push_back(pcl_cloud[i]);
        }
    }

    sensor_msgs::msg::PointCloud2 ros_ground_result{};
    pcl::toROSMsg(ground_cloud, ros_ground_result);
    ros_ground_result.header = msg->header;
    pcl_ground_pub_->publish(ros_ground_result);

    sensor_msgs::msg::PointCloud2 ros_obstacle_result{};
    pcl::toROSMsg(obstacle_cloud, ros_obstacle_result);
    ros_obstacle_result.header = msg->header;
    pcl_obstacle_pub_->publish(ros_obstacle_result);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundSegmentation>());
    rclcpp::shutdown();
    return 0;
}