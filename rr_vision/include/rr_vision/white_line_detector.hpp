#ifndef WHITE_LINE_DETECTOR_HPP_
#define WHITE_LINE_DETECTOR_HPP_

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rr_msgs/msg/speed.hpp>
#include <rr_msgs/msg/steering.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace white_line {

class WhitePixelNode : public rclcpp::Node {
  public:
    WhitePixelNode(const rclcpp::NodeOptions& options);

  private:
    void process_image(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    /**
     * @brief Publishes to /speed
     */
    rclcpp::Publisher<rr_msgs::msg::Speed>::SharedPtr speed_publisher;
    /**
     * @brief Publishes to /steering
     */
    rclcpp::Publisher<rr_msgs::msg::Steering>::SharedPtr steering_publisher;
    int min_white;
};

}  // namespace white_line
#endif  // WHITE_LINE_DETECTOR_HPP_
