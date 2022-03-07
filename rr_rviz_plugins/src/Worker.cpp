#include "Worker.hpp"

/**
 * @brief Construct a new Worker object
 * Subscribe to the proper nodes
 */
Worker::Worker() {  // Constructor
    // you could copy data from constructor arguments to internal variables here.
    node = std::make_shared<rclcpp::Node>("Speed");
    speed_sub_ = node->create_subscription<rr_msgs::msg::Speed>(
          "/speed", rclcpp::SystemDefaultsQoS(), std::bind(&Worker::speedCallback, this, std::placeholders::_1));
}

/**
 * @brief Destroy the Worker object
 */
Worker::~Worker() {
    rclcpp::shutdown();
}

/**
 * @brief Start spinning the node
 */
void Worker::startNode() {
    rclcpp::spin(node);
}

/**
 * @brief Handle a speed message
 * @param msg received message
 */
void Worker::speedCallback(const rr_msgs::msg::Speed::SharedPtr msg) {
    // Create the new contents of the label based on the speed message.
    current_speed = std::abs(msg->speed);
    emit updateSpeed(current_speed);
}
