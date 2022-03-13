#include "SpeedometerWorker.hpp"

/**
 * @brief Construct a new Worker object
 * Subscribe to the proper nodes
 */
SpeedometerWorker::SpeedometerWorker() {  // Constructor
    // you could copy data from constructor arguments to internal variables here.
    node = std::make_shared<rclcpp::Node>("Speed");
    speed_sub_ = node->create_subscription<rr_msgs::msg::ChassisState>(
          "/speed", rclcpp::SystemDefaultsQoS(), std::bind(&SpeedometerWorker::speedCallback, this, std::placeholders::_1));
}

/**
 * @brief Destroy the Worker object
 */
SpeedometerWorker::~SpeedometerWorker() {
    rclcpp::shutdown();
}

/**
 * @brief Start spinning the node
 */
void SpeedometerWorker::startNode() {
    rclcpp::spin(node);
}

/**
 * @brief Handle a speed message
 * @param msg received message
 */
void SpeedometerWorker::speedCallback(const rr_msgs::msg::ChassisState::SharedPtr msg) {
    // Create the new contents of the label based on the speed message.
    current_speed = std::abs(msg->speed_mps);
    emit updateSpeed(current_speed);
}
