#include "SteeringWorker.hpp"

/**
 * @brief Construct a new Worker object
 * Subscribe to the proper nodes
 */
SteeringWorker::SteeringWorker() {  // Constructor
    // you could copy data from constructor arguments to internal variables here.
    node = std::make_shared<rclcpp::Node>("Steering");
    steering_sub_ = node->create_subscription<rr_msgs::msg::ChassisState>(
          "/angle", rclcpp::SystemDefaultsQoS(), std::bind(&SteeringWorker::angleCallback, this, std::placeholders::_1));
}

/**
 * @brief Destroy the Worker object
 */
SteeringWorker::~SteeringWorker() {
    rclcpp::shutdown();
}

/**
 * @brief Start spinning the node
 */
void SteeringWorker::startNode() {
    rclcpp::spin(node);
}

/**
 * @brief Handle a speed message
 * @param msg received message
 */
void SteeringWorker::angleCallback(const rr_msgs::msg::ChassisState::SharedPtr msg) {
    // Create the new contents of the label based on the speed message.
    current_angle = std::abs(msg->steer_rad);
    emit updateAngle(current_angle);
}
