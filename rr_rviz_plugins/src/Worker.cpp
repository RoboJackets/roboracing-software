#include "Worker.hpp"

Worker::Worker() {  // Constructor
    // you could copy data from constructor arguments to internal variables here.
    node = std::make_shared<rclcpp::Node>("Speed");
    speed_pub_ = node->create_publisher<rr_msgs::msg::Speed>("/speed_new", rclcpp::SystemDefaultsQoS());
    speed_sub_ = node->create_subscription<rr_msgs::msg::Speed>(
          "/speed", rclcpp::SystemDefaultsQoS(), std::bind(&Worker::speedCallback, this, std::placeholders::_1));
}

Worker::~Worker() {  // Destructor
    rclcpp::shutdown();
}

void Worker::startNode() {
    rclcpp::spin(node);
}

void Worker::speedCallback(const rr_msgs::msg::Speed::SharedPtr msg) {
    // Create the new contents of the label based on the speed message.
    current_speed = std::abs(msg->speed);
    emit finished(current_speed);
}
