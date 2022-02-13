#include "SteeringDirection.h"

#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/panel.hpp>

#include "rr_msgs/msg/steering.hpp"

namespace rr_rviz_plugins {
QLabel *label;
rclcpp::Publisher<rr_msgs::msg::Steering>::SharedPtr steering_pub_;
rclcpp::Subscription<rr_msgs::msg::Steering>::SharedPtr steering_sub_;
std::shared_ptr<rclcpp::Node> node;
SteeringDirection::SteeringDirection(QWidget *parent) : rviz_common::Panel(parent) {
    node = std::make_shared<rclcpp::Node>("SteeringDirection");
    float curr_direction;
    steering_pub_ = node->create_publisher<rr_msgs::msg::Steering>("/steering", rclcpp::SystemDefaultsQoS());
        steering_sub_ = node->create_subscription<rr_msgs::msg::Steering>(
              "/steering", rclcpp::SystemDefaultsQoS(),
              std::bind(&SteeringDirection::direction_callback, this, std::placeholders::_1);

        QVBoxLayout *layout = new QVBoxLayout;
        setLayout(layout);
        rclcpp::spin(node);
        rclcpp::shutdown();
}

void SteeringDirection::direction_callback(const rr_msgs::msg::Steering &msg) {
    auto text = std::to_string(msg->Steering) + " m/s";
    // Set the contents of the label.
    label->setText(text.c_str());
}
void displayGUI() {}
}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::SteeringDirection, rviz_common::Panel)