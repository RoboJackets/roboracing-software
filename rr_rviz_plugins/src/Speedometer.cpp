#include "../include/Speedometer.hpp"

#include <cmath>

namespace rr_rviz_plugins {

rclcpp::Subscription<rr_msgs::msg::Speed>::SharedPtr speed_subscriber;
float current_speed;
QLabel* label;
rclcpp::Publisher<rr_msgs::msg::Speed>::SharedPtr speed_pub_;
rclcpp::Subscription<rr_msgs::msg::Speed>::SharedPtr speed_sub_;
std::shared_ptr<rclcpp::Node> node;

Speedometer::Speedometer(QWidget *parent) : rviz_common::Panel(parent)  // Base class constructor
{
    // Initialize a label for displaying some data
    label = new QLabel("0 m/s");
    node = std::make_shared<rclcpp::Node>("Speed");
    speed_pub_ = node->create_publisher<rr_msgs::msg::Speed>("/speed", rclcpp::SystemDefaultsQoS());
    speed_sub_ = node->create_subscription<rr_msgs::msg::Speed>(
                "/speed", rclcpp::SystemDefaultsQoS(),
                std::bind(&Speedometer::speedCallback, this, std::placeholders::_1));

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    setLayout(layout);
    rclcpp::spin(node);
    rclcpp::shutdown();
}

void Speedometer::speedCallback(const rr_msgs::msg::Speed::SharedPtr msg) {
    // Create the new contents of the label based on the speed message.
    current_speed = std::abs(msg->speed);
    auto text = std::to_string(current_speed) + " m/s";
    // Set the contents of the label.
    label->setText(text.c_str());
}

// create image of speedometer
void Speedometer::paintEvent(QPaintEvent *event)  {
    //draw a speedometer
    float max_speed = 40;
    float adjusted_speed = std::min(max_speed, current_speed);
    float angle = M_PI * (1.0 - current_speed / max_speed); // in radians
}
}

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::Speedometer, rviz_common::Panel)