#ifndef SPEEDOMETER_H
#define SPEEDOMETER_H

#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QtGui/QPaintEvent>
#include <QtWidgets/QLabel>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rr_msgs/msg/speed.hpp>
#include <rviz_common/panel.hpp>

namespace rr_rviz_plugins {

class Speedometer : public rviz_common::Panel {
    Q_OBJECT

  public:
    explicit Speedometer(QWidget *parent = nullptr);

  private:
    float current_speed;
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<rr_msgs::msg::Speed>::SharedPtr speed_pub_;
    rclcpp::Subscription<rr_msgs::msg::Speed>::SharedPtr speed_sub_;
    QLabel *label;
    void speedCallback(const rr_msgs::msg::Speed::SharedPtr msg);
    // void paintEvent(QPaintEvent *event);
};
}  // namespace rr_rviz_plugins

#endif
