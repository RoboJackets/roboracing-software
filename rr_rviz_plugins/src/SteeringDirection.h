#ifndef RR_RVIZ_PLUGIN_STEERINGDIRECTION_H
#define RR_RVIZ_PLUGIN_STEERINGDIRECTION_H

#include <QPushButton>
#include <QVBoxLayout>
#include <QtGui/QPaintEvent>
#include <QtWidgets/QLabel>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace rr_rviz_plugins {

class SteeringDirection : public rviz_common::Panel {
    Q_OBJECT
  public:
    explicit SteeringDirection(QWidget *parent = nullptr);
};

}  // namespace rr_rviz_plugins

#endif