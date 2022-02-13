#ifndef RR_RVIZ_PLUGIN_STEERINGDIRECTION_H
#define RR_RVIZ_PLUGIN_STEERINGDIRECTION_H

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <QPushButton>
#include <rviz_common/panel.hpp>
#include <QVBoxLayout>
#include <QtGui/QPaintEvent>
#include <QtWidgets/QLabel>

namespace rr_rviz_plugins {

class SteeringDirection : public rviz_common::Panel {
  Q_OBJECT
  public:
    explicit SteeringDirection(QWidget *parent = nullptr);

};

}

#endif