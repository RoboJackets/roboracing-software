#ifndef STEERING_DIRECTION_H
#define STEERING_DIRECTION_H

#include <QObject>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QtGui/QPaintEvent>
#include <QtWidgets/QLabel>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rr_msgs/msg/chassis_state.hpp>
#include <rviz_common/panel.hpp>

#include "SteeringWorker.hpp"

namespace rr_rviz_plugins {

class SteeringDirection : public rviz_common::Panel {
    Q_OBJECT

  public:
    explicit SteeringDirection(QWidget *parent = nullptr);
    // DO NOT CREATE A DESTRUCTOR

  public slots:
    void setLabel(float speed);

  signals:
    void startNode();

  private:
    QLabel *label;
    SteeringWorker *worker;
};
}  // namespace rr_rviz_plugins

#endif
