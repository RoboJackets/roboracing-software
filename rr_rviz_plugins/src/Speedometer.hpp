#ifndef SPEEDOMETER_H
#define SPEEDOMETER_H

#include <QObject>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QtGui/QPaintEvent>
#include <QtWidgets/QLabel>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rr_msgs/msg/speed.hpp>
#include <rviz_common/panel.hpp>

#include "Worker.hpp"

namespace rr_rviz_plugins {

class Speedometer : public rviz_common::Panel {
    Q_OBJECT

  public:
    explicit Speedometer(QWidget *parent = nullptr);

  public slots:
    void setLabel(float speed);

  signals:
    void startNode();

  private:
    QLabel *label;
};
}  // namespace rr_rviz_plugins

#endif
