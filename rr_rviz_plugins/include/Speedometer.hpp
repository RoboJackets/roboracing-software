#ifndef CATKIN_WS_SPEEDOMETER_H
#define CATKIN_WS_SPEEDOMETER_H

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rr_msgs/msg/speed.hpp>

#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QtGui/QPaintEvent>
#include <QtWidgets/QLabel>

namespace rr_rviz_plugins {

    class Speedometer : public rviz_common::Panel {
        Q_OBJECT

        public:
            Speedometer(QWidget *parent);

        private:
            void speedCallback(const rr_msgs::msg::Speed::SharedPtr msg);
            void paintEvent(QPaintEvent *event);
    };
}

#endif 
