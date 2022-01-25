#ifndef CATKIN_WS_SPEEDOMETER_H
#define CATKIN_WS_SPEEDOMETER_H

#include <ros/ros.h>
#include <rr_msgs/speed.h>
#include <rviz/display.h>
#include <std_msgs/Bool.h>

#include <QPushButton>
#include <QVBoxLayout>
#include <QtGui/QPaintEvent>
#include <QtWidgets/QLabel>

namespace rr_rviz_plugins {

// class ChassisPanel : public rviz::Panel {
//     Q_OBJECT
//   public:
//     explicit ChassisPanel(QWidget *parent = nullptr);

    protected:
        rclcpp::Subscription<rr_msgs::msg::Speed>::SharedPtr speed_subscriber;

    private:
        void Speedometer::speedCallback(const rr_msgs::msg::Speed::SharedPtr msg);
        void Speedometer::paintEvent(QPaintEvent *event);
// };

} 

#endif 
