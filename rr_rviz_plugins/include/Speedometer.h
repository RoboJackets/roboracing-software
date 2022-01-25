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
        ros::NodeHandle handle;
        ros::Subscriber speed_subscriber;

    private slots:
        static void Speedometer::speedCallback(const rr_msgs::speed &msg, QLabel *label);
        void Speedometer::paintEvent(QPaintEvent *event);
// };

} 

#endif 
