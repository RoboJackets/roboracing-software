////
//// Created by root on 12/20/19.
////
//
//#ifndef SRC_CHASSISPANEL_H
//#define SRC_CHASSISPANEL_H
//
//#include <ros/ros.h>
//#include <rviz/panel.h>
//#include <QVBoxLayout>
//#include <QtGui/QPaintEvent>
//#include <QtWidgets/QLabel>
//#include <rr_msgs/chassis_state.h>
//
// namespace rr_rviz_plugins {
// class ChassisPanel : public rviz::Panel {
//    Q_OBJECT
// public:
//    explicit ChassisPanel(QWidget *parent = nullptr);
//
// protected:
//    ros::NodeHandle nh;
//    ros::Subscriber chassis_sub;
//
// private slots:
//    static void chassisStateCallback(rr_msgs::chassis_state &msg); //need to find type of message later
//    void paintEvent(QPaintEvent *e) override;
//};
//
//} // namespace rr_rviz_plugins
//
//#endif //SRC_CHASSISPANEL_H

#ifndef CATKIN_WS_CHASSISPANEL_H
#define CATKIN_WS_CHASSISPANEL_H

#include <ros/ros.h>
#include <rr_msgs/chassis_state.h>
#include <rviz/panel.h>
#include <std_msgs/Bool.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QtGui/QPaintEvent>
#include <QtWidgets/QLabel>

namespace rr_rviz_plugins {

class ChassisPanel : public rviz::Panel {
    Q_OBJECT
  public:
    explicit ChassisPanel(QWidget *parent = nullptr);

  protected:
    ros::NodeHandle nh;
    ros::Subscriber chassis_sub;

  private slots:
    static void chassisStateCallback(const rr_msgs::chassis_state msg);
    static void timerCallback(const ros::TimerEvent &e);
    void paintEvent(QPaintEvent *e) override;
};

}  // namespace rr_rviz_plugins

#endif  // CATKIN_WS_STARTLIGHTPANEL_H
