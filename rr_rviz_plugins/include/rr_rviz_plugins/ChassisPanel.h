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
