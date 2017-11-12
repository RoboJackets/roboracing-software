#ifndef PROJECT_PLATFORMENABLEPANEL_H
#define PROJECT_PLATFORMENABLEPANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <rr_platform/speed.h>
#include <QLabel>

namespace rr_rviz_plugins {

class ExamplePanel : public rviz::Panel {
Q_OBJECT
public:

    ExamplePanel(QWidget *parent = 0);

protected:
    ros::Subscriber speed_subscriber;

    void speed_callback(const rr_platform::speedConstPtr &msg, QLabel *label);

};

}

#endif //PROJECT_PLATFORMENABLEPANEL_H
