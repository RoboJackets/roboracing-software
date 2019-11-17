//
// Created by nico on 11/3/19.
//

#ifndef CATKIN_WS_STARTLIGHTPANEL_H
#define CATKIN_WS_STARTLIGHTPANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/Bool.h>
#include <rr_msgs/speed.h>
#include <QLabel> //<-- not sure about this one

namespace rr_rviz_plugins {
    class StartlightPanel : public rviz::Panel {
        Q_OBJECT
    public:
        StartlightPanel(QWidget *parent = 0);

    private:
        ros::Subscriber start_detector;
        void paintEvent(QPaintEvent *event) override;
        static void startlight_callback(const std_msgs::Bool msg/*, QLabel *label*/);
        void speed_callback(const rr_msgs::speedConstPtr &msg, QLabel *label);
    };
}

#endif //CATKIN_WS_STARTLIGHTPANEL_H
