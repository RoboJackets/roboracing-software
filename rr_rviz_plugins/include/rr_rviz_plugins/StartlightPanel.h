//
// Created by nico on 11/3/19.
//
//
//#ifndef CATKIN_WS_STARTLIGHTPANEL_H
//#define CATKIN_WS_STARTLIGHTPANEL_H
//
//#include <ros/ros.h>
//#include <rviz/panel.h>
//#include <std_msgs/Bool.h>
//#include <rr_msgs/speed.h>
//#include <QLabel> //<-- not sure about this one
//
//namespace rr_rviz_plugins {
//    class StartlightPanel : public rviz::Panel {
//        Q_OBJECT
//    public:
//        StartlightPanel(QWidget *parent = 0);
//
//    private:
//        ros::Subscriber start_detector;
//        void paintEvent(QPaintEvent *event) override;
//        static void startlight_callback(const std_msgs::Bool msg/*, QLabel *label*/);
//        void speed_callback(const rr_msgs::speedConstPtr &msg, QLabel *label);
//    };
//}

#ifndef CATKIN_WS_STARTLIGHTPANEL_H
#define CATKIN_WS_STARTLIGHTPANEL_H

#include <ros/ros.h>
#include <rr_msgs/race_reset.h>
#include <rviz/panel.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QtWidgets/QLabel>
#include <std_msgs/Bool.h>

/*
 * All of our panels need to be under the rr_rviz_plugins namespace.
 */
namespace rr_rviz_plugins {

    class StartlightPanel : public rviz::Panel {
    Q_OBJECT
    public:
        StartlightPanel(QWidget *parent = 0);

    protected:
        QPushButton *reset_btn{};
        ros::NodeHandle nh;
        ros::Publisher reset_pub;
        ros::Subscriber start_detector;

    private slots:
//        void resetCallback();
        static void startlightCallback(std_msgs::Bool msg);
        void paintEvent(QPaintEvent *);
    };

}  // namespace rr_rviz_plugins

#endif //CATKIN_WS_STARTLIGHTPANEL_H
