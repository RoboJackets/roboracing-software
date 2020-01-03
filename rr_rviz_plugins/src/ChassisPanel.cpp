////
//// Created by root on 12/20/19.
////
//#include <pluginlib/class_list_macros.h>
//#include <rr_rviz_plugins/ChassisPanel.h>
//#include <QVBoxLayout>
//#include <QtGui/QPaintEvent>
//#include <QtGui/QPainter>
//#include <QtWidgets/QLabel>
//#include <rr_msgs/chassis_state.h>
//#include <math.h>
//
//
//namespace rr_rviz_plugins {
//    bool received_message;
//    QLabel *speed_label;
//    QLabel *heading_label;
//    QLabel *boardstate_label;
//ChassisPanel::ChassisPanel(QWidget *parent)
//        : rviz::Panel(parent)
//{
//    auto *layout = new QVBoxLayout;
//    //labels for all the information
//    speed_label = new QLabel("No speed");
//    heading_label = new QLabel("no heading");
//    boardstate_label = new QLabel("no message");
//    //subscriber
//  //  chassis_sub = nh.subscribe<rr_msgs::chassis_state>("/chassis_state",1,ChassisPanel::chassisStateCallback);
//    layout->addWidget(speed_label);
//    layout->addWidget(heading_label);
//    layout->addWidget(boardstate_label);
//    setLayout(layout);
//}
//void ChassisPanel::chassisStateCallback(rr_msgs::chassis_state &msg) {
//    auto speed_text = std::to_string(msg.speed_mps) + "mph";
//    speed_label->setText(speed_text.c_str());
//    auto heading_text = std::to_string(msg.steer_rad) + "rad / " + std::to_string(180 / M_PI * msg.steer_rad) + "deg";
//    heading_label->setText(heading_text.c_str());
////    auto boardstate_text = msg.state;
////    boardstate_label->setText(boardstate_text.c_str());
//}
//}  // namespace rr_rviz_plugins
//
//PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::ChassisPanel, rviz::Panel)
//
// Created by nico on 11/3/19.
//
#include <pluginlib/class_list_macros.h>
#include <rr_rviz_plugins/ChassisPanel.h>
#include <std_msgs/Bool.h>
#include <QVBoxLayout>
#include <QtGui/QPaintEvent>
#include <QtGui/QPainter>
#include <QtWidgets/QLabel>
#include <rr_msgs/chassis_state.h>

namespace rr_rviz_plugins {
    bool received_signal = false;
    QLabel *speed_label;
    QLabel *heading_label;
    QLabel *msg_label;
    ros::Time now;
    ros::Time chassisUpdateTime = ros::Time::now();
    ros::Timer time;
    ChassisPanel::ChassisPanel(QWidget *parent)
            : rviz::Panel(parent)  // Base class constructor
    {
        auto *layout = new QVBoxLayout;
        speed_label = new QLabel("No Message");
        heading_label = new QLabel("No Message");
        msg_label = new QLabel("No Message");
        //speed_label->setGeometry(QRect(80, 0, 30, 30));
        chassis_sub = nh.subscribe<rr_msgs::chassis_state>("/chassis_state", 1, ChassisPanel::chassisStateCallback);
        time = nh.createTimer(ros::Duration(1), ChassisPanel::timerCallback);
        layout->addWidget(speed_label);
        layout->addWidget(heading_label);
        layout->addWidget(msg_label);
        setLayout(layout);
        this->setFixedHeight(100);
    }

// draws the stoplight
    void ChassisPanel::paintEvent(QPaintEvent *e) {

        QWidget::paintEvent(e);
    }

    void ChassisPanel::chassisStateCallback(const rr_msgs::chassis_state msg){
        chassisUpdateTime = ros::Time::now();
        received_signal = true;
        auto speed = std::to_string(msg.speed_mps) + " m/s";
        auto heading = std::to_string(msg.steer_rad) + "rad";
        auto message = msg.state;
        speed_label->setText(speed.c_str());
        heading_label->setText(heading.c_str());
        msg_label->setText(message.c_str());
    }

    void ChassisPanel::timerCallback(const ros::TimerEvent &e) {
        now = ros::Time::now();
        if (now - chassisUpdateTime > ros::Duration(1)) {
            received_signal = false;
            speed_label->setText("No Message");
        }
    }
}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::ChassisPanel, rviz::Panel)