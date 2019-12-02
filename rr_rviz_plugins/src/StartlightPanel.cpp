//
// Created by nico on 11/3/19.
//
#include <pluginlib/class_list_macros.h>
#include <rr_rviz_plugins/StartlightPanel.h>
#include <std_msgs/Bool.h>
#include <QVBoxLayout>
#include <QtWidgets/QLabel>
#include <QtGui/QPainter>
#include <QtGui/QPaintEvent>

namespace rr_rviz_plugins {
    bool isGreen = false;
    bool receivedSignal = false;
    QLabel *label;
    ros::Time current;
    ros::Time callbackTime;
    ros::Timer timer;

    StartlightPanel::StartlightPanel(QWidget *parent)
            : rviz::Panel(parent)  // Base class constructor
    {
        label = new QLabel("        No Message");
        label->setFixedHeight(30);
        label->setFixedHeight(30);
        label->setGeometry(QRect(0, 0, 30, 30));

        start_detector = nh.subscribe<std_msgs::Bool>("/start_detected", 1,
                StartlightPanel::startlightCallback);
        timer = nh.createTimer(ros::Duration(1), StartlightPanel::timerCallback);

        auto *layout = new QVBoxLayout;
        layout->addWidget(label);
        setLayout(layout);
    }
    //draws the circle
    void StartlightPanel::paintEvent(QPaintEvent *e) {
        QPainter painter(this);

        if (isGreen) {
            painter.setPen(Qt::green);
            painter.setBrush(Qt::green);
        } else {
            painter.setPen(Qt::red);
            painter.setBrush(Qt::red);
        }
        if (!receivedSignal) {
            painter.setPen(Qt::gray);
            painter.setBrush(Qt::gray);
        }

        int radius = 21;
        painter.drawEllipse(16, 12, radius, radius);
        QWidget::paintEvent(e);
    }

    void StartlightPanel::startlightCallback(const std_msgs::Bool msg){

        if(!timer.hasStarted()) {
            timer.start();
        }

        callbackTime = ros::Time::now();
        receivedSignal = true;
        isGreen = msg.data != 0;

        if (isGreen) {
            label->setText("        Go!");
        } else {
            label->setText("        Stop!");
        }
    }

    void StartlightPanel::timerCallback(const ros::TimerEvent& e) {
        current = ros::Time::now();

        if (current - callbackTime > ros::Duration(1)) {
            receivedSignal = false;
            label->setText("        No Message");
        }
        timer.stop();
    }
}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::StartlightPanel, rviz::Panel)