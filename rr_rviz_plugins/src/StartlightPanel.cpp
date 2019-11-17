//
// Created by nico on 11/3/19.
//
#include <pluginlib/class_list_macros.h>
#include <rr_rviz_plugins/StartlightPanel.h>
#include <rr_rviz_plugins/ExamplePanel.h>
#include <std_msgs/Bool.h>
#include <QVBoxLayout>
#include <QPainter>

namespace rr_rviz_plugins {
    static bool isGreen = false;
    static bool recievedSignal = false;
    StartlightPanel::StartlightPanel(QWidget *parent)
    : rviz::Panel(parent) {

        ros::NodeHandle nh;
        auto *label = new QLabel("No Message");
        start_detector = nh.subscribe<std_msgs::Bool>("/start_detected", 1, startlight_callback);

        //Creates painter to make the circle
        QPainter painter;
        //Makes the pen that the painter uses
        QPen myPen;

        //sets the color of the pen
        if (isGreen) {
            myPen.setColor(Qt::green);
        } else {
            myPen.setColor(Qt::red);
        }
        if (!recievedSignal) {
            myPen.setColor(Qt::gray);
        }

        //makes a layout
        QVBoxLayout *layout = new QVBoxLayout;
        //draws the circle
        painter.drawEllipse(0, 0, 100, 100);
        setLayout(layout);
    }
    static void startlight_callback(const std_msgs::Bool msg/*, QLabel *label*/){
        //auto text = std::to_string(msg.data);
        //label->setText(text.c_str());
        recievedSignal = true;
        isGreen = msg.data != 0;
    }
//This is literally just example panel
} // end of rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::StartlightPanel, rviz::Panel);