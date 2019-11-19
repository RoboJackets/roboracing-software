//
// Created by nico on 11/3/19.
//
//#include <pluginlib/class_list_macros.h>
//#include <rr_rviz_plugins/StartlightPanel.h>
//#include <rr_rviz_plugins/ExamplePanel.h>
//#include <std_msgs/Bool.h>
//#include <QVBoxLayout>
//#include <QPainter>
//
//namespace rr_rviz_plugins {
//    static bool isGreen = false;
//    static bool recievedSignal = false;
//    StartlightPanel::StartlightPanel(QWidget *parent)
//    : rviz::Panel(parent) {
//
//        ros::NodeHandle nh;
//        auto *label = new QLabel("No Message");
//        start_detector = nh.subscribe<std_msgs::Bool>("/start_detected", 1, startlight_callback);
//
//        //Creates painter to make the circle
//        QPainter painter;
//        //Makes the pen that the painter uses
//        QPen myPen;
//
//        //sets the color of the pen
//        if (isGreen) {
//            myPen.setColor(Qt::green);
//        } else {
//            myPen.setColor(Qt::red);
//        }
//        if (!recievedSignal) {
//            myPen.setColor(Qt::gray);
//        }
//
//        //makes a layout
//        QVBoxLayout *layout = new QVBoxLayout;
//        //draws the circle
//        painter.drawEllipse(0, 0, 100, 100);
//        setLayout(layout);
//    }
//    static void startlight_callback(const std_msgs::Bool msg/*, QLabel *label*/){
//        //auto text = std::to_string(msg.data);
//        //label->setText(text.c_str());
//        recievedSignal = true;
//        isGreen = msg.data != 0;
//    }
////This is literally just example panel
//} // end of rr_rviz_plugins
#include <pluginlib/class_list_macros.h>
#include <rr_rviz_plugins/StartlightPanel.h>
#include <std_msgs/Bool.h>
#include <QVBoxLayout>
#include <QtWidgets/QLabel>
#include <QtGui/QPainter>

namespace rr_rviz_plugins {
    bool isGreen = false;
    bool receivedSignal = false;
    QLabel *label;
    StartlightPanel::StartlightPanel(QWidget *parent)
            : rviz::Panel(parent)  // Base class constructor
    {
//        reset_btn = new QPushButton("Reset!");
//        reset_pub = nh.advertise<rr_msgs::race_reset>("/reset_detected", 0);
//        connect(reset_btn, SIGNAL(released()), this, SLOT(resetCallback()));
        label = new QLabel("No Message");
        start_detector = nh.subscribe<std_msgs::Bool>("/start_detected", 1, startlightCallback);

        //Creates painter to make the circle
//        QPaintEvent screen;
        QPainter painter;

        //Makes the pen that the painter uses
        QPen pen;

        //Sets the color of the pen
        if (isGreen) {
            pen.setColor(Qt::green);
        } else {
            pen.setColor(Qt::red);
        }
        if(!receivedSignal) {
            pen.setColor(Qt::gray);
        }

        //makes a layout
        auto *layout = new QVBoxLayout;

        //draws the circle
        painter.drawEllipse(0, 0, 10, 10);
//        layout->addWidget(reset_btn);
        layout->addWidget(label);
        setLayout(layout);
    }
    void StartlightPanel::paintEvent(QPaintEvent *) {
        QPainter painter(this);
        if (isGreen) {
            painter.setPen(Qt::green);
        } else {
            painter.setPen(Qt::red);
        }
        if (!receivedSignal) {
            painter.setPen(Qt::gray);
        }
    }
//    void StartlightPanel::resetCallback() {
//        rr_msgs::race_reset reset;
//        reset_pub.publish(reset);
//    }

    void StartlightPanel::startlightCallback(const std_msgs::Bool msg){
//        auto text = std::to_string(msg.data);
//        label->setText(text.c_str());
        receivedSignal = true;
        isGreen = msg.data != 0;
        if (isGreen) {
            label->setText("Green");
        } else {
            label->setText("Red");
        }
    }
}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::StartlightPanel, rviz::Panel)