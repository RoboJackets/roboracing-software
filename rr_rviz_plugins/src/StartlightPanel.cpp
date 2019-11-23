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
    StartlightPanel::StartlightPanel(QWidget *parent)
            : rviz::Panel(parent)  // Base class constructor
    {
        label = new QLabel("        No Message");
        label->setMinimumHeight(30);
        label->setMinimumWidth(130);
        start_detector = nh.subscribe<std_msgs::Bool>("/start_detected", 1, startlightCallback);
        auto *layout = new QVBoxLayout;
        QWidget::repaint();

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
        receivedSignal = true;
        isGreen = msg.data != 0;
        if (isGreen) {
            label->setText("        Go!");
        } else {
            label->setText("        Stop!");
        }
    }
}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::StartlightPanel, rviz::Panel)