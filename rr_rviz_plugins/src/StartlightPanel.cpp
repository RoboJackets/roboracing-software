//
// Created by nico on 11/3/19.
//
#include <pluginlib/class_list_macros.h>
#include <rr_rviz_plugins/StartlightPanel.h>
#include <std_msgs/Bool.h>
#include <QVBoxLayout>
#include <QtGui/QPaintEvent>
#include <QtGui/QPainter>
#include <QtWidgets/QLabel>

namespace rr_rviz_plugins {
bool isGreen = false;
bool receivedSignal = false;
QLabel *label;
ros::Time current;
ros::Time callbackTime = ros::Time::now();
ros::Timer timer;
StartlightPanel::StartlightPanel(QWidget *parent)
      : rviz::Panel(parent)  // Base class constructor
{
    auto *layout = new QVBoxLayout;
    label = new QLabel("              No Message");
    label->setGeometry(QRect(80, 0, 30, 30));
    start_detector = nh.subscribe<std_msgs::Bool>("/start_detected", 1, StartlightPanel::startlightCallback);
    timer = nh.createTimer(ros::Duration(1), StartlightPanel::timerCallback);
    layout->addWidget(label);
    setLayout(layout);
    this->setFixedHeight(50);
}

// draws the stoplight
void StartlightPanel::paintEvent(QPaintEvent *e) {
    int radius = 21;
    int posX = 16;
    int posY = 12;

    // Background rectangle
    QPainter painter1(this);
    painter1.setBrush(Qt::black);
    painter1.setPen(Qt::black);
    painter1.drawRect(posX - 1, posY - 1, 2 * radius + 5, radius + 2);

    // Left circle
    QPainter painter(this);
    if (isGreen) {
        painter.setPen(Qt::green);
        painter.setBrush(Qt::green);
    } else {
        painter.setPen(Qt::gray);
        painter.setBrush(Qt::gray);
    }
    if (!receivedSignal) {
        painter.setPen(Qt::gray);
        painter.setBrush(Qt::gray);
    }
    painter.drawEllipse(posX, posY, radius, radius);

    // Right circle
    if (!isGreen) {
        painter.setPen(Qt::red);
        painter.setBrush(Qt::red);
    } else {
        painter.setPen(Qt::gray);
        painter.setBrush(Qt::gray);
    }
    if (!receivedSignal) {
        painter.setPen(Qt::gray);
        painter.setBrush(Qt::gray);
    }
    painter.drawEllipse(posX + radius + 3, posY, radius, radius);

    QWidget::paintEvent(e);
}

void StartlightPanel::startlightCallback(const std_msgs::Bool msg) {
    callbackTime = ros::Time::now();
    receivedSignal = true;
    isGreen = msg.data != 0;
    if (isGreen) {
        label->setText("              Go!");
    } else {
        label->setText("              Stop!");
    }
}

void StartlightPanel::timerCallback(const ros::TimerEvent &e) {
    current = ros::Time::now();
    if (current - callbackTime > ros::Duration(1)) {
        receivedSignal = false;
        label->setText("              No Message");
    }
}
}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::StartlightPanel, rviz::Panel)