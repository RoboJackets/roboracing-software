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
ros::Time startlightUpdateTime = ros::Time::now();
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
    int diameter = 21;
    int posX = 16;
    int posY = 12;
    auto green = Qt::green;
    auto red = Qt::red;
    auto gray = Qt::gray;
    auto left_color = gray;
    auto right_color = gray;

    // Background rectangle
    QPainter painter(this);
    painter.setBrush(Qt::black);
    painter.setPen(Qt::black);
    painter.drawRect(posX - 1, posY - 1, 2 * diameter + 5, diameter + 2);

    // Left circle
    if (isGreen && receivedSignal) {
        left_color = green;
    }
    painter.setPen(left_color);
    painter.setBrush(left_color);
    painter.drawEllipse(posX, posY, diameter, diameter);

    // Right circle
    if (!isGreen && receivedSignal) {
        right_color = red;
    }
    painter.setPen(right_color);
    painter.setBrush(right_color);
    painter.drawEllipse(posX + diameter + 3, posY, diameter, diameter);

    QWidget::paintEvent(e);
}

void StartlightPanel::startlightCallback(const std_msgs::Bool msg) {
    startlightUpdateTime = ros::Time::now();
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
    if (current - startlightUpdateTime > ros::Duration(1)) {
        receivedSignal = false;
        label->setText("              No Message");
    }
}
}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::StartlightPanel, rviz::Panel)