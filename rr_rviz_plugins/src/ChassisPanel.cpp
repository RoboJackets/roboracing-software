#include <pluginlib/class_list_macros.h>
#include <rr_rviz_plugins/ChassisPanel.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <QVBoxLayout>
#include <QtGui/QPaintEvent>
#include <QtGui/QPainter>
#include <QtWidgets/QLabel>
#include <rr_msgs/chassis_state.h>

namespace rr_rviz_plugins {
    bool received_signal = false;
    int mph;
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
        speed_label = new QLabel("      No Message");
        speed_label->setGeometry(QRect(80, 0, 30, 30));
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

// draws the images
    void ChassisPanel::paintEvent(QPaintEvent *e) {
        int x_pos = 1;
        int y_pos = 7;
        int width = 30;
        int height = 40;
        QRectF rect = QRectF(x_pos,y_pos,width,height);
        QWidget::paintEvent(e);
        QPainter painter(this);
        painter.setBrush(Qt::gray);
        painter.setPen(Qt::black);
        painter.drawEllipse(rect);
        painter.eraseRect(x_pos,y_pos + (height / 2), width + 2,height / 2 + 2);
        painter.drawChord(rect,0,5760 / 2);
        painter.setBrush(Qt::red);
        painter.setBrush(Qt::red);
        //for drawChord and drawArc 5760 = 360degrees
        int top_speed = 60; //sets the max speed displayed by the speedometer
        auto angle = (((top_speed - mph) * M_PI) / top_speed); //the angle the speedometer needle needs to point in
        auto x_comp = cos(angle) * width / 2;
        auto y_comp = sin(angle) * height / 2;
        QLineF needle = QLineF(width / 2, y_pos + height / 2, width / 2 + x_comp, y_pos + height / 2 + y_comp); //The shape and position of the needle
        painter.drawLine(needle);


        QWidget::paintEvent(e);
    }

    void ChassisPanel::chassisStateCallback(const rr_msgs::chassis_state msg){
        chassisUpdateTime = ros::Time::now();
        received_signal = true;
        mph = msg.speed_mps;
        auto speed = "      " + std::to_string(msg.speed_mps) + " m/s";
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
            speed_label->setText("      No Message");
        }
    }
}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::ChassisPanel, rviz::Panel)