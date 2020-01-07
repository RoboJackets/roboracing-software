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
    float hding;
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
        heading_label = new QLabel("      No Message");
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
        //initial positions / dimensions for the speedometer
        const QRect & rct = e->rect();
        int x_pos = 1;
        int y_pos = 7;
        int width = 30;
        int height = 40;
        QRectF rect = QRectF(x_pos,y_pos,width,height);
        QPainter painter(this);
        painter.eraseRect(rct);
        painter.setRenderHint(QPainter::Antialiasing);
        //sets the color and draws the speedometer
        painter.setBrush(Qt::gray);
        painter.setPen(Qt::gray);
        painter.drawEllipse(rect);
        painter.drawChord(rect,0,5760 / 2);
        //erases half of the ellipse so it becomes a semicircle and looks like a speedometer
        painter.eraseRect(x_pos,y_pos + (height / 2), width + 2,height / 2 + 2);
        //for drawChord and drawArc 5760 = 360 degrees
        //sets the max speed displayed by the speedometer
        int top_speed = 35;
        //the angle the speedometer needle needs to point in
        auto angle = (((top_speed - mph) * M_PI) / top_speed);
        //determines x and y height of the needle
        float x_comp = cos(angle) * width / 2;
        float y_comp = sin(angle) * height / 2;
        //the shape and direction of the needle
        QLineF needle = QLineF(width / 2,
                y_pos + height / 2 - 1,
                width / 2 + x_comp,
                y_pos + height / 2 - y_comp);
        painter.setBrush(Qt::red);
        painter.setPen(Qt::red);
        //draws the needle
        painter.drawLine(needle);

        //sets the new positions of the heading arrow
        y_pos = 36;
        height = 25;
        //determines the rectangle shape the arrow will fit in
        rect = QRectF(x_pos,y_pos,width,height);
        painter.setPen(Qt::white);
        painter.setBrush(Qt::white);
        painter.drawRect(rect);
        painter.setBrush(Qt::black);
        painter.setPen(Qt::black);

//        painter.drawRect(rect);
        //For the directional arrow to show the heading
        //the x component of the origin
        int x_org = x_pos + width / 2;
        int y_org = y_pos + height / 2;
        //The size of the arrow
        int arr_size = 10; //this is actually  half the length of the arrow

        y_comp = sin(hding) * (float) arr_size;
        x_comp = cos(hding) * (float) arr_size;

        QLineF arrow_shaft = QLineF(x_org - x_comp, y_org + y_comp, x_org + x_comp, y_org - y_comp);

        painter.drawLine(arrow_shaft);

        painter.setPen(Qt::white);
        painter.setBrush(Qt::white);


        //For the arrowhead
        //the angle it juts out at
        float pivot = M_PI / 6;

        //actually paints everything from above
        QWidget::paintEvent(e);
    }

    void ChassisPanel::chassisStateCallback(const rr_msgs::chassis_state msg){
        chassisUpdateTime = ros::Time::now();
        received_signal = true;
        mph = msg.speed_mps;
        hding = msg.steer_rad;
        auto speed = "      " + std::to_string(msg.speed_mps) + " m/s";
        auto heading = "      " + std::to_string(msg.steer_rad) + " rad";
        auto message = msg.state;
        speed_label->setText(speed.c_str());
        heading_label->setText(heading.c_str());
        msg_label->setText(message.c_str());
    }

    void ChassisPanel::timerCallback(const ros::TimerEvent &e) {
        now = ros::Time::now();
        if (now - chassisUpdateTime > ros::Duration(.5)) { //latency for detecting if its no longer sending a message
            received_signal = false;
            mph = 0;
            hding = 0;
            speed_label->setText("      No Message");
            heading_label->setText("      No Message");
            msg_label->setText("No Message");
        }
    }
}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::ChassisPanel, rviz::Panel)