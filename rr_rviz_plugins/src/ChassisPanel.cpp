#include <pluginlib/class_list_macros.h>
#include <rr_msgs/chassis_state.h>
#include <rr_rviz_plugins/ChassisPanel.h>
#include <std_msgs/Bool.h>
#include <QVBoxLayout>
#include <QtGui/QPaintEvent>
#include <QtGui/QPainter>
#include <QtWidgets/QLabel>
#include <cmath>

namespace rr_rviz_plugins {
bool received_signal = false;
float cur_speed;
float steer_rad;
bool record_bag;
bool estop;
QLabel *speed_label;
QLabel *heading_label;
QLabel *msg_label;
QLabel *rcding_bag_label;
QLabel *estop_label;
ros::Time now;
ros::Timer time;
ros::Time chassisUpdateTime = ros::Time::now();
const QString spacing = "          ";
ChassisPanel::ChassisPanel(QWidget *parent)
      : rviz::Panel(parent)  // Base class constructor
{
    auto *layout = new QVBoxLayout;

    speed_label = new QLabel("");
    heading_label = new QLabel("");
    msg_label = new QLabel("No Message");
    rcding_bag_label = new QLabel("");
    estop_label = new QLabel("");

    chassis_sub = nh.subscribe<rr_msgs::chassis_state>("/chassis_state", 1, ChassisPanel::chassisStateCallback);
    time = nh.createTimer(ros::Duration(1), ChassisPanel::timerCallback);

    layout->addWidget(speed_label);
    layout->addWidget(heading_label);
    layout->addWidget(msg_label);
    layout->addWidget(rcding_bag_label);
    layout->addWidget(estop_label);
    setLayout(layout);

    this->setFixedHeight(150);
    this->setFixedWidth(340);
}

// draws the images
void ChassisPanel::paintEvent(QPaintEvent *e) {
    if (received_signal) {  // will only paint if receiving a signal
        // initial positions / dimensions for the speedometer
        const QRect &rct = e->rect();
        int x_pos = 10;
        int y_pos = 9;
        int width = 30;
        int height = 38;
        QRectF rect = QRectF(x_pos, y_pos, width, height);
        QPainter painter(this);
        painter.eraseRect(rct);
        painter.setRenderHint(QPainter::Antialiasing);
        // sets the color and draws the speedometer
        painter.setBrush(Qt::gray);
        painter.setPen(Qt::gray);
        painter.drawEllipse(rect);
        painter.drawChord(rect, 0, 5760 / 2);
        // erases half of the ellipse so it becomes a semicircle and looks like a speedometer
        painter.eraseRect(x_pos, y_pos + (height / 2), width + 2, height / 2 + 2);
        // top speed displayed by the speedometer
        float top_speed = 40;
        // displaying speed as positive, even for in reverse
        cur_speed = std::abs(cur_speed);
        // only changes the speed shown by the speedometer, so it does not go below its half circle,
        // does not affect what the label says
        if (cur_speed >= top_speed) {
            cur_speed = top_speed;
        }
        // the angle the speedometer needle needs to point in
        float angle = M_PI * (1.0 - cur_speed / top_speed);
        // determines x and y height of the needle
        float x_comp = cos(angle) * width / 2;
        float y_comp = sin(angle) * height / 2;
        // the shape and direction of the needle
        QLineF needle = QLineF(width / 2 + x_pos, y_pos + height / 2 - 1, width / 2 + x_comp + x_pos,
                               y_pos + height / 2 - y_comp);
        painter.setBrush(Qt::red);
        painter.setPen(Qt::red);
        painter.drawLine(needle);
        // new params for the heading arrow
        y_pos = 38;
        height = 18;
        // determines the rectangle shape the arrow will fit in
        rect = QRectF(x_pos, y_pos, width, height);
        painter.setPen(QColor(255, 255, 255, 255));
        painter.setBrush(QColor(255, 255, 255, 255));
        painter.drawRect(rect);
        painter.setBrush(Qt::black);
        painter.setPen(Qt::black);
        int x_org = x_pos + width / 2;
        int y_org = y_pos + height / 2;
        // The size of the arrow
        int arr_size = 9;  // this is actually  half the length of the arrow
        // makes it so that zero points north instead of east
        steer_rad = steer_rad + M_PI / 2;
        y_comp = sin(steer_rad) * (float)arr_size;
        x_comp = cos(steer_rad) * (float)arr_size;

        QLineF arrow_shaft = QLineF(x_org - x_comp, y_org + y_comp, x_org + x_comp, y_org - y_comp);
        painter.drawLine(arrow_shaft);

        // For the arrowhead
        painter.setPen(Qt::red);
        painter.setBrush(Qt::red);
        QLineF arrow_tip = QLineF(x_org + x_comp, y_org - y_comp, x_org + .5 * x_comp, y_org - .5 * y_comp);
        painter.drawLine(arrow_tip);

        // For the status of recording_bag
        x_pos = 137;
        y_pos = 95;
        width = 40;
        height = 15;
        if (received_signal && record_bag) {
            painter.setBrush(Qt::green);
            painter.setPen(Qt::green);
            painter.drawRect(x_pos, y_pos, width, height);
        } else if (received_signal) {
            painter.setBrush(Qt::red);
            painter.setPen(Qt::red);
            painter.drawRect(x_pos, y_pos, width, height);
        }

        // for the status of estop
        x_pos = 52;
        y_pos = 122;
        width = 23;
        if (estop) {
            painter.setBrush(Qt::red);
            painter.setPen(Qt::red);
            painter.drawRect(x_pos, y_pos, width, height);
        } else {
            painter.setBrush(Qt::green);
            painter.setPen(Qt::green);
            painter.drawRect(x_pos, y_pos, width, height);
        }
        QWidget::paintEvent(e);
    }
}

void ChassisPanel::chassisStateCallback(const rr_msgs::chassis_state msg) {
    // for timerCallback
    chassisUpdateTime = ros::Time::now();
    received_signal = true;

    // global variables
    cur_speed = msg.speed_mps;
    steer_rad = msg.steer_rad;
    record_bag = msg.record_bag;
    estop = msg.estop_on;

    // label updates
    std::string speed_str = spacing.toStdString() + std::to_string(msg.speed_mps) + " m/s";
    std::string heading_str = spacing.toStdString() + std::to_string(msg.steer_rad) + " rad";
    std::string message_str = msg.state;
    speed_label->setText(speed_str.c_str());
    heading_label->setText(heading_str.c_str());
    msg_label->setText(message_str.c_str());

    if (msg.record_bag) {
        rcding_bag_label->setText("Recording Bag File:  true");
    } else {
        rcding_bag_label->setText("Recording Bag File:  false");
    }
    if (msg.estop_on) {
        estop_label->setText("Estop: on");
    } else {
        estop_label->setText("Estop: off");
    }
}

void ChassisPanel::timerCallback(const ros::TimerEvent &e) {
    // latency for detecting if its no longer sending a message
    if (ros::Time::now() - chassisUpdateTime > ros::Duration(.5)) {
        received_signal = false;
        cur_speed = 0;
        steer_rad = 0;
        speed_label->setText(spacing + "");
        heading_label->setText(spacing + "");
        msg_label->setText("Not Recently Updated");
        rcding_bag_label->setText("");
        estop_label->setText("");
    }
}

}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::ChassisPanel, rviz::Panel)