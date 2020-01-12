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
int cur_speed;
float hding;
QLabel *speed_label;
QLabel *heading_label;
QLabel *msg_label;
QLabel *rcding_bag_label;
ros::Time now;
ros::Time chassisUpdateTime = ros::Time::now();
ros::Timer time;
QString spacing = "          ";
ChassisPanel::ChassisPanel(QWidget *parent)
      : rviz::Panel(parent)  // Base class constructor
{
    auto *layout = new QVBoxLayout;
    speed_label = new QLabel(spacing + "No Message");
    heading_label = new QLabel(spacing + "No Message");
    msg_label = new QLabel("No Message");
    rcding_bag_label = new QLabel("No Message");
    chassis_sub = nh.subscribe<rr_msgs::chassis_state>("/chassis_state", 1, ChassisPanel::chassisStateCallback);
    time = nh.createTimer(ros::Duration(1), ChassisPanel::timerCallback);
    layout->addWidget(speed_label);
    layout->addWidget(heading_label);
    layout->addWidget(msg_label);
    layout->addWidget(rcding_bag_label);
    setLayout(layout);
    this->setFixedHeight(120);
    this->setFixedWidth(340);
}

// draws the images
void ChassisPanel::paintEvent(QPaintEvent *e) {
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
    //top speed displayed by the speedometer
    int top_speed = 40;
    // the angle the speedometer needle needs to point in
    auto angle = (((top_speed - cur_speed) * M_PI) / top_speed);
    // determines x and y height of the needle
    float x_comp = cos(angle) * width / 2;
    float y_comp = sin(angle) * height / 2;
    // the shape and direction of the needle
    QLineF needle = QLineF(width / 2 + x_pos, y_pos + height / 2 - 1, width / 2 + x_comp + x_pos, y_pos + height / 2 - y_comp);
    painter.setBrush(Qt::red);
    painter.setPen(Qt::red);
    painter.drawLine(needle);
    // new params for the heading arrow
    y_pos = 37;
    height = 25;
    // determines the rectangle shape the arrow will fit in
    rect = QRectF(x_pos, y_pos, width, height);
    painter.setPen(QColor(255,255,255,255));
    painter.setBrush(QColor(255,255,255,255));
    painter.drawRect(rect);
    painter.setBrush(Qt::black);
    painter.setPen(Qt::black);
    int x_org = x_pos + width / 2;
    int y_org = y_pos + height / 2;
    // The size of the arrow
    int arr_size = 10;  // this is actually  half the length of the arrow
    // makes it so that zero points north instead of east
    hding = hding + M_PI / 2;
    y_comp = sin(hding) * (float)arr_size;
    x_comp = cos(hding) * (float)arr_size;

    QLineF arrow_shaft = QLineF(x_org - x_comp, y_org + y_comp, x_org + x_comp, y_org - y_comp);
    painter.drawLine(arrow_shaft);

    // For the arrowhead
    painter.setPen(Qt::red);
    painter.setBrush(Qt::red);
    QLineF arrow_tip = QLineF(x_org + x_comp, y_org - y_comp, x_org + .5 * x_comp, y_org - .5 * y_comp);
    QRectF arrow_head = QRectF(x_org + x_comp, y_org - y_comp, x_org + .5 * x_comp, y_org - .5 * y_comp);

    painter.drawLine(arrow_tip);
//    painter.drawRect(arrow_head);
    // actually paints everything from above
    QWidget::paintEvent(e);
}

void ChassisPanel::chassisStateCallback(const rr_msgs::chassis_state msg) {
    chassisUpdateTime = ros::Time::now();
    received_signal = true;
    cur_speed = msg.speed_mps;
    hding = msg.steer_rad;
    auto speed = spacing.toStdString() + std::to_string(msg.speed_mps) + " m/s";
    auto heading = spacing.toStdString() + std::to_string(msg.steer_rad) + " rad";
    auto message = msg.state;
    speed_label->setText(speed.c_str());
    heading_label->setText(heading.c_str());
    msg_label->setText(message.c_str());

    if (msg.record_bag) {
        rcding_bag_label->setText("Recording Bag File: true");
        //rcding_bag_label->setStyleSheet("QLabel { background-color : green}");
    } else {
        rcding_bag_label->setText("Recording Bag File: false");
        //rcding_bag_label->setStyleSheet("QLabel { background-color : red}");
    }
}

void ChassisPanel::timerCallback(const ros::TimerEvent &e) {
    now = ros::Time::now();
    if (now - chassisUpdateTime > ros::Duration(.5)) {  // latency for detecting if its no longer sending a message
        received_signal = false;
        cur_speed = 0;
        hding = 0;
        speed_label->setText(spacing + "No Message");
        heading_label->setText(spacing + "No Message");
        msg_label->setText("No Message");
        rcding_bag_label->setText("No Message");
    }
}
}  // namespace rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::ChassisPanel, rviz::Panel)