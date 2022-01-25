#include <pluginlib/class_list_macros.h>
#include <rr_msgs/speed.h>
#include <rr_rviz_plugins/Speedometer.h>

#include <QVBoxLayout>
#include <QtGui/QPaintEvent>
#include <cmath>

namespace rr_rviz_plugins {

ros::NodeHandle handle;
ros::Subscriber speed_subscriber;

float current_speed;

QLabel *label;

Speedometer::Speedometer(QWidget *parent)
      : rviz_common::Panel(parent)  // Base class constructor
{
    // Initialize a label for displaying some data
    label = new QLabel("0 m/s");

    /* Initialize our subscriber to listen to the /speed topic.
     * Note the use of boost::bind, which allows us to give the callback a pointer to our UI label.
     */
    speed_subscriber =
          handle.subscribe<rr_msgs::speed>("/speed", 1, boost::bind(&Speedometer::speedCallback, this, _1, label));

    /* Use QT layouts to add widgets to the panel.
     * Here, we're using a VBox layout, which allows us to stack all of our
     * widgets vertically.
     */
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    setLayout(layout);
}

void Speedometer::speedCallback(const rr_msgs::speed &msg, QLabel *label) {
    // Create the new contents of the label based on the speed message.
    current_speed = std::abs(msg.speed);
    auto text = std::to_string(current_speed) + " m/s";
    // Set the contents of the label.
    label->setText(text.c_str());
}

// create image of speedometer
void Speedometer::paintEvent(QPaintEvent *event)  {
    //draw a speedometer
    float max_speed = 40;
    float adjusted_speed = std::min(max_speed, current_speed);
    float angle = M_PI * (1.0 - current_speed / max_speed);
}

}

PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::Speedometer, rviz::Panel)