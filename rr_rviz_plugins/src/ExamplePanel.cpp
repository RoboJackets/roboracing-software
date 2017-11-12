#include <rr_rviz_plugins/ExamplePanel.h>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>

namespace rr_rviz_plugins {

ExamplePanel::ExamplePanel(QWidget *parent)
  : rviz::Panel(parent)
{
  ros::NodeHandle handle;

  QLabel *label = new QLabel("0 m/s");

  speed_subscriber = handle.subscribe<rr_platform::speed>("/speed", 1, boost::bind(&ExamplePanel::speed_callback, this, _1, label));

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(label);
  setLayout(layout);
}

void ExamplePanel::speed_callback(const rr_platform::speedConstPtr &msg, QLabel *label) {
    auto text = std::to_string(msg->speed) + " m/s";
    label->setText(text.c_str());
}

}

PLUGINLIB_EXPORT_CLASS( rr_rviz_plugins::ExamplePanel, rviz::Panel)
